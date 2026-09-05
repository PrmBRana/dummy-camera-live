/****************************************************************************
 * drivers/analog/ads7953.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads7953.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_ADC_ADS7953)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ADS7953_FREQUENCY
#  define CONFIG_ADS7953_FREQUENCY 10000000
#endif

/* Device uses SPI Mode 0: CPOL = 0, CPHA = 0 */

#define ADS7953_SPI_MODE          (SPIDEV_MODE0)
#define ADS7953_SPI_BITS          16

/* ADS7953 Protocol Definitions */

#define ADS7953_CMD_CONTINUE      (0x0000)
#define ADS7953_CMD_MANUAL        (0x1 << 12)
#define ADS7953_CMD_RESET         (0x4 << 12)

#define ADS7953_RANGE_BIT         (1 << 11)
#define ADS7953_CHANNEL_SHIFT     7

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads7953_dev_s
{
  FAR const struct adc_callback_s *cb;
  FAR struct spi_dev_s            *spi;
  uint32_t                         devno;
  mutex_t                          lock;
  uint8_t                          range;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ads7953_configspi(FAR struct spi_dev_s *spi);
static int  ads7953_readall(FAR struct adc_dev_s *dev);

/* ADC operations */

static int  ads7953_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback);
static void ads7953_reset(FAR struct adc_dev_s *dev);
static int  ads7953_setup(FAR struct adc_dev_s *dev);
static void ads7953_shutdown(FAR struct adc_dev_s *dev);
static void ads7953_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  ads7953_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  ads7953_bind,      /* ao_bind */
  ads7953_reset,     /* ao_reset */
  ads7953_setup,     /* ao_setup */
  ads7953_shutdown,  /* ao_shutdown */
  ads7953_rxint,     /* ao_rxint */
  ads7953_ioctl      /* ao_ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7953_configspi
 ****************************************************************************/

static void ads7953_configspi(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, ADS7953_SPI_MODE);
  SPI_SETBITS(spi, ADS7953_SPI_BITS);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ADS7953_FREQUENCY);
}


/****************************************************************************
 * Name: ads7953_readall
 *
 * Description:
 *   Reads all 16 channels in Manual Mode taking into account the 2-frame
 *   pipeline latency:
 *     Cycle 0: Request CH0
 *     Cycle 1: Request CH1
 *     Cycle 2: Request CH2  -> DOUT yields CH0
 *     ...
 *     Cycle 15: Request CH15 -> DOUT yields CH13
 *     Cycle 16: Continue     -> DOUT yields CH14
 *     Cycle 17: Continue     -> DOUT yields CH15
 ****************************************************************************/

static int ads7953_readall(FAR struct adc_dev_s *dev)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;
  FAR struct spi_dev_s *spi = priv->spi;
  uint16_t cmd;
  uint16_t dout;
  uint8_t ch;
  int32_t val;
  int cycle;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  SPI_LOCK(spi, true);
  ads7953_configspi(spi);

  for (cycle = 0; cycle < (ADS7953_NCHANNELS + 2); cycle++)
    {
      if (cycle < ADS7953_NCHANNELS)
        {
          /* Manual Mode command for channel (cycle) */

          cmd = ADS7953_CMD_MANUAL |
                ((uint16_t)(priv->range & 0x01) << 11) |
                ((uint16_t)(cycle & 0x0F) << ADS7953_CHANNEL_SHIFT);
        }
      else
        {
          /* Trailing cycles to drain pipeline */

          cmd = ADS7953_CMD_CONTINUE;
        }

      SPI_SELECT(spi, priv->devno, true);
      dout = (uint16_t)SPI_SEND(spi, cmd);
      SPI_SELECT(spi, priv->devno, false);

      /* Pipeline yields valid sample from cycle 2 onward */

      if (cycle >= 2 && priv->cb != NULL)
        {
          ch  = (dout >> 12) & 0x0F;
          val = dout & 0x0FFF;

          priv->cb->au_receive(dev, ch, val);
        }
    }

  SPI_LOCK(spi, false);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: ads7953_bind
 ****************************************************************************/

static int ads7953_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: ads7953_reset
 ****************************************************************************/

static void ads7953_reset(FAR struct adc_dev_s *dev)
{
  /* No hardware reset on registration; hardware transactions
   * are performed on-demand via ANIOC_TRIGGER.
   */
}

/****************************************************************************
 * Name: ads7953_setup
 ****************************************************************************/

static int ads7953_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: ads7953_shutdown
 ****************************************************************************/

static void ads7953_shutdown(FAR struct adc_dev_s *dev)
{
  /* No special shutdown required for ADS7953 */
}

/****************************************************************************
 * Name: ads7953_rxint
 ****************************************************************************/

static void ads7953_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: ads7953_ioctl
 ****************************************************************************/

static int ads7953_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct ads7953_dev_s *priv = (FAR struct ads7953_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        ret = ads7953_readall(dev);
        break;

      case ANIOC_GET_NCHANNELS:
        ret = ADS7953_NCHANNELS;
        break;

      case ANIOC_ADS7953_SET_RANGE:
        if (arg == ADS7953_RANGE_1X || arg == ADS7953_RANGE_2X)
          {
            priv->range = (uint8_t)arg;
          }
        else
          {
            ret = -EINVAL;
          }
        break;

      default:
        aerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7953_register
 ****************************************************************************/

int ads7953_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     uint32_t devno)
{
  FAR struct ads7953_dev_s *priv;
  FAR struct adc_dev_s *adcdev;
  int ret;

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(spi != NULL);

  priv = kmm_zalloc(sizeof(struct ads7953_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate ads7953_dev_s\n");
      return -ENOMEM;
    }

  priv->spi   = spi;
  priv->devno = devno;
  priv->range = ADS7953_RANGE_2X; /* Default to 5.0V range */

  nxmutex_init(&priv->lock);

  adcdev = kmm_zalloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      aerr("ERROR: Failed to allocate adc_dev_s\n");
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return -ENOMEM;
    }

  adcdev->ad_ops  = &g_adcops;
  adcdev->ad_priv = priv;

  ret = adc_register(devpath, adcdev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to register %s: %d\n", devpath, ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      kmm_free(adcdev);
    }

  return ret;
}

#endif /* CONFIG_ADC_ADS7953 */
