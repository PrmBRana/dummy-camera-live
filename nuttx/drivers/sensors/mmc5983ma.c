/****************************************************************************
 * drivers/sensors/mmc5983ma.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA 3-Axis Magnetometer Driver
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/mmc5983ma.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mmc5983ma_config_s g_mmc5983ma_cfg =
{
  .frequency = 10000000,
  .mode      = SPIDEV_MODE0,
  .bits      = 8,
  .cs        = SPIDEV_USER(0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmc5983ma_write_reg
 ****************************************************************************/

static void mmc5983ma_write_reg(FAR struct spi_dev_s *spi,
                                uint8_t reg,
                                uint8_t value)
{
  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, g_mmc5983ma_cfg.mode);
  SPI_SETBITS(spi, g_mmc5983ma_cfg.bits);
  SPI_SETFREQUENCY(spi, g_mmc5983ma_cfg.frequency);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, true);

  SPI_SEND(spi, reg & 0x7f);
  SPI_SEND(spi, value);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, false);

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: mmc5983ma_read_reg
 ****************************************************************************/

static uint8_t mmc5983ma_read_reg(FAR struct spi_dev_s *spi,
                                  uint8_t reg)
{
  uint8_t value;

  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, g_mmc5983ma_cfg.mode);
  SPI_SETBITS(spi, g_mmc5983ma_cfg.bits);
  SPI_SETFREQUENCY(spi, g_mmc5983ma_cfg.frequency);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, true);

  SPI_SEND(spi, reg | 0x80);
  value = SPI_SEND(spi, 0xff);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, false);

  SPI_LOCK(spi, false);

  return value;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmc5983ma_read_id
 ****************************************************************************/

int mmc5983ma_read_id(FAR struct spi_dev_s *spi,
                      FAR uint8_t *id)
{
  if (spi == NULL || id == NULL)
    {
      return -EINVAL;
    }

  *id = mmc5983ma_read_reg(spi, MMC5983MA_REG_PRODUCT_ID);
  return OK;
}

/****************************************************************************
 * Name: mmc5983ma_initialize
 ****************************************************************************/

int mmc5983ma_initialize(FAR struct spi_dev_s *spi,
                         FAR const struct mmc5983ma_config_s *config)
{
  uint8_t id;

  if (spi == NULL)
    {
      return -EINVAL;
    }

  /* Save device configuration if provided */

  if (config != NULL)
    {
      g_mmc5983ma_cfg = *config;
      if (g_mmc5983ma_cfg.frequency == 0)
        {
          g_mmc5983ma_cfg.frequency = 10000000;
        }

      if (g_mmc5983ma_cfg.bits == 0)
        {
          g_mmc5983ma_cfg.bits = 8;
        }
    }

  /* 1. Verify Product ID (0x30) */

  id = mmc5983ma_read_reg(spi, MMC5983MA_REG_PRODUCT_ID);
  if (id != MMC5983MA_PRODUCT_ID_VALUE)
    {
      return -ENODEV;
    }

  /* 2. Execute SET pulse to restore magnetization alignment */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, MMC5983MA_CMD_SET);
  up_udelay(1000);

  /* 3. Initialize Control registers to default state */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, 0x00);
  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_1, 0x00);
  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_2, 0x00);
  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_3, 0x00);

  return OK;
}

/****************************************************************************
 * Name: mmc5983ma_read
 ****************************************************************************/

int mmc5983ma_read(FAR struct spi_dev_s *spi,
                   FAR struct mmc5983ma_data_s *data)
{
  uint8_t buffer[7];
  int timeout = 50;

  if (spi == NULL || data == NULL)
    {
      return -EINVAL;
    }

  /* 1. Trigger magnetic measurement (TM_M) */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, MMC5983MA_CMD_TAKE_MEAS_M);

  /* 2. Wait until measurement completes (Status bit 0 = Meas_M_Done) */

  while (timeout-- > 0)
    {
      uint8_t status = mmc5983ma_read_reg(spi, MMC5983MA_REG_STATUS);
      if (status & MMC5983MA_STATUS_MEAS_M_DONE)
        {
          break;
        }

      up_udelay(500);
    }

  if (timeout <= 0)
    {
      return -ETIMEDOUT;
    }

  /* 3. Burst read 7 bytes: XOUT[0..1], YOUT[0..1], ZOUT[0..1], XYZOUT_2 */

  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, g_mmc5983ma_cfg.mode);
  SPI_SETBITS(spi, g_mmc5983ma_cfg.bits);
  SPI_SETFREQUENCY(spi, g_mmc5983ma_cfg.frequency);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, true);

  SPI_SEND(spi, MMC5983MA_REG_XOUT_0 | 0x80);

  buffer[0] = SPI_SEND(spi, 0xff);
  buffer[1] = SPI_SEND(spi, 0xff);
  buffer[2] = SPI_SEND(spi, 0xff);
  buffer[3] = SPI_SEND(spi, 0xff);
  buffer[4] = SPI_SEND(spi, 0xff);
  buffer[5] = SPI_SEND(spi, 0xff);
  buffer[6] = SPI_SEND(spi, 0xff);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, false);

  SPI_LOCK(spi, false);

  /* 4. Decode 18-bit offset-binary values and center to signed integers */

  uint32_t raw_x = ((uint32_t)buffer[0] << 10) |
                   ((uint32_t)buffer[1] << 2)  |
                   ((buffer[6] >> 6) & 0x03);

  uint32_t raw_y = ((uint32_t)buffer[2] << 10) |
                   ((uint32_t)buffer[3] << 2)  |
                   ((buffer[6] >> 4) & 0x03);

  uint32_t raw_z = ((uint32_t)buffer[4] << 10) |
                   ((uint32_t)buffer[5] << 2)  |
                   ((buffer[6] >> 2) & 0x03);

  /* Subtract 18-bit zero-Gauss offset (131072 = 2^17) */

  data->x = (int32_t)raw_x - 131072;
  data->y = (int32_t)raw_y - 131072;
  data->z = (int32_t)raw_z - 131072;

  return OK;
}