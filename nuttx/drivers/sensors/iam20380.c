/****************************************************************************
 * drivers/sensors/iam20380.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * InvenSense IAM-20380 3-Axis Gyroscope Driver
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
#include <nuttx/sensors/iam20380.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct iam20380_config_s g_iam20380_cfg =
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
 * Name: iam20380_write_reg
 ****************************************************************************/

static void iam20380_write_reg(FAR struct spi_dev_s *spi,
                               uint8_t reg,
                               uint8_t value)
{
  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, g_iam20380_cfg.mode);
  SPI_SETBITS(spi, g_iam20380_cfg.bits);
  SPI_SETFREQUENCY(spi, g_iam20380_cfg.frequency);

  SPI_SELECT(spi, g_iam20380_cfg.cs, true);

  SPI_SEND(spi, reg & 0x7f);
  SPI_SEND(spi, value);

  SPI_SELECT(spi, g_iam20380_cfg.cs, false);

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: iam20380_read_reg
 ****************************************************************************/

static uint8_t iam20380_read_reg(FAR struct spi_dev_s *spi,
                                 uint8_t reg)
{
  uint8_t value;

  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, g_iam20380_cfg.mode);
  SPI_SETBITS(spi, g_iam20380_cfg.bits);
  SPI_SETFREQUENCY(spi, g_iam20380_cfg.frequency);

  SPI_SELECT(spi, g_iam20380_cfg.cs, true);

  SPI_SEND(spi, reg | 0x80);
  value = SPI_SEND(spi, 0xff);

  SPI_SELECT(spi, g_iam20380_cfg.cs, false);

  SPI_LOCK(spi, false);

  return value;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iam20380_read_whoami
 ****************************************************************************/

int iam20380_read_whoami(FAR struct spi_dev_s *spi,
                         FAR uint8_t *whoami)
{
  if (spi == NULL || whoami == NULL)
    {
      return -EINVAL;
    }

  *whoami = iam20380_read_reg(spi, IAM20380_REG_WHO_AM_I);
  return OK;
}

/****************************************************************************
 * Name: iam20380_initialize
 ****************************************************************************/

int iam20380_initialize(FAR struct spi_dev_s *spi,
                        FAR const struct iam20380_config_s *config)
{
  uint8_t whoami;

  if (spi == NULL)
    {
      return -EINVAL;
    }

  /* Save device configuration if provided */

  if (config != NULL)
    {
      g_iam20380_cfg = *config;
      if (g_iam20380_cfg.frequency == 0)
        {
          g_iam20380_cfg.frequency = 10000000;
        }

      if (g_iam20380_cfg.bits == 0)
        {
          g_iam20380_cfg.bits = 8;
        }
    }

  /* 1. Wake device from sleep mode (PWR_MGMT_1 = 0x00) */

  iam20380_write_reg(spi, IAM20380_REG_PWR_MGMT_1, 0x00);

  /* Allow 15 ms for internal gyro PLL clock to stabilize */

  up_mdelay(15);

  /* 2. Verify WHO_AM_I identifier */

  whoami = iam20380_read_reg(spi, IAM20380_REG_WHO_AM_I);
  if (whoami != IAM20380_WHO_AM_I_VALUE && whoami != IAM20380_WHO_AM_I_ALT_VALUE)
    {
      return -ENODEV;
    }

  /* 3. Configure Sample Rate Divider (SMPLRT_DIV = 7 -> 1 kHz / (1 + 7) = 125 Hz) */

  iam20380_write_reg(spi, IAM20380_REG_SMPLRT_DIV, 0x07);

  /* 4. Configure Digital Low Pass Filter (CONFIG = 0x00) */

  iam20380_write_reg(spi, IAM20380_REG_CONFIG, 0x00);

  /* 5. Set Gyroscope Full-Scale Range (±250 dps) */

  iam20380_write_reg(spi, IAM20380_REG_GYRO_CONFIG, IAM20380_GYRO_FS_250DPS);

  return OK;
}

/****************************************************************************
 * Name: iam20380_read
 ****************************************************************************/

int iam20380_read(FAR struct spi_dev_s *spi,
                  FAR struct iam20380_data_s *data)
{
  uint8_t buffer[8];

  if (spi == NULL || data == NULL)
    {
      return -EINVAL;
    }

  /* Burst read starting from TEMP_OUT_H (0x41) through GYRO_ZOUT_L (0x48) */

  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, g_iam20380_cfg.mode);
  SPI_SETBITS(spi, g_iam20380_cfg.bits);
  SPI_SETFREQUENCY(spi, g_iam20380_cfg.frequency);

  SPI_SELECT(spi, g_iam20380_cfg.cs, true);

  SPI_SEND(spi, IAM20380_REG_TEMP_OUT_H | 0x80);

  /* Read 8 bytes: Temp(2B), Gyro_X(2B), Gyro_Y(2B), Gyro_Z(2B) */

  buffer[0] = SPI_SEND(spi, 0xff);
  buffer[1] = SPI_SEND(spi, 0xff);
  buffer[2] = SPI_SEND(spi, 0xff);
  buffer[3] = SPI_SEND(spi, 0xff);
  buffer[4] = SPI_SEND(spi, 0xff);
  buffer[5] = SPI_SEND(spi, 0xff);
  buffer[6] = SPI_SEND(spi, 0xff);
  buffer[7] = SPI_SEND(spi, 0xff);

  SPI_SELECT(spi, g_iam20380_cfg.cs, false);

  SPI_LOCK(spi, false);

  /* Decode 16-bit signed two's complement data */

  data->temp = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
  data->x    = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
  data->y    = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
  data->z    = (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);

  return OK;
}