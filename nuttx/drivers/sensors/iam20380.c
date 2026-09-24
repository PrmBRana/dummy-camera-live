/****************************************************************************
 * drivers/sensors/iam20380.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TDK InvenSense IAM-20380 / IAM-20380HT 3-Axis Gyroscope Driver
 *
 * VARIANTS:
 *   IAM-20380    WHO_AM_I = 0xB5
 *   IAM-20380HT  WHO_AM_I = 0xFA
 *   Same register map; both are accepted.
 *
 * NOTE ON SPI CLOCK:
 *   Up to 8 MHz is allowed ONLY for burst reads of the sensor data
 *   registers.  ALL other register reads and writes (WHO_AM_I, PWR_MGMT_1,
 *   USER_CTRL, configuration registers) are limited to 1 MHz.  This driver
 *   switches the SPI clock per transaction type.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/iam20380.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register access: 1 MHz maximum (datasheet limit) */

#define IAM20380_SPI_REG_FREQ     1000000

/* Sensor data burst read: 8 MHz maximum (datasheet limit) */

#define IAM20380_SPI_DATA_FREQ    8000000

/* Gyroscope sensitivity for the +/-250 dps full-scale range */

#define IAM20380_LSB_PER_DPS      131.0f

/* On-chip temperature sensor scaling */

#define IAM20380_TEMP_SENSITIVITY 326.8f
#define IAM20380_TEMP_OFFSET_C    25.0f

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct iam20380_config_s g_iam20380_cfg =
{
  .frequency     = IAM20380_SPI_DATA_FREQ,
  .mode          = SPIDEV_MODE0,
  .bits          = 8,
  .cs            = SPIDEV_USER(0),
  .reg_frequency = IAM20380_SPI_REG_FREQ,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iam20380_reg_freq
 *
 * Description:
 *   Register-access clock, clamped to the 1 MHz datasheet limit.
 ****************************************************************************/

static uint32_t iam20380_reg_freq(void)
{
  uint32_t freq = g_iam20380_cfg.reg_frequency;

  if (freq == 0 || freq > IAM20380_SPI_REG_FREQ)
    {
      freq = IAM20380_SPI_REG_FREQ;
    }

  return freq;
}

/****************************************************************************
 * Name: iam20380_data_freq
 *
 * Description:
 *   Clamp the configured clock to the 8 MHz burst-read limit.
 ****************************************************************************/

static uint32_t iam20380_data_freq(void)
{
  if (g_iam20380_cfg.frequency == 0 ||
      g_iam20380_cfg.frequency > IAM20380_SPI_DATA_FREQ)
    {
      return IAM20380_SPI_DATA_FREQ;
    }

  return g_iam20380_cfg.frequency;
}

/****************************************************************************
 * Name: iam20380_configspi
 *
 * Description:
 *   Apply SPI settings at the requested clock.  Caller must hold the lock.
 ****************************************************************************/

static void iam20380_configspi(FAR struct spi_dev_s *spi, uint32_t frequency)
{
  SPI_SETMODE(spi, g_iam20380_cfg.mode);
  SPI_SETBITS(spi, g_iam20380_cfg.bits);
  SPI_SETFREQUENCY(spi, frequency);
}

/****************************************************************************
 * Name: iam20380_apply_config
 ****************************************************************************/

static void iam20380_apply_config(FAR const struct iam20380_config_s *config)
{
  g_iam20380_cfg = *config;

  if (g_iam20380_cfg.frequency == 0)
    {
      g_iam20380_cfg.frequency = IAM20380_SPI_DATA_FREQ;
    }

  if (g_iam20380_cfg.bits == 0)
    {
      g_iam20380_cfg.bits = 8;
    }

  if (g_iam20380_cfg.reg_frequency == 0)
    {
      g_iam20380_cfg.reg_frequency = IAM20380_SPI_REG_FREQ;
    }
}

/****************************************************************************
 * Name: iam20380_write_reg
 *
 * Description:
 *   Single register write.  Clock limited to 1 MHz.
 ****************************************************************************/

static void iam20380_write_reg(FAR struct spi_dev_s *spi,
                               uint8_t reg,
                               uint8_t value)
{
  uint8_t tx[2];

  tx[0] = reg & 0x7f;
  tx[1] = value;

  SPI_LOCK(spi, true);
  iam20380_configspi(spi, iam20380_reg_freq());

  SPI_SELECT(spi, g_iam20380_cfg.cs, true);
  up_udelay(1);

  SPI_EXCHANGE(spi, tx, NULL, 2);

  up_udelay(1);
  SPI_SELECT(spi, g_iam20380_cfg.cs, false);
  up_udelay(2);

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: iam20380_read_reg
 *
 * Description:
 *   Single register read.  Clock limited to 1 MHz.
 ****************************************************************************/

static uint8_t iam20380_read_reg(FAR struct spi_dev_s *spi,
                                 uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80;
  tx[1] = 0xff;
  rx[0] = 0;
  rx[1] = 0;

  SPI_LOCK(spi, true);
  iam20380_configspi(spi, iam20380_reg_freq());

  SPI_SELECT(spi, g_iam20380_cfg.cs, true);
  up_udelay(1);

  SPI_EXCHANGE(spi, tx, rx, 2);

  up_udelay(1);
  SPI_SELECT(spi, g_iam20380_cfg.cs, false);
  up_udelay(2);

  SPI_LOCK(spi, false);

  return rx[1];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iam20380_id_valid
 ****************************************************************************/

bool iam20380_id_valid(uint8_t id)
{
  return (id == IAM20380_WHO_AM_I_VALUE) ||
         (id == IAM20380HT_WHO_AM_I_VALUE) ||
         (id == IAM20380HT_WHO_AM_I_ALT) ||
         (id == MPU6000_WHO_AM_I_VALUE) ||
         (id == MPU6500_WHO_AM_I_VALUE) ||
         (id == MPU9250_WHO_AM_I_VALUE) ||
         (id == MPU6515_WHO_AM_I_VALUE) ||
         (id == ICM20600_WHO_AM_I_VALUE) ||
         (id == ICM20602_WHO_AM_I_VALUE) ||
         (id == ICM20689_WHO_AM_I_VALUE);
}

/****************************************************************************
 * Name: iam20380_id_name
 ****************************************************************************/

const char *iam20380_id_name(uint8_t id)
{
  switch (id)
    {
      case IAM20380_WHO_AM_I_VALUE:
        return "IAM-20380";

      case IAM20380HT_WHO_AM_I_VALUE:
      case IAM20380HT_WHO_AM_I_ALT:
        return "IAM-20380HT";

      case MPU6000_WHO_AM_I_VALUE:
        return "MPU-6000/6050";

      case MPU6500_WHO_AM_I_VALUE:
        return "MPU-6500";

      case MPU9250_WHO_AM_I_VALUE:
        return "MPU-9250";

      case MPU6515_WHO_AM_I_VALUE:
        return "MPU-6515";

      case ICM20600_WHO_AM_I_VALUE:
        return "ICM-20600";

      case ICM20602_WHO_AM_I_VALUE:
        return "ICM-20602";

      case ICM20689_WHO_AM_I_VALUE:
        return "ICM-20689";

      default:
        return "unknown";
    }
}

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
 * Name: iam20380_probe
 *
 * Description:
 *   Read WHO_AM_I with the given SPI config.  Read-only: no sensor register
 *   is written, so this is safe to call repeatedly while scanning candidate
 *   chip-select IDs, SPI modes and clocks.  Replaces the stored config, so
 *   call iam20380_initialize() afterwards with the final config.
 ****************************************************************************/

int iam20380_probe(FAR struct spi_dev_s *spi,
                   FAR const struct iam20380_config_s *config,
                   FAR uint8_t *whoami)
{
  if (spi == NULL || config == NULL || whoami == NULL)
    {
      return -EINVAL;
    }

  iam20380_apply_config(config);

  /* Pulse CS to switch device from default I2C mode into SPI mode */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, g_iam20380_cfg.cs, true);
  up_udelay(2);
  SPI_SELECT(spi, g_iam20380_cfg.cs, false);
  up_udelay(5);
  SPI_LOCK(spi, false);

  *whoami = iam20380_read_reg(spi, IAM20380_REG_WHO_AM_I);
  return OK;
}

/****************************************************************************
 * Name: iam20380_initialize
 ****************************************************************************/

int iam20380_initialize(FAR struct spi_dev_s *spi,
                        FAR const struct iam20380_config_s *config)
{
  uint8_t whoami = 0;
  int retry;

  if (spi == NULL)
    {
      return -EINVAL;
    }

  /* Save device configuration if provided */

  if (config != NULL)
    {
      iam20380_apply_config(config);
    }

  /* 1. Pulse CS to ensure sensor selects SPI interface mode */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, g_iam20380_cfg.cs, false);
  up_udelay(5);
  SPI_SELECT(spi, g_iam20380_cfg.cs, true);
  up_udelay(5);
  SPI_SELECT(spi, g_iam20380_cfg.cs, false);
  up_udelay(10);
  SPI_LOCK(spi, false);

  /* 2. Device reset, then wait for it to complete */

  iam20380_write_reg(spi, IAM20380_REG_PWR_MGMT_1, IAM20380_PWR1_DEVICE_RESET);
  nxsig_usleep(50000);

  /* 3. Disable the I2C interface (required when using SPI) */

  iam20380_write_reg(spi, IAM20380_REG_USER_CTRL, IAM20380_USERCTRL_I2C_IF_DIS);
  up_udelay(100);

  /* 4. Wake from sleep and select PLL as clock source (CLKSEL = 1) */

  iam20380_write_reg(spi, IAM20380_REG_PWR_MGMT_1, IAM20380_PWR1_CLKSEL_AUTO);
  nxsig_usleep(20000);

  /* 5. Enable all gyro axes */

  iam20380_write_reg(spi, IAM20380_REG_PWR_MGMT_2, 0x00);
  up_udelay(100);

  /* 6. Verify WHO_AM_I: accept IAM-20380, IAM-20380HT, and MPU/ICM variants */

  for (retry = 0; retry < 5; retry++)
    {
      whoami = iam20380_read_reg(spi, IAM20380_REG_WHO_AM_I);
      if (iam20380_id_valid(whoami))
        {
          break;
        }
      nxsig_usleep(10000);
    }

  syslog(LOG_INFO, "IAM20380: WHO_AM_I = 0x%02x (%s)\n",
         whoami, iam20380_id_name(whoami));

  if (!iam20380_id_valid(whoami))
    {
      syslog(LOG_ERR, "IAM20380: WHO_AM_I mismatch (0x%02x), init aborted\n", whoami);
      return -ENODEV;
    }

  /* 7. Digital low pass filter: DLPF_CFG = 1 (176 Hz, 1 kHz internal rate) */

  iam20380_write_reg(spi, IAM20380_REG_CONFIG, IAM20380_DLPF_176HZ);

  /* 8. Sample rate divider: 1 kHz / (1 + 7) = 125 Hz
   * (only effective because DLPF_CFG is 1..6 above)
   */

  iam20380_write_reg(spi, IAM20380_REG_SMPLRT_DIV, 0x07);

  /* 9. Gyroscope full-scale range: +/-250 dps */

  iam20380_write_reg(spi, IAM20380_REG_GYRO_CONFIG, IAM20380_GYRO_FS_250DPS);

  /* 10. Give the PLL time to settle before the first sample */

  nxsig_usleep(20000);

  syslog(LOG_INFO,
         "IAM20380: %s initialized OK (125 Hz, +/-250 dps, burst clk=%u Hz)\n",
         iam20380_id_name(whoami), (unsigned int)iam20380_data_freq());
  return OK;
}

/****************************************************************************
 * Name: iam20380_read
 *
 * Description:
 *   Burst read of TEMP_OUT and the three gyro axes.  Runs at the clamped
 *   burst clock (<= 8 MHz).  Returns -EIO if the whole frame reads back as
 *   all 0xFF (dead bus / CS not asserted / floating MISO).
 ****************************************************************************/

int iam20380_read(FAR struct spi_dev_s *spi,
                  FAR struct iam20380_data_s *data)
{
  uint8_t tx[9];
  uint8_t rx[9];
  bool all_ones = true;
  int i;

  if (spi == NULL || data == NULL)
    {
      return -EINVAL;
    }

  /* Burst read TEMP_OUT_H (0x41) .. GYRO_ZOUT_L (0x48): 8 data bytes */

  memset(tx, 0xff, sizeof(tx));
  tx[0] = IAM20380_REG_TEMP_OUT_H | 0x80;

  SPI_LOCK(spi, true);
  iam20380_configspi(spi, iam20380_data_freq());

  SPI_SELECT(spi, g_iam20380_cfg.cs, true);
  up_udelay(1);
  SPI_EXCHANGE(spi, tx, rx, sizeof(tx));
  up_udelay(1);
  SPI_SELECT(spi, g_iam20380_cfg.cs, false);
  up_udelay(2);

  SPI_LOCK(spi, false);

  for (i = 1; i < (int)sizeof(rx); i++)
    {
      if (rx[i] != 0xff)
        {
          all_ones = false;
          break;
        }
    }

  if (all_ones)
    {
      syslog(LOG_ERR, "IAM20380: burst read returned a dead frame (all 0xFF)\n");
      return -EIO;
    }

  /* rx[0] is received while the address is sent; data starts at rx[1] */

  data->temp = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);
  data->x    = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
  data->y    = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  data->z    = (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);

  syslog(LOG_INFO, "IAM20380: raw temp=%d x=%d y=%d z=%d\n",
         data->temp, data->x, data->y, data->z);

  return OK;
}

/****************************************************************************
 * Name: iam20380_counts_to_dps
 *
 * Description:
 *   Convert a raw gyro axis count to degrees per second for the +/-250 dps
 *   full-scale range configured by iam20380_initialize().
 ****************************************************************************/

float iam20380_counts_to_dps(int16_t counts)
{
  return (float)counts / IAM20380_LSB_PER_DPS;
}

/****************************************************************************
 * Name: iam20380_counts_to_degc
 *
 * Description:
 *   Convert the raw on-chip temperature count to degrees Celsius.
 ****************************************************************************/

float iam20380_counts_to_degc(int16_t counts)
{
  return ((float)counts / IAM20380_TEMP_SENSITIVITY) + IAM20380_TEMP_OFFSET_C;
}