/****************************************************************************
 * drivers/sensors/mmc5983ma.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA 3-Axis Magnetometer Driver
 *
 * NOTE ON UNITS (microtesla):
 *   In 18-bit mode the sensitivity is 16384 counts/Gauss (0.0625 mG/LSB).
 *   Since 1 Gauss = 100 microtesla:
 *
 *       uT = counts / 16384 * 100 = counts / 163.84
 *
 *   The SET/RESET pair in mmc5983ma_read() cancels the SENSOR's internal
 *   zero-field offset.  It does NOT remove external hard-iron fields from
 *   the spacecraft itself (battery, magnetorquers, steel fasteners).  If
 *   the measured magnitude is far from the local geomagnetic field
 *   (typically 25-65 uT), run a hard/soft-iron calibration on the fully
 *   assembled satellite and apply the correction above this driver.
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
#include <nuttx/sensors/mmc5983ma.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 18-bit mode: 16384 counts/Gauss.  1 Gauss = 100 uT -> 163.84 counts/uT */

#define MMC5983MA_COUNTS_PER_GAUSS  16384.0f
#define MMC5983MA_COUNTS_PER_UT     163.84f

/* The MMC5983MA tolerates 10 MHz SPI for all transactions */

#define MMC5983MA_SPI_DEFAULT_FREQ  10000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mmc5983ma_config_s g_mmc5983ma_cfg =
{
  .frequency = MMC5983MA_SPI_DEFAULT_FREQ,
  .mode      = SPIDEV_MODE0,
  .bits      = 8,
  .cs        = SPIDEV_USER(0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmc5983ma_configspi
 *
 * Description:
 *   Apply SPI settings. Caller must hold the SPI lock.
 ****************************************************************************/

static void mmc5983ma_configspi(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, g_mmc5983ma_cfg.mode);
  SPI_SETBITS(spi, g_mmc5983ma_cfg.bits);
  SPI_SETFREQUENCY(spi, g_mmc5983ma_cfg.frequency);
}

/****************************************************************************
 * Name: mmc5983ma_write_reg
 ****************************************************************************/

static void mmc5983ma_write_reg(FAR struct spi_dev_s *spi,
                                uint8_t reg,
                                uint8_t value)
{
  SPI_LOCK(spi, true);
  mmc5983ma_configspi(spi);

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
  mmc5983ma_configspi(spi);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, true);

  SPI_SEND(spi, reg | 0x80);
  value = SPI_SEND(spi, 0xff);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, false);

  SPI_LOCK(spi, false);

  return value;
}

/****************************************************************************
 * Name: mmc5983ma_measure
 *
 * Description:
 *   Trigger one magnetic measurement, wait for completion and return the
 *   three axes as signed counts centered on the nominal zero-field value.
 *   No SET/RESET offset cancellation is done here.
 ****************************************************************************/

static int mmc5983ma_measure(FAR struct spi_dev_s *spi,
                             FAR int32_t *out)
{
  uint8_t tx[8];
  uint8_t rx[8];
  uint8_t status = 0;
  uint32_t raw_x;
  uint32_t raw_y;
  uint32_t raw_z;
  bool done = false;
  int timeout;

  /* Clear any stale Meas_M_Done flag (write 1 to clear) */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_STATUS, MMC5983MA_STATUS_MEAS_M_DONE);

  /* Trigger magnetic measurement (TM_M) */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, MMC5983MA_CMD_TAKE_MEAS_M);

  /* Wait for Meas_M_Done (100 Hz bandwidth setting: about 8 ms).
   * Uses a scheduler sleep, not a busy-wait, so the wait time does not
   * depend on the busy-loop delay calibration.
   */

  for (timeout = 100; timeout > 0; timeout--)
    {
      status = mmc5983ma_read_reg(spi, MMC5983MA_REG_STATUS);
      if ((status & MMC5983MA_STATUS_MEAS_M_DONE) != 0)
        {
          done = true;
          break;
        }

      nxsig_usleep(1000);
    }

  if (!done)
    {
      syslog(LOG_ERR, "MMC5983MA: measurement timeout, last STATUS=0x%02x\n",
             status);
      return -ETIMEDOUT;
    }

  /* Burst read XOUT_0 (0x00) .. XYZOUT_2 (0x06): 7 data bytes */

  memset(tx, 0xff, sizeof(tx));
  tx[0] = MMC5983MA_REG_XOUT_0 | 0x80;

  SPI_LOCK(spi, true);
  mmc5983ma_configspi(spi);

  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, true);
  SPI_EXCHANGE(spi, tx, rx, sizeof(tx));
  SPI_SELECT(spi, g_mmc5983ma_cfg.cs, false);

  SPI_LOCK(spi, false);

  /* rx[0] is received while the address is sent; data starts at rx[1].
   * rx[7] is XYZOUT_2: X[1:0] = bits 7:6, Y[1:0] = 5:4, Z[1:0] = 3:2.
   */

  raw_x = ((uint32_t)rx[1] << 10) | ((uint32_t)rx[2] << 2) |
          ((rx[7] >> 6) & 0x03);
  raw_y = ((uint32_t)rx[3] << 10) | ((uint32_t)rx[4] << 2) |
          ((rx[7] >> 4) & 0x03);
  raw_z = ((uint32_t)rx[5] << 10) | ((uint32_t)rx[6] << 2) |
          ((rx[7] >> 2) & 0x03);

  out[0] = (int32_t)raw_x - MMC5983MA_COUNTS_MID;
  out[1] = (int32_t)raw_y - MMC5983MA_COUNTS_MID;
  out[2] = (int32_t)raw_z - MMC5983MA_COUNTS_MID;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmc5983ma_counts_to_ut
 *
 * Description:
 *   Convert a raw axis count (18-bit mode) to microtesla.
 ****************************************************************************/

float mmc5983ma_counts_to_ut(int32_t counts)
{
  return (float)counts / MMC5983MA_COUNTS_PER_UT;
}

/****************************************************************************
 * Name: mmc5983ma_counts_to_gauss
 *
 * Description:
 *   Convert a raw axis count (18-bit mode) to Gauss.  Kept only for legacy
 *   callers; all telemetry uses microtesla.
 ****************************************************************************/

float mmc5983ma_counts_to_gauss(int32_t counts)
{
  return (float)counts / MMC5983MA_COUNTS_PER_GAUSS;
}

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
          g_mmc5983ma_cfg.frequency = MMC5983MA_SPI_DEFAULT_FREQ;
        }

      if (g_mmc5983ma_cfg.bits == 0)
        {
          g_mmc5983ma_cfg.bits = 8;
        }
    }

  /* 0. Read Product ID first (before any register write) and log it */

  id = mmc5983ma_read_reg(spi, MMC5983MA_REG_PRODUCT_ID);
  syslog(LOG_INFO, "MMC5983MA: PRODUCT_ID before reset = 0x%02x (expected 0x%02x)\n",
         id, MMC5983MA_PRODUCT_ID_VALUE);

  /* 1. Software reset, then wait for the device to come back up */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_1, MMC5983MA_CTRL1_SW_RST);
  nxsig_usleep(20000);

  /* 2. Verify Product ID (0x30) */

  id = mmc5983ma_read_reg(spi, MMC5983MA_REG_PRODUCT_ID);
  syslog(LOG_INFO, "MMC5983MA: PRODUCT_ID after reset  = 0x%02x (expected 0x%02x)\n",
         id, MMC5983MA_PRODUCT_ID_VALUE);

  if (id != MMC5983MA_PRODUCT_ID_VALUE)
    {
      syslog(LOG_ERR, "MMC5983MA: PRODUCT_ID mismatch, init aborted\n");
      return -ENODEV;
    }

  /* 3. Execute SET pulse, wait for the set/reset capacitor to recharge */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, MMC5983MA_CMD_SET);
  nxsig_usleep(2000);

  /* 4. Control registers to default state (BW = 100 Hz, all axes on) */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, 0x00);
  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_1, 0x00);
  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_2, 0x00);
  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_3, 0x00);

  syslog(LOG_INFO, "MMC5983MA: initialized OK (18-bit, BW 100 Hz, "
         "%.2f counts/uT)\n", (double)MMC5983MA_COUNTS_PER_UT);
  return OK;
}

/****************************************************************************
 * Name: mmc5983ma_read_raw
 *
 * Description:
 *   Single measurement. Fast, but includes the sensor's zero-field offset.
 ****************************************************************************/

int mmc5983ma_read_raw(FAR struct spi_dev_s *spi,
                       FAR struct mmc5983ma_data_s *data)
{
  int32_t m[3];
  int ret;

  if (spi == NULL || data == NULL)
    {
      return -EINVAL;
    }

  ret = mmc5983ma_measure(spi, m);
  if (ret < 0)
    {
      return ret;
    }

  data->x = m[0];
  data->y = m[1];
  data->z = m[2];

  syslog(LOG_INFO,
         "MMC5983MA: raw (single) x=%ld y=%ld z=%ld counts "
         "-> %.2f %.2f %.2f uT\n",
         (long)data->x, (long)data->y, (long)data->z,
         (double)mmc5983ma_counts_to_ut(data->x),
         (double)mmc5983ma_counts_to_ut(data->y),
         (double)mmc5983ma_counts_to_ut(data->z));

  return OK;
}

/****************************************************************************
 * Name: mmc5983ma_read
 *
 * Description:
 *   Offset-cancelled measurement:
 *     Out_set   =  Field + Offset
 *     Out_reset = -Field + Offset
 *     Field     = (Out_set - Out_reset) / 2
 ****************************************************************************/

int mmc5983ma_read(FAR struct spi_dev_s *spi,
                   FAR struct mmc5983ma_data_s *data)
{
  int32_t set_out[3];
  int32_t rst_out[3];
  int ret;

  if (spi == NULL || data == NULL)
    {
      return -EINVAL;
    }

  /* SET, then measure */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, MMC5983MA_CMD_SET);
  nxsig_usleep(2000);

  ret = mmc5983ma_measure(spi, set_out);
  if (ret < 0)
    {
      return ret;
    }

  /* RESET, then measure */

  mmc5983ma_write_reg(spi, MMC5983MA_REG_CONTROL_0, MMC5983MA_CMD_RESET);
  nxsig_usleep(2000);

  ret = mmc5983ma_measure(spi, rst_out);
  if (ret < 0)
    {
      return ret;
    }

  data->x = (set_out[0] - rst_out[0]) / 2;
  data->y = (set_out[1] - rst_out[1]) / 2;
  data->z = (set_out[2] - rst_out[2]) / 2;

  syslog(LOG_INFO,
         "MMC5983MA: raw (SET/RESET) x=%ld y=%ld z=%ld counts "
         "-> %.2f %.2f %.2f uT\n",
         (long)data->x, (long)data->y, (long)data->z,
         (double)mmc5983ma_counts_to_ut(data->x),
         (double)mmc5983ma_counts_to_ut(data->y),
         (double)mmc5983ma_counts_to_ut(data->z));

  return OK;
}