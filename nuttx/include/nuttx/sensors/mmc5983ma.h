/****************************************************************************
 * include/nuttx/sensors/mmc5983ma.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA 3-axis magnetometer (SPI).
 *
 * UNITS: microtesla (uT).  18-bit mode = 16384 counts/Gauss,
 *        1 Gauss = 100 uT -> 163.84 counts/uT.
 *
 * IMPORTANT: mmc5983ma_counts_to_ut() and mmc5983ma_counts_to_gauss()
 * return float.  If a stale header declares them as int (or has no
 * prototype), the caller reads the float bit pattern as an integer and
 * prints values like 1099546240.00 uT.  Keep this header in sync.
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MMC5983MA_H
#define __INCLUDE_NUTTX_SENSORS_MMC5983MA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define MMC5983MA_REG_XOUT_0          0x00
#define MMC5983MA_REG_STATUS          0x08
#define MMC5983MA_REG_CONTROL_0       0x09
#define MMC5983MA_REG_CONTROL_1       0x0a
#define MMC5983MA_REG_CONTROL_2       0x0b
#define MMC5983MA_REG_CONTROL_3       0x0c
#define MMC5983MA_REG_PRODUCT_ID      0x2f

#define MMC5983MA_PRODUCT_ID_VALUE    0x30

/* Register bit fields */

#define MMC5983MA_STATUS_MEAS_M_DONE  0x01
#define MMC5983MA_CMD_TAKE_MEAS_M     0x01   /* CONTROL_0: TM_M  */
#define MMC5983MA_CMD_SET             0x08   /* CONTROL_0: SET   */
#define MMC5983MA_CMD_RESET           0x10   /* CONTROL_0: RESET */
#define MMC5983MA_CTRL1_SW_RST        0x80

/* 18-bit output is offset binary: zero field = 2^17 */

#define MMC5983MA_COUNTS_MID          131072

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mmc5983ma_config_s
{
  uint32_t        frequency;   /* SPI clock, 10 MHz max          */
  enum spi_mode_e mode;        /* SPIDEV_MODE0                   */
  uint8_t         bits;        /* 8                              */
  uint32_t        cs;          /* SPIDEV_USER(n) chip-select id  */
};

struct mmc5983ma_data_s
{
  int32_t x;   /* signed counts, zero-field centred */
  int32_t y;
  int32_t z;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

int mmc5983ma_initialize(FAR struct spi_dev_s *spi,
                         FAR const struct mmc5983ma_config_s *config);

int mmc5983ma_read_id(FAR struct spi_dev_s *spi, FAR uint8_t *id);

/* Single measurement (includes sensor zero-field offset) */

int mmc5983ma_read_raw(FAR struct spi_dev_s *spi,
                       FAR struct mmc5983ma_data_s *data);

/* SET/RESET offset-cancelled measurement (use this one) */

int mmc5983ma_read(FAR struct spi_dev_s *spi,
                   FAR struct mmc5983ma_data_s *data);

/* Counts -> microtesla (float) */

float mmc5983ma_counts_to_ut(int32_t counts);

/* Counts -> Gauss (float), kept only for legacy callers */

float mmc5983ma_counts_to_gauss(int32_t counts);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_MMC5983MA_H */