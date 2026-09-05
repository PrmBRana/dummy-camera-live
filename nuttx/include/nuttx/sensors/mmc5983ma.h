/****************************************************************************
 * include/nuttx/sensors/mmc5983ma.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA 3-Axis Magnetometer Driver Header
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

/* Register Map */

#define MMC5983MA_REG_XOUT_0         0x00
#define MMC5983MA_REG_XOUT_1         0x01
#define MMC5983MA_REG_YOUT_0         0x02
#define MMC5983MA_REG_YOUT_1         0x03
#define MMC5983MA_REG_ZOUT_0         0x04
#define MMC5983MA_REG_ZOUT_1         0x05
#define MMC5983MA_REG_XYZOUT_2       0x06
#define MMC5983MA_REG_TOUT           0x07
#define MMC5983MA_REG_STATUS         0x08
#define MMC5983MA_REG_CONTROL_0      0x09
#define MMC5983MA_REG_CONTROL_1      0x0A
#define MMC5983MA_REG_CONTROL_2      0x0B
#define MMC5983MA_REG_CONTROL_3      0x0C
#define MMC5983MA_REG_PRODUCT_ID     0x2F

#define MMC5983MA_PRODUCT_ID_VALUE   0x30

/* STATUS Register Bits */

#define MMC5983MA_STATUS_MEAS_M_DONE 0x01
#define MMC5983MA_STATUS_MEAS_T_DONE 0x02

/* CONTROL 0 Register Bits */

#define MMC5983MA_CMD_TAKE_MEAS_M    0x01  /* Trigger magnetic measurement */
#define MMC5983MA_CMD_TAKE_MEAS_T    0x02  /* Trigger temperature measurement */
#define MMC5983MA_CMD_SET            0x08  /* Apply SET pulse */
#define MMC5983MA_CMD_RESET          0x10  /* Apply RESET pulse */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mmc5983ma_config_s
{
  uint32_t frequency;                 /* SPI clock frequency (e.g. 10000000 = 10 MHz) */
  enum spi_mode_e mode;               /* SPI mode (SPIDEV_MODE0 or SPIDEV_MODE3) */
  uint8_t  bits;                      /* SPI bit width (typically 8) */
  uint32_t cs;                        /* Chip select devid (e.g. SPIDEV_USER(0)) */
};

struct mmc5983ma_data_s
{
  int32_t x;                          /* Magnetic field X (18-bit signed LSB, 16384 LSB/G) */
  int32_t y;                          /* Magnetic field Y (18-bit signed LSB, 16384 LSB/G) */
  int32_t z;                          /* Magnetic field Z (18-bit signed LSB, 16384 LSB/G) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Initialize MMC5983MA sensor on SPI bus */

int mmc5983ma_initialize(FAR struct spi_dev_s *spi,
                         FAR const struct mmc5983ma_config_s *config);

/* Trigger and read 18-bit magnetic measurement (X, Y, Z) */

int mmc5983ma_read(FAR struct spi_dev_s *spi,
                   FAR struct mmc5983ma_data_s *data);

/* Read product identifier register */

int mmc5983ma_read_id(FAR struct spi_dev_s *spi,
                      FAR uint8_t *id);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_MMC5983MA_H */