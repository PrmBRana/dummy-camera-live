/****************************************************************************
 * include/nuttx/sensors/iam20380.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * InvenSense IAM-20380 3-Axis Gyroscope Driver Header
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_IAM20380_H
#define __INCLUDE_NUTTX_SENSORS_IAM20380_H

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

/* IAM-20380 Register Map */

#define IAM20380_REG_SMPLRT_DIV       0x19
#define IAM20380_REG_CONFIG           0x1A
#define IAM20380_REG_GYRO_CONFIG      0x1B
#define IAM20380_REG_INT_ENABLE       0x38
#define IAM20380_REG_INT_STATUS       0x3A
#define IAM20380_REG_TEMP_OUT_H       0x41
#define IAM20380_REG_TEMP_OUT_L       0x42
#define IAM20380_REG_GYRO_XOUT_H      0x43
#define IAM20380_REG_GYRO_XOUT_L      0x44
#define IAM20380_REG_GYRO_YOUT_H      0x45
#define IAM20380_REG_GYRO_YOUT_L      0x46
#define IAM20380_REG_GYRO_ZOUT_H      0x47
#define IAM20380_REG_GYRO_ZOUT_L      0x48
#define IAM20380_REG_PWR_MGMT_1       0x6B
#define IAM20380_REG_PWR_MGMT_2       0x6C
#define IAM20380_REG_WHO_AM_I         0x75

/* WHO_AM_I Values:
 * 0xB5: Official InvenSense IAM-20380 Silicon ID
 * 0x68: Compatible MPU-6000 / MPU-6050 Silicon ID
 */

#define IAM20380_WHO_AM_I_VALUE       0xB5
#define IAM20380_WHO_AM_I_ALT_VALUE   0x68

/* Gyroscope Full-Scale Selection */

#define IAM20380_GYRO_FS_250DPS       0x00
#define IAM20380_GYRO_FS_500DPS       0x08
#define IAM20380_GYRO_FS_1000DPS      0x10
#define IAM20380_GYRO_FS_2000DPS      0x18

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct iam20380_config_s
{
  uint32_t frequency;                 /* SPI clock frequency (e.g. 10000000 = 10 MHz) */
  enum spi_mode_e mode;               /* SPI mode (SPIDEV_MODE0 or SPIDEV_MODE3) */
  uint8_t  bits;                      /* SPI bit width (typically 8) */
  uint32_t cs;                        /* Chip select devid (e.g. SPIDEV_USER(0)) */
};

struct iam20380_data_s
{
  int16_t x;                          /* Angular rate X (raw LSBs) */
  int16_t y;                          /* Angular rate Y (raw LSBs) */
  int16_t z;                          /* Angular rate Z (raw LSBs) */
  int16_t temp;                       /* Die temperature (raw LSBs) */
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

/* Initialize IAM-20380 sensor on SPI bus */

int iam20380_initialize(FAR struct spi_dev_s *spi,
                        FAR const struct iam20380_config_s *config);

/* Read raw gyroscope data (X, Y, Z) and temperature */

int iam20380_read(FAR struct spi_dev_s *spi,
                  FAR struct iam20380_data_s *data);

/* Read WHO_AM_I register */

int iam20380_read_whoami(FAR struct spi_dev_s *spi,
                         FAR uint8_t *whoami);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_IAM20380_H */