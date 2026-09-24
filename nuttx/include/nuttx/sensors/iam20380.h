/****************************************************************************
 * include/nuttx/sensors/iam20380.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TDK InvenSense IAM-20380 and IAM-20380HT 3-axis gyroscope (SPI).
 *
 * Both variants share the same register map.  Only WHO_AM_I differs:
 *
 *   IAM-20380    -> 0xB5
 *   IAM-20380HT  -> 0xFA   (confirm against your HT datasheet revision)
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

/* WHO_AM_I values */

#define IAM20380_WHO_AM_I_VALUE     0xb5   /* IAM-20380 (Section 9.26) */
#define IAM20380HT_WHO_AM_I_VALUE   0xfd   /* IAM-20380HT (Section 9.26 of Datasheet DS-000521) */
#define IAM20380HT_WHO_AM_I_ALT     0xfa   /* IAM-20380HT alternate revision */
#define MPU6000_WHO_AM_I_VALUE      0x68   /* MPU-6000 / MPU-6050 */
#define MPU6500_WHO_AM_I_VALUE      0x70   /* MPU-6500    */
#define MPU9250_WHO_AM_I_VALUE      0x71   /* MPU-9250    */
#define MPU6515_WHO_AM_I_VALUE      0x73   /* MPU-6515    */
#define ICM20600_WHO_AM_I_VALUE     0x11   /* ICM-20600   */
#define ICM20602_WHO_AM_I_VALUE     0x12   /* ICM-20602   */
#define ICM20689_WHO_AM_I_VALUE     0x98   /* ICM-20689   */

/* Registers */

#define IAM20380_REG_SMPLRT_DIV     0x19
#define IAM20380_REG_CONFIG         0x1a
#define IAM20380_REG_GYRO_CONFIG    0x1b
#define IAM20380_REG_TEMP_OUT_H     0x41
#define IAM20380_REG_TEMP_OUT_L     0x42
#define IAM20380_REG_GYRO_XOUT_H    0x43
#define IAM20380_REG_USER_CTRL      0x6a
#define IAM20380_REG_PWR_MGMT_1     0x6b
#define IAM20380_REG_PWR_MGMT_2     0x6c
#define IAM20380_REG_WHO_AM_I       0x75

/* Register bit fields */

#define IAM20380_PWR1_DEVICE_RESET  0x80
#define IAM20380_PWR1_CLKSEL_AUTO   0x01   /* Auto-select PLL */
#define IAM20380_USERCTRL_I2C_IF_DIS 0x10
#define IAM20380_DLPF_176HZ         0x01   /* DLPF_CFG = 1 */
#define IAM20380_GYRO_FS_250DPS     0x00   /* FS_SEL = 0   */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct iam20380_config_s
{
  uint32_t        frequency;      /* Sensor-data burst clock, 8 MHz max     */
  enum spi_mode_e mode;           /* SPIDEV_MODE0 or SPIDEV_MODE3           */
  uint8_t         bits;           /* 8                                      */
  uint32_t        cs;             /* SPIDEV_USER(n) chip-select id          */
  uint32_t        reg_frequency;  /* Register-access clock, 0 = 1 MHz (max) */
};

struct iam20380_data_s
{
  int16_t temp;
  int16_t x;
  int16_t y;
  int16_t z;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* true for 0xB5 (IAM-20380) or 0xFA (IAM-20380HT) */

bool iam20380_id_valid(uint8_t id);

/* "IAM-20380", "IAM-20380HT" or "unknown" */

const char *iam20380_id_name(uint8_t id);

int iam20380_read_whoami(FAR struct spi_dev_s *spi, FAR uint8_t *whoami);

int iam20380_probe(FAR struct spi_dev_s *spi,
                   FAR const struct iam20380_config_s *config,
                   FAR uint8_t *whoami);

int iam20380_initialize(FAR struct spi_dev_s *spi,
                        FAR const struct iam20380_config_s *config);

int iam20380_read(FAR struct spi_dev_s *spi,
                  FAR struct iam20380_data_s *data);

float iam20380_counts_to_dps(int16_t counts);
float iam20380_counts_to_degc(int16_t counts);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_IAM20380_H */