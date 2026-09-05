/****************************************************************************
 * apps/include/telemetry_rb.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compact Scaled (x100) Telemetry Packet & Ring Buffer Definition.
 * Shared between OBC_main (Producer) and littlefs (Consumer).
 ****************************************************************************/

#ifndef __APPS_INCLUDE_TELEMETRY_RB_H
#define __APPS_INCLUDE_TELEMETRY_RB_H

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#define ADC1_CHANNELS_COUNT     16  /* 16 channels x 2B = 32 Bytes */
#define ADC2_ACTIVE_CHANNELS    12  /* 12 active channels (excluding NC: 0,4,7,12) x 2B = 24 Bytes */

#define TELEM_FOOTER_ADC1       0xAA55  /* Footer 1 for ADC 1 packet */
#define TELEM_FOOTER_ADC2       0xBB66  /* Footer 2 for ADC 2 + IMU packet */

#define TELEM_RB_DEPTH          16

/* ADC 1 Packet: 16 channels x 2B (scaled x100) + 2B Footer = 34 Bytes */
struct adc1_packet_s
{
  int16_t  data[ADC1_CHANNELS_COUNT]; /* 16 channels x 2B = 32 Bytes (scaled x100) */
  uint16_t footer;                    /* 0xAA55 = 2 Bytes */
};

/* ADC 2 + IMU Packet: 12 Currents + 3-Axis Gyro + 3-Axis Mag (scaled x100) + 2B Footer = 38 Bytes */
struct adc2_packet_s
{
  int16_t  data[ADC2_ACTIVE_CHANNELS]; /* 12 channels x 2B = 24 Bytes (scaled x100) */
  int16_t  gyro_x;                     /* Gyro X deg/s (scaled x100) = 2 Bytes */
  int16_t  gyro_y;                     /* Gyro Y deg/s (scaled x100) = 2 Bytes */
  int16_t  gyro_z;                     /* Gyro Z deg/s (scaled x100) = 2 Bytes */
  int16_t  mag_x;                      /* Mag X Gauss  (scaled x100) = 2 Bytes */
  int16_t  mag_y;                      /* Mag Y Gauss  (scaled x100) = 2 Bytes */
  int16_t  mag_z;                      /* Mag Z Gauss  (scaled x100) = 2 Bytes */
  uint16_t footer;                     /* 0xBB66 = 2 Bytes */
};

/* Unified Telemetry Envelope Packet for Ring Buffer Transfer */
struct telemetry_envelope_s
{
  uint8_t  len;                        /* Payload length (34 for ADC1, 38 for ADC2+IMU) */
  uint16_t footer;                     /* 0xAA55 (HK1) or 0xBB66 (HK2) */
  union
  {
    struct adc1_packet_s adc1;
    struct adc2_packet_s adc2;
    uint8_t              raw[48];
  } pkt;
};

/* Telemetry Ring Buffer */
struct telemetry_rb_s
{
  struct telemetry_envelope_s slots[TELEM_RB_DEPTH];
  volatile uint32_t           wr_ptr;
  volatile uint32_t           rd_ptr;
  volatile uint32_t           count;
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Exported Global Telemetry Ring Buffer & Notification Semaphore */
EXTERN struct telemetry_rb_s g_telemetry_rb;
EXTERN sem_t                 g_telemetry_sem;

/* Ring Buffer Function Prototypes */
void telemetry_rb_init(void);
int  telemetry_rb_write(const struct telemetry_envelope_s *env);
int  telemetry_rb_read(struct telemetry_envelope_s *env);
int  telemetry_rb_count(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_TELEMETRY_RB_H */
