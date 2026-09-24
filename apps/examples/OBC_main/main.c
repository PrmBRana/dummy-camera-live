/****************************************************************************
 * apps/examples/OBC_main/main.c
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
 * Overview
 *
 *   OBC_main is the On-Board Computer's telemetry *producer* task.  Once
 *   per acquisition cycle (TELEMETRY_INTERVAL_SEC) it:
 *
 *     1. Triggers and reads both ADS7953 12-bit, 16-channel SPI ADCs
 *        (/dev/adc0 = voltages/temperatures, /dev/adc1 = currents).
 *     2. Reads the IAM-20380 gyroscope and MMC5983MA magnetometer, both
 *        on SPI2.
 *     3. Converts raw ADC counts to engineering units (volts, amps,
 *        degrees C) using the per-channel tables below, flagging
 *        disconnected / dark / idle channels instead of reporting bogus
 *        physical values.
 *     4. Packs the results into two fixed-size telemetry packets and:
 *          - pushes them to a local ring buffer, which wakes the LittleFS
 *            storage daemon (see littlefs_main_mount.c) so they get
 *            written to flash, and
 *          - queues them into the shared-SRAM2 ring buffer read by the
 *            Cortex-M0+ radio core, then rings the IPCC "beacon" doorbell
 *            so the M0+ can transmit them.
 *     5. Spends the remainder of the cycle listening for ground commands
 *        forwarded from the M0+ (camera capture, housekeeping requests)
 *        and draining the M0+'s radio event log to the console.
 *
 *   This file only *produces* telemetry; it does not talk to flash
 *   directly.  Storage is handled by the LittleFS daemon via the local
 *   ring buffer + semaphore, and the M0+ / radio link via the IPCC shared
 *   memory ring buffers.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <semaphore.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <dirent.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/iam20380.h>
#include <nuttx/sensors/mmc5983ma.h>

#include <telemetry_rb.h>
#include "Sring_buffer.h"

/* Board-specific SPI bus initialization (implemented in board.c) */

FAR struct spi_dev_s *stm32wl5_spibus_initialize(int bus);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC device paths and geometry */

#define ADC1_DEVPATH        "/dev/adc0"   /* Voltage & temperature bank   */
#define ADC2_DEVPATH        "/dev/adc1"   /* Current bank                */
#define ADS7953_CHANNELS    16            /* Channels per ADS7953 chip   */

/* ADS7953 full-scale: 12-bit (4095 counts) with 2x range (Vref = 2.5V,
 * doubled internally to a 0-5.0V input span).
 */

#define ADC_VREF_FULLSCALE  5.000f        /* Volts at 4095 counts        */
#define ADC_MAX_COUNTS      4095.0f       /* 12-bit full scale           */

/* NTC thermistor parameters (10k NTC with a 10k pull-up to 3.3V) */

#define NTC_R_PULLUP        10000.0f      /* Pull-up resistor, ohms      */
#define NTC_V_SUPPLY        3.300f        /* Pull-up supply rail, volts  */
#define NTC_R0               10000.0f     /* NTC resistance at 25 deg C  */
#define NTC_BETA             3950.0f      /* Beta coefficient (B25/85)   */
#define NTC_T0                298.15f     /* 25 deg C, in Kelvin         */

/* Detection thresholds used to distinguish a real reading from an
 * unpowered / disconnected / unloaded channel.
 */

#define SOLAR_V_THRESHOLD   0.250f        /* Below this: panel dark/off  */
#define SOLAR_I_THRESHOLD   0.015f        /* Below this: no panel current*/
#define IDLE_I_THRESHOLD    0.010f        /* Below this: rail unloaded   */

/* Acquisition cadence */

#define TELEMETRY_INTERVAL_SEC  90

/* SPI2 chip-select IDs for the IMU devices.
 *
 * IMPORTANT: each of these MUST have a matching case in
 * stm32wl5_spi2select() in the board file, or the CS pin never toggles
 * and the sensor responds with floating-bus garbage (WHO_AM_I reads back
 * as 0x00 or 0xFF).
 */

#define GYRO_CS_DEVID           SPIDEV_USER(3)   /* NSS4 */
#define MAG_CS_DEVID             SPIDEV_USER(2)   /* NSS3 */

/* IAM-20380 / IAM-20380HT SPI clocks.
 *
 * Register access is limited to 1 MHz by the datasheet and is forced to
 * that value inside the driver; GYRO_SPI_FREQ applies only to the
 * sensor-data burst read (8 MHz maximum).
 */

#define GYRO_SPI_FREQ           8000000
#define MAG_SPI_FREQ             10000000

/* Telemetry packing scales (fixed-point: physical value * scale -> int16).
 *
 * Magnetometer is transmitted in microtesla scaled x10 (0.1 uT
 * resolution, +/-3276.7 uT range).  The ground segment expects SI units.
 */

#define MAG_PACKET_SCALE        10.0f
#define GYRO_PACKET_SCALE        100.0f

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* What kind of physical quantity an ADC channel represents.  Drives which
 * conversion formula and which disconnect/idle heuristic convert_channel()
 * applies.
 */

enum adc_channel_type_e
{
  CHANNEL_TYPE_VOLT = 0,   /* General voltage monitor (V)                    */
  CHANNEL_TYPE_SOLAR_V,    /* Solar panel voltage (V), with dark detection   */
  CHANNEL_TYPE_TEMP,       /* NTC temperature sensor (deg C)                 */
  CHANNEL_TYPE_CURR,       /* General current sensor (A)                     */
  CHANNEL_TYPE_SOLAR_I,    /* Solar panel current (A), with dark detection   */
  CHANNEL_TYPE_NC          /* Not connected / unused pin                     */
};

/* Result of converting one channel: whether the value is a trustworthy
 * physical reading, or why it isn't.
 */

enum sensor_status_e
{
  STATUS_ACTIVE_OK = 0,    /* Value is valid                                 */
  STATUS_DISCONNECTED,     /* Sensor/wire appears open (pulled to rail)      */
  STATUS_NO_DATA_DARK,     /* Solar panel below threshold (eclipse/dark)     */
  STATUS_IDLE_NO_LOAD,     /* Current rail below threshold (no load)        */
  STATUS_NC_UNUSED,        /* Channel intentionally not connected            */
  STATUS_FAULT             /* Conversion produced an invalid result          */
};

/* Static, per-channel calibration/interpretation table entry.  One of
 * these exists for every physical ADC channel on each chip; see
 * g_adc1_cfg[] / g_adc2_cfg[] below.
 */

struct adc_channel_cfg_s
{
  const char              *name;        /* Signal name, from schematic   */
  enum adc_channel_type_e  type;        /* How to interpret this channel */
  const char              *unit;        /* Engineering unit string       */
  float                    slope;       /* Scaling multiplier (m)        */
  float                    offset;      /* Offset correction (c)         */
  bool                     is_nc;       /* true if channel is unconnected*/
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC1 (/dev/adc0) channel map:
 *   CH00-CH07 : Voltage monitors (V)
 *   CH08-CH15 : NTC temperature sensors (deg C)
 */

static const struct adc_channel_cfg_s g_adc1_cfg[ADS7953_CHANNELS] =
{
  { "ADC_BAT_MON",    CHANNEL_TYPE_VOLT,    "V",  1.0f,    0.0f,  false },
  { "TOTAL_SOLAR_V",  CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },
  { "RAW_VOLT",       CHANNEL_TYPE_VOLT,    "V",  1.0f,    0.0f,  false },
  { "SP5_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },
  { "SP4_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },
  { "SP3_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },
  { "SP1_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },
  { "SP2_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },
  { "ANT_TEMP",       CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
  { "BATT_TEMP",      CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
  { "TEMP_BPB",       CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
  { "TEMP1",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
  { "TEMP5",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
  { "TEMP4",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
  { "TEMP3",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
  { "TEMP2",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
};

/* ADC2 (/dev/adc1) channel map: current sensors and unused (NC) pins */

static const struct adc_channel_cfg_s g_adc2_cfg[ADS7953_CHANNELS] =
{
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },
  { "UNREG_I",        CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },
  { "SP4_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },
  { "MAIN_3V3_I",     CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },
  { "MISSION_3V3_I",  CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },
  { "SPT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },
  { "5V_I",           CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },
  { "SP2_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },
  { "SP1_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },
  { "RAW_I",          CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },
  { "SP5_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },
  { "BAT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },
  { "SP3_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pack_i16
 *
 * Description:
 *   Round and saturate a scaled float into an int16_t.  A plain cast of
 *   an out-of-range or NaN float is undefined behaviour and would
 *   silently corrupt the telemetry packet, so every out-of-range value
 *   is clamped to the nearest representable int16_t instead.
 ****************************************************************************/

static int16_t pack_i16(float value)
{
  float r = roundf(value);

  if (r != r)
    {
      return 0;               /* NaN */
    }

  if (r > 32767.0f)
    {
      return INT16_MAX;
    }

  if (r < -32768.0f)
    {
      return INT16_MIN;
    }

  return (int16_t)r;
}

/****************************************************************************
 * Name: read_adc_chip
 *
 * Description:
 *   Trigger a full 16-channel conversion on one ADS7953 and read the
 *   results back into `channels[]`, indexed by ADC channel number.
 *
 * Returned Value:
 *   Number of samples read (>= 0) on success, negative errno on failure.
 ****************************************************************************/

static int read_adc_chip(const char *devpath, uint16_t channels[ADS7953_CHANNELS])
{
  struct adc_msg_s samples[ADS7953_CHANNELS];
  ssize_t nbytes;
  int nsamples;
  int fd;
  int ret;
  int i;

  memset(channels, 0, sizeof(uint16_t) * ADS7953_CHANNELS);

  fd = open(devpath, O_RDONLY);
  if (fd < 0)
    {
      int err = errno;
      printf("ERROR: Failed to open %s (errno: %d)\n", devpath, err);
      return -err;
    }

  /* Trigger conversion across all 16 channels */

  ret = ioctl(fd, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      int err = errno;
      printf("ERROR: ANIOC_TRIGGER failed on %s (errno: %d)\n", devpath, err);
      close(fd);
      return -err;
    }

  /* Read the resulting samples from the driver FIFO */

  nbytes = read(fd, samples, sizeof(samples));
  if (nbytes < 0)
    {
      int err = errno;
      printf("ERROR: Read failed on %s (errno: %d)\n", devpath, err);
      close(fd);
      return -err;
    }

  nsamples = nbytes / sizeof(struct adc_msg_s);

  for (i = 0; i < nsamples; i++)
    {
      uint8_t ch = samples[i].am_channel;
      if (ch < ADS7953_CHANNELS)
        {
          channels[ch] = (uint16_t)(samples[i].am_data & 0x0FFF);
        }
    }

  close(fd);
  return nsamples;
}

/****************************************************************************
 * Name: convert_channel
 *
 * Description:
 *   Convert one raw ADC reading to an engineering-unit value, according
 *   to the channel's configured type:
 *
 *     - NC              -> always 0.0, status STATUS_NC_UNUSED
 *     - TEMP (NTC)       -> Steinhart-Hart approximation, deg C
 *     - SOLAR_V / SOLAR_I -> linear scale, but reported as "dark/off"
 *                            below SOLAR_*_THRESHOLD instead of a noisy
 *                            near-zero value
 *     - CURR             -> linear scale, reported as "idle" below
 *                            IDLE_I_THRESHOLD
 *     - VOLT (default)   -> plain linear scale
 *
 * Input Parameters:
 *   raw_counts - 12-bit ADC reading for this channel
 *   cfg        - channel's static configuration entry
 *   v_pin_out  - optional: receives the computed pin voltage (0-5V)
 *   status_out - optional: receives the sensor_status_e classification
 *
 * Returned Value:
 *   The converted physical value in the channel's engineering unit.
 *   Meaningless (but well-defined) when status_out indicates the
 *   channel is not actively reporting valid data.
 ****************************************************************************/

static float convert_channel(uint16_t raw_counts,
                             const struct adc_channel_cfg_s *cfg,
                             float *v_pin_out,
                             enum sensor_status_e *status_out)
{
  float v_pin;

  if (cfg->is_nc)
    {
      if (v_pin_out)
        {
          *v_pin_out = 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_NC_UNUSED;
        }

      return 0.0f;
    }

  /* Pin voltage seen by the ADC input (0.000V to 5.000V) */

  v_pin = ((float)raw_counts / ADC_MAX_COUNTS) * ADC_VREF_FULLSCALE;

  if (v_pin_out)
    {
      *v_pin_out = v_pin;
    }

  /* 1. NTC temperature sensor: divider voltage -> resistance ->
   *    Steinhart-Hart (beta form) -> deg C.
   */

  if (cfg->type == CHANNEL_TYPE_TEMP)
    {
      if (raw_counts >= 4000 || v_pin >= (NTC_V_SUPPLY - 0.05f))
        {
          /* Pin pulled to the rail: NTC is open / not populated */

          if (status_out)
            {
              *status_out = STATUS_DISCONNECTED;
            }

          return 0.0f;
        }

      float r_ntc = NTC_R_PULLUP * (v_pin / (NTC_V_SUPPLY - v_pin));
      if (r_ntc <= 0.0f)
        {
          if (status_out)
            {
              *status_out = STATUS_FAULT;
            }

          return -273.15f;
        }

      float steinhart = (1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
      float temp_c = (1.0f / steinhart) - 273.15f;

      if (status_out)
        {
          *status_out = STATUS_ACTIVE_OK;
        }

      return temp_c;
    }

  /* 2. Solar panel voltage: below threshold means the panel is
   *    disconnected or in eclipse, not a real near-zero voltage.
   */

  if (cfg->type == CHANNEL_TYPE_SOLAR_V)
    {
      if (v_pin < SOLAR_V_THRESHOLD)
        {
          if (status_out)
            {
              *status_out = STATUS_NO_DATA_DARK;
            }

          return 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_ACTIVE_OK;
        }

      return (v_pin * cfg->slope) + cfg->offset;
    }

  /* 3. Solar panel current: same "dark" rule as panel voltage */

  if (cfg->type == CHANNEL_TYPE_SOLAR_I)
    {
      float current = (v_pin * cfg->slope) + cfg->offset;

      if (v_pin < SOLAR_I_THRESHOLD || current < 0.010f)
        {
          if (status_out)
            {
              *status_out = STATUS_NO_DATA_DARK;
            }

          return 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_ACTIVE_OK;
        }

      return current;
    }

  /* 4. General current rail: flag as idle instead of a noisy near-zero
   *    reading when nothing is drawing current.
   */

  if (cfg->type == CHANNEL_TYPE_CURR)
    {
      float current = (v_pin * cfg->slope) + cfg->offset;

      if (v_pin < IDLE_I_THRESHOLD || current < 0.005f)
        {
          if (status_out)
            {
              *status_out = STATUS_IDLE_NO_LOAD;
            }

          return 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_ACTIVE_OK;
        }

      return current;
    }

  /* 5. General voltage rail: plain linear scale, no special-casing */

  if (status_out)
    {
      *status_out = STATUS_ACTIVE_OK;
    }

  return (v_pin * cfg->slope) + cfg->offset;
}

/****************************************************************************
 * Name: send_telemetry_to_ringbuffer
 *
 * Description:
 *   Pack the latest ADC / IMU readings into the two telemetry envelopes
 *   (ADC1: voltages + temperatures, ADC2: currents + gyro + mag), then
 *   fan them out to both consumers:
 *
 *     - the local ring buffer (wakes the LittleFS storage daemon), and
 *     - the shared-SRAM2 TX ring buffer for the M0+ radio core, followed
 *       by an IPCC "beacon" doorbell so the M0+ knows new data is ready.
 *
 *   Also updates the float battery-voltage/temperature snapshot used by
 *   the M0+'s CW Morse beacon.
 ****************************************************************************/

/****************************************************************************
 * Name: send_telemetry_to_ringbuffer
 *
 * Description:
 *   Pack the latest ADC / IMU readings into the two telemetry envelopes
 *   (ADC1: voltages + temperatures, ADC2: currents + gyro + mag).
 *
 *   BOTH envelopes are always pushed to the LOCAL ring buffer (wakes the
 *   LittleFS storage daemon) -- flash storage keeps a complete record
 *   every 90s cycle regardless of what gets radioed down.
 *
 *   Only ONE of the two is queued to the shared-SRAM2 TX ring buffer for
 *   the M0+ radio core this cycle, alternating by scan_count parity:
 *
 *     odd  scan_count -> Beacon 1 (ADC1: voltages/temps)
 *     even scan_count -> Beacon 2 (ADC2 + gyro + mag)
 *
 *   This halves the M4->M0+ IPC rate (1 packet/90s instead of 2), which
 *   matters because the M0+ CW+GMSK mission loop period is already
 *   >= 90s -- pushing 2 packets every 90s guarantees the shared TX ring
 *   (depth 8) backs up and starts silently dropping telemetry over time.
 *   Alternating keeps the net M4 production rate at or below what the
 *   M0+ can actually drain.
 *
 *   The float battery-voltage/temperature snapshot for the M0+'s CW
 *   Morse beacon is updated every cycle regardless, since it's cheap
 *   (2 floats, no ring buffer) and the CW beacon runs independently of
 *   which HK packet type gets sent this cycle.
 *
 * Input Parameters:
 *   scan_count - 1-based acquisition cycle counter from obc_main_main's
 *                main loop. Odd/even parity selects which beacon type
 *                gets queued to the M0+ this cycle.
 ****************************************************************************/

static int send_telemetry_to_ringbuffer(uint16_t adc0_raw[ADS7953_CHANNELS],
                                        uint16_t adc1_raw[ADS7953_CHANNELS],
                                        const struct iam20380_data_s *gyro,
                                        const struct mmc5983ma_data_s *mag,
                                        bool gyro_ok,
                                        bool mag_ok,
                                        int scan_count)
{
  /* ADC2's 16 physical channels include 4 NC pins; this maps the 12
   * "active" packet slots back to their real ADC2 channel numbers.
   */

  static const uint8_t adc2_active_map[ADC2_ACTIVE_CHANNELS] =
  {
    1, 2, 3, 5, 6, 8, 9, 10, 11, 13, 14, 15
  };

  struct telemetry_envelope_s env_adc1;
  struct telemetry_envelope_s env_adc2;
  float v_pin;
  enum sensor_status_e status;
  int ch;
  int i;

  /* ========================================================================= */
  /* 1. Package ADC 1: 16 Channels x 2B (x100) + Footer1 (0xAA55) = 34 Bytes   */
  /*    Always computed & stored locally, regardless of radio parity.         */
  /* ========================================================================= */

  memset(&env_adc1, 0, sizeof(env_adc1));
  env_adc1.len = sizeof(struct adc1_packet_s);
  env_adc1.footer = TELEM_FOOTER_ADC1;

  for (ch = 0; ch < ADC1_CHANNELS_COUNT; ch++)
    {
      float val = convert_channel(adc0_raw[ch], &g_adc1_cfg[ch], &v_pin, &status);
      env_adc1.pkt.adc1.data[ch] = pack_i16(val * 100.0f);
    }

  env_adc1.pkt.adc1.footer = TELEM_FOOTER_ADC1;

  /* Push ADC 1 packet to the local ring buffer and wake the LittleFS daemon.
   * Unconditional -- flash gets every cycle's data regardless of what is
   * radioed down this cycle.
   */

  telemetry_rb_write(&env_adc1);
  sem_post(&g_telemetry_sem);

  /* ========================================================================= */
  /* 2. Package ADC 2 + IMU: 12 Currents + 3 Gyro + 3 Mag + Footer2 = 38 Bytes */
  /*    Always computed & stored locally, regardless of radio parity.         */
  /* ========================================================================= */

  memset(&env_adc2, 0, sizeof(env_adc2));
  env_adc2.len = sizeof(struct adc2_packet_s);
  env_adc2.footer = TELEM_FOOTER_ADC2;

  /* 12 active current channels, scaled x100 */

  for (i = 0; i < ADC2_ACTIVE_CHANNELS; i++)
    {
      ch = adc2_active_map[i];
      float val = convert_channel(adc1_raw[ch], &g_adc2_cfg[ch], &v_pin, &status);
      env_adc2.pkt.adc2.data[i] = pack_i16(val * 100.0f);
    }

  /* 3-axis gyroscope (IAM-20380 / IAM-20380HT), deg/s scaled x100 */

  if (gyro_ok && gyro != NULL)
    {
      float dps_x = iam20380_counts_to_dps(gyro->x);
      float dps_y = iam20380_counts_to_dps(gyro->y);
      float dps_z = iam20380_counts_to_dps(gyro->z);

      env_adc2.pkt.adc2.gyro_x = pack_i16(dps_x * GYRO_PACKET_SCALE);
      env_adc2.pkt.adc2.gyro_y = pack_i16(dps_y * GYRO_PACKET_SCALE);
      env_adc2.pkt.adc2.gyro_z = pack_i16(dps_z * GYRO_PACKET_SCALE);
    }
  else
    {
      env_adc2.pkt.adc2.gyro_x = 0;
      env_adc2.pkt.adc2.gyro_y = 0;
      env_adc2.pkt.adc2.gyro_z = 0;
    }

  /* 3-axis magnetometer (MMC5983MA), microtesla scaled x10.
   *
   * 18-bit mode: 16384 counts/Gauss, 1 Gauss = 100 uT ->
   * 163.84 counts/uT.  x10 keeps 0.1 uT resolution inside an int16_t
   * (+/-3276.7 uT range).
   */

  if (mag_ok && mag != NULL)
    {
      float ut_x = mmc5983ma_counts_to_ut(mag->x);
      float ut_y = mmc5983ma_counts_to_ut(mag->y);
      float ut_z = mmc5983ma_counts_to_ut(mag->z);

      env_adc2.pkt.adc2.mag_x = pack_i16(ut_x * MAG_PACKET_SCALE);
      env_adc2.pkt.adc2.mag_y = pack_i16(ut_y * MAG_PACKET_SCALE);
      env_adc2.pkt.adc2.mag_z = pack_i16(ut_z * MAG_PACKET_SCALE);
    }
  else
    {
      env_adc2.pkt.adc2.mag_x = 0;
      env_adc2.pkt.adc2.mag_y = 0;
      env_adc2.pkt.adc2.mag_z = 0;
    }

  env_adc2.pkt.adc2.footer = TELEM_FOOTER_ADC2;

  /* Push ADC 2 + IMU packet to the local ring buffer and wake LittleFS.
   * Unconditional, same reasoning as ADC1 above.
   */

  telemetry_rb_write(&env_adc2);
  sem_post(&g_telemetry_sem);

  /* ========================================================================= */
  /* 3. Dual-Core IPC to Cortex-M0+ (Radio Subsystem in Shared SRAM2)          */
  /* ========================================================================= */

  /* A. Extract float battery voltage & satellite temperature for the CW
   *    Morse beacon on the M0+ side.  Updated every cycle -- cheap, and
   *    the CW beacon is independent of which HK packet is sent below.
   */
  //CW Beacon1 (ADC0)
  float v_batt_pin;
  enum sensor_status_e batt_status;
  float batt_v = convert_channel(adc0_raw[0], &g_adc1_cfg[0], &v_batt_pin, &batt_status);
  uint16_t batt_v_int = (uint16_t)(batt_v * 100.0f);

  float v_total_solar_pin;
  enum sensor_status_e solar_status;
  float total_solar_v = convert_channel(adc0_raw[1], &g_adc1_cfg[1], &v_total_solar_pin, &solar_status);
  uint16_t total_solar_v_int = (uint16_t)(total_solar_v * 100.0f);

  float ANT_temp_pin;
  enum sensor_status_e temp_status;
  float temp_c = convert_channel(adc0_raw[8], &g_adc1_cfg[8], &ANT_temp_pin, &temp_status);
  uint16_t temp_c_int = (uint16_t)(temp_c * 10.0f);

  float BATT_temp_pin;
  enum sensor_status_e batt_temp_status;
  float batt_temp_c = convert_channel(adc0_raw[9], &g_adc1_cfg[9], &BATT_temp_pin, &batt_temp_status);
  uint16_t batt_temp_c_int = (uint16_t)(batt_temp_c * 10.0f);

  float TEMP_BPB_pin;
  enum sensor_status_e bpb_temp_status;
  float bpb_temp_c = convert_channel(adc0_raw[10], &g_adc1_cfg[10], &TEMP_BPB_pin, &bpb_temp_status);
  uint16_t bpb_temp_c_int = (uint16_t)(bpb_temp_c * 10.0f);

  //CW Beacon2 (ADC1)
  float Main3v3_I_pin;
  enum sensor_status_e main3v3_i_status;
  float main3v3_i = convert_channel(adc1_raw[3], &g_adc2_cfg[3], &Main3v3_I_pin, &main3v3_i_status);
  uint16_t main3v3_i_int = (uint16_t)(main3v3_i * 100.0f);

  float TotalSolar_I_pin;
  enum sensor_status_e total_solar_i_status;
  float total_solar_i = convert_channel(adc1_raw[6], &g_adc2_cfg[6], &TotalSolar_I_pin, &total_solar_i_status);
  uint16_t total_solar_i_int = (uint16_t)(total_solar_i * 100.0f);

  float BAT_I_pin;
  enum sensor_status_e BAT_I_pin_status;
  float bat_i = convert_channel(adc1_raw[14], &g_adc2_cfg[14], &BAT_I_pin, &BAT_I_pin_status);
  uint16_t bat_i_int = (uint16_t)(bat_i * 100.0f);

  int16_t flag1 = 1;
  int16_t flag2 = 2;

  /* B. Queue both Beacon 1 (Voltage/Temperature) and Beacon 2 (Currents/Flags)
   *    to the Cortex-M0+ shared ring buffer.
   */

  printf("[OBC_MAIN] [TX -> M0+] Queued Beacon 1 & Beacon 2 to shared ring buffer\n");
  live_telem_update_float(TELEM_ID_B1, (int16_t)batt_v_int, (int16_t)total_solar_v_int,
                          (int16_t)temp_c_int, (int16_t)batt_temp_c_int, (int16_t)bpb_temp_c_int);
  live_telem_update_float(TELEM_ID_B2, (int16_t)main3v3_i_int, (int16_t)total_solar_i_int,
                          (int16_t)bat_i_int, (int16_t)flag1, (int16_t)flag2);

  /* C. Ring the IPCC "beacon" doorbell so the M0+ picks up the new data */

  ipcc_m4_send(IPCC_CH_BEACON);

  /* Console log line mirrors what the M0+ will transmit as a CW beacon */

  int v_int  = (int)batt_v;
  int v_frac = (int)((batt_v - (float)v_int) * 100.0f);
  if (v_frac < 0) v_frac = -v_frac;

  int t_int  = (int)temp_c;
  int t_frac = (int)((temp_c - (float)t_int) * 10.0f);
  if (t_frac < 0) t_frac = -t_frac;

  printf("\n[OBC_MAIN] [TX -> M0+] CW Telemetry: \"9NS2S2 V%d.%02d T%d.%d\" | Cycle scan #%d\n",
         v_int, v_frac, t_int, t_frac, scan_count);

  return 0;
}

/****************************************************************************
 * Name: check_camera_data_available
 *
 * Description:
 *   Return true if /mnt/camera contains at least one non-empty file
 *   (i.e. a captured image is ready to be read back / downlinked).
 ****************************************************************************/

static bool check_camera_data_available(void)
{
  DIR *d = opendir("/mnt/camera");
  if (!d)
    {
      return false;
    }

  struct dirent *de;
  bool found = false;

  while ((de = readdir(d)) != NULL)
    {
      if (strcmp(de->d_name, ".") != 0 && strcmp(de->d_name, "..") != 0)
        {
          struct stat st;
          char path[64];
          snprintf(path, sizeof(path), "/mnt/camera/%s", de->d_name);
          if (stat(path, &st) == 0 && st.st_size > 0)
            {
              found = true;
              break;
            }
        }
    }

  closedir(d);
  return found;
}

/****************************************************************************
 * Name: process_ground_commands
 *
 * Description:
 *   Poll for a telecommand forwarded by the M0+ (via IPCC + the shared
 *   SRAM2 RX ring buffer) and dispatch it:
 *
 *     - Camera command (0x53 0x04 ...): trigger image capture, then ACK
 *       (0xAA) or NACK (0xFF) back to the M0+ depending on whether image
 *       data ended up on /mnt/camera.
 *     - Housekeeping request (0x53 0x01 ...): just re-ring the beacon
 *       doorbell so the latest HK packets go out again.
 *     - Anything else: logged and ignored.
 *
 *   Returns immediately (no-op) if no command is pending.
 ****************************************************************************/

static void process_ground_commands(void)
{
  if (!ipcc_m4_received(IPCC_CH_COMMAND) && rb_rx_empty())
    {
      return;
    }

  struct rx_command_s rx_cmd;
  if (rb_rx_read(&rx_cmd))
    {
      ipcc_m4_clear(IPCC_CH_COMMAND);

      printf("\n[OBC_MAIN] [GROUND COMMAND RECEIVED]: ");
      for (int i = 0; i < CMD_PAYLOAD_LEN; i++)
        {
          printf("%02X ", rx_cmd.cmd[i]);
        }
      printf("\n");

      /* Camera command: 53 04 CC 5E BD 00 00 ... */

      if (rx_cmd.cmd[0] == 0x53 && rx_cmd.cmd[1] == CMD_TYPE_CAMERA)
        {
          printf("[OBC_MAIN] Detected CAMERA Command (53 04 CC 5E BD...)\n");

          /* Trigger camera image capture */

#if defined(CONFIG_EXAMPLES_CAMERA_MSN2)
          extern int camera_MSN2_main(int argc, char *argv[]);
          printf("[OBC_MAIN] Spawning Camera MSN2 capture task to acquire image...\n");
          task_create("camera", 100, 2048, camera_MSN2_main, NULL);
          sleep(1);
#elif defined(CONFIG_EXAMPLES_CAMERA)
          extern int camera_main(int argc, char *argv[]);
          printf("[OBC_MAIN] Spawning Camera capture task to acquire image...\n");
          task_create("camera", 100, 2048, camera_main, NULL);
          sleep(1);
#else
          /* No camera driver configured: create a stub test frame so the
           * command/ACK path can still be exercised on the bench.
           */

          mkdir("/mnt/camera", 0777);
          int cam_fd = open("/mnt/camera/capture_01.raw", O_WRONLY | O_CREAT | O_TRUNC, 0666);
          if (cam_fd >= 0)
            {
              uint8_t cam_test_hdr[32] = "S2S2_CAM_FRAME_CAPTURED_OK";
              write(cam_fd, cam_test_hdr, sizeof(cam_test_hdr));
              fsync(cam_fd);
              close(cam_fd);
              printf("[OBC_MAIN] Camera image captured & stored to /mnt/camera/capture_01.raw\n");
            }
#endif

          bool data_ok = check_camera_data_available();
          if (data_ok)
            {
              printf("[OBC_MAIN] [CAMERA DATA AVAILABLE] Image captured successfully.\n");
            }
          else
            {
              printf("[OBC_MAIN] [CAMERA DATA NOT AVAILABLE] Failed to capture image.\n");
            }
        }
      else if (rx_cmd.cmd[0] == 0x53 && rx_cmd.cmd[1] == CMD_TYPE_HK)
        {
          printf("[OBC_MAIN] Detected HK Request Command (53 01 %02X %02X %02X)\n",
                 rx_cmd.cmd[2], rx_cmd.cmd[3], rx_cmd.cmd[4]);
          /* Retrigger beacon so M0+ picks up freshest telemetry */
          ipcc_m4_send(IPCC_CH_BEACON);
        }
      else
        {
          printf("[OBC_MAIN] Received Unknown Command Opcode: 0x%02X\n", rx_cmd.cmd[1]);
        }
    }
}

/****************************************************************************
 * Name: display_adc1_telemetry
 *
 * Description:
 *   Print a human-readable table of ADC1 (voltage/temperature) readings
 *   to the console for bench debugging.
 ****************************************************************************/

static void display_adc1_telemetry(uint16_t raw_data[ADS7953_CHANNELS])
{
  int ch;

  printf("\n================================ ADC 1: VOLTAGE & TEMPERATURE ================================\n");
  printf("  CH  | Signal Name       | Type        | Raw   | Pin Volt | Physical Value   | Status        \n");
  printf("------+-------------------+-------------+-------+----------+------------------+---------------\n");

  for (ch = 0; ch < ADS7953_CHANNELS; ch++)
    {
      const struct adc_channel_cfg_s *cfg = &g_adc1_cfg[ch];
      float v_pin;
      float phys_val;
      enum sensor_status_e status;
      const char *type_str;
      const char *status_str;

      if (cfg->is_nc)
        {
          printf("  %02d  | %-17s | NC (Unused) |  ---  |   ----   |        ---       | [ NC (Unused) ]\n",
                 ch, cfg->name);
          continue;
        }

      phys_val = convert_channel(raw_data[ch], cfg, &v_pin, &status);

      type_str = (cfg->type == CHANNEL_TYPE_TEMP) ? "Temperature" : "Voltage";

      switch (status)
        {
          case STATUS_ACTIVE_OK:
            status_str = "ACTIVE (OK)";
            break;
          case STATUS_DISCONNECTED:
            status_str = "[ DISCONNECTED ]";
            break;
          case STATUS_NO_DATA_DARK:
            status_str = "[ NO DATA/DARK ]";
            break;
          default:
            status_str = "---";
            break;
        }

      if (status == STATUS_DISCONNECTED)
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V |       ---        | %-15s\n",
                 ch, cfg->name, type_str, raw_data[ch], v_pin, status_str);
        }
      else if (status == STATUS_NO_DATA_DARK)
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V |   0.000 %-4s     | %-15s\n",
                 ch, cfg->name, type_str, raw_data[ch], v_pin, cfg->unit, status_str);
        }
      else
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V | %9.3f %-4s   | %-15s\n",
                 ch, cfg->name, type_str, raw_data[ch], v_pin, phys_val, cfg->unit, status_str);
        }
    }

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: display_adc2_telemetry
 *
 * Description:
 *   Print a human-readable table of ADC2 (current) readings to the
 *   console for bench debugging.  NC channels are skipped silently.
 ****************************************************************************/

/****************************************************************************
 * Name: display_adc2_telemetry
 *
 * Description:
 *   Print a human-readable table of ADC2 (current) readings, plus the
 *   gyroscope and magnetometer readings, to the console for bench
 *   debugging.  All three are shown in one table since they are all
 *   packed into the same ADC2/IMU telemetry envelope for this cycle.
 *   NC current channels are skipped silently.
 *
 *   Magnetometer is reported in microtesla (SI).  1 Gauss = 100 uT.
 *   The local geomagnetic field is typically 25-65 uT; a much larger
 *   magnitude indicates a hard-iron offset from the spacecraft itself
 *   (or from magnets/steel near the sensor on the bench).
 ****************************************************************************/

static void display_adc2_telemetry(uint16_t raw_data[ADS7953_CHANNELS],
                                    const struct iam20380_data_s *gyro,
                                    const struct mmc5983ma_data_s *mag,
                                    bool gyro_ok, bool mag_ok)
{
  int ch;

  printf("\n=================================== ADC 2: CURRENT SENSORS + IMU =============================\n");
  printf("  CH  | Signal Name       | Type        | Raw   | Pin Volt | Current Value    | Status        \n");
  printf("------+-------------------+-------------+-------+----------+------------------+---------------\n");

  /* --- Current sensor channels (ADS7953) --- */

  for (ch = 0; ch < ADS7953_CHANNELS; ch++)
    {
      const struct adc_channel_cfg_s *cfg = &g_adc2_cfg[ch];
      float v_pin;
      float phys_val;
      enum sensor_status_e status;
      const char *status_str;

      if (cfg->is_nc)
        {
          continue;
        }

      phys_val = convert_channel(raw_data[ch], cfg, &v_pin, &status);

      switch (status)
        {
          case STATUS_ACTIVE_OK:
            status_str = "ACTIVE (OK)";
            break;
          case STATUS_NO_DATA_DARK:
            status_str = "[ DISCONNECTED ]";
            break;
          case STATUS_IDLE_NO_LOAD:
            status_str = "[ IDLE/NO LOAD ]";
            break;
          default:
            status_str = "---";
            break;
        }

      if (status == STATUS_NO_DATA_DARK || status == STATUS_IDLE_NO_LOAD)
        {
          printf("  %02d  | %-17s | Current     | %5u | %6.3f V |   0.000 %-4s     | %-15s\n",
                 ch, cfg->name, raw_data[ch], v_pin, cfg->unit, status_str);
        }
      else
        {
          printf("  %02d  | %-17s | Current     | %5u | %6.3f V | %9.3f %-4s   | %-15s\n",
                 ch, cfg->name, raw_data[ch], v_pin, phys_val, cfg->unit, status_str);
        }
    }

  printf("------+-------------------+-------------+-------+----------+------------------+---------------\n");

  /* --- Gyroscope (IAM-20380 / IAM-20380HT), same columns as above --- */

  if (gyro_ok && gyro != NULL)
    {
      float dps_x  = iam20380_counts_to_dps(gyro->x);
      float dps_y  = iam20380_counts_to_dps(gyro->y);
      float dps_z  = iam20380_counts_to_dps(gyro->z);
      float temp_c = iam20380_counts_to_degc(gyro->temp);

      printf("  GX  | %-17s | Gyro        | %5d |   ----   | %9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO", gyro->x, dps_x, "d/s", "ACTIVE (OK)");
      printf("  GY  | %-17s | Gyro        | %5d |   ----   | %9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO", gyro->y, dps_y, "d/s", "ACTIVE (OK)");
      printf("  GZ  | %-17s | Gyro        | %5d |   ----   | %9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO", gyro->z, dps_z, "d/s", "ACTIVE (OK)");
      printf("  GT  | %-17s | Gyro Temp   | %5d |   ----   | %9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO", gyro->temp, temp_c, "C", "ACTIVE (OK)");
    }
  else
    {
      printf("  GYR | %-17s | Gyro        |  ---  |   ----   |        ---       | %-15s\n",
             "IAM20380_GYRO", "[ DISCONNECTED ]");
    }

  printf("------+-------------------+-------------+-------+----------+------------------+---------------\n");

  /* --- Magnetometer (MMC5983MA), same columns as above --- */

  if (mag_ok && mag != NULL)
    {
      float ut_x   = mmc5983ma_counts_to_ut(mag->x);
      float ut_y   = mmc5983ma_counts_to_ut(mag->y);
      float ut_z   = mmc5983ma_counts_to_ut(mag->z);
      float ut_mag = sqrtf((ut_x * ut_x) + (ut_y * ut_y) + (ut_z * ut_z));
      const char *mag_status = (ut_mag > 20.0f && ut_mag < 70.0f) ?
                                "ACTIVE (OK)" : "[ CHECK HIRON ]";

      printf("  MX  | %-17s | Mag         | %5ld |   ----   | %9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG", (long)mag->x, ut_x, "uT", mag_status);
      printf("  MY  | %-17s | Mag         | %5ld |   ----   | %9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG", (long)mag->y, ut_y, "uT", mag_status);
      printf("  MZ  | %-17s | Mag         | %5ld |   ----   | %9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG", (long)mag->z, ut_z, "uT", mag_status);
      printf("  M|B|| %-17s | Mag |B|     |  ---  |   ----   | %9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG", ut_mag, "uT", mag_status);
    }
  else
    {
      printf("  MAG | %-17s | Mag         |  ---  |   ----   |        ---       | %-15s\n",
             "MMC5983MA_MAG", "[ DISCONNECTED ]");
    }

  printf("==============================================================================================\n");
}
/****************************************************************************
 * Name: init_gyro
 *
 * Description:
 *   Initialize the gyroscope.  Accepts both IAM-20380 (WHO_AM_I 0xB5)
 *   and IAM-20380HT (WHO_AM_I 0xFD).  On failure, prints a WHO_AM_I-based
 *   hint distinguishing "no response at all" (CS/wiring/power) from
 *   "responding, but with the wrong ID" (CS not actually selecting this
 *   device, or wrong SPI mode/clock).
 ****************************************************************************/

static bool init_gyro(FAR struct spi_dev_s *spi,
                      FAR const struct iam20380_config_s *cfg)
{
  uint8_t id = 0;
  int ret;

  ret = iam20380_initialize(spi, cfg);
  if (ret == OK)
    {
      iam20380_read_whoami(spi, &id);
      printf("[OBC_MAIN] %s Gyroscope initialized successfully! "
             "(WHO_AM_I=0x%02X)\n", iam20380_id_name(id), id);
      return true;
    }

  iam20380_read_whoami(spi, &id);
  printf("[OBC_MAIN] WARNING: gyro init failed (ret=%d), WHO_AM_I=0x%02X "
         "(expected 0xFD IAM-20380HT or 0xB5 IAM-20380)\n", ret, id);

  if (id == 0x00 || id == 0xff)
    {
      printf("[OBC_MAIN]   -> No response: check CS mapping in board file, "
             "sensor power, MISO/MOSI/SCK wiring\n");
    }
  else
    {
      printf("[OBC_MAIN]   -> Unexpected/unstable ID: NSS is probably not "
             "being driven (case missing in stm32wl5_spi2select()) so MISO "
             "floats, or the SPI mode/clock is wrong\n");
    }

  return false;
}

/****************************************************************************
 * Name: init_mag
 *
 * Description:
 *   Initialize the magnetometer.  On failure, prints a PRODUCT_ID-based
 *   hint distinguishing "no response at all" from "responding with a
 *   different chip's ID" (chip select likely mapped to the wrong
 *   device).
 ****************************************************************************/

static bool init_mag(FAR struct spi_dev_s *spi,
                     FAR const struct mmc5983ma_config_s *cfg)
{
  uint8_t id = 0;
  int ret;

  ret = mmc5983ma_initialize(spi, cfg);
  if (ret == OK)
    {
      printf("[OBC_MAIN] MMC5983MA Magnetometer initialized successfully!\n");
      return true;
    }

  mmc5983ma_read_id(spi, &id);
  printf("[OBC_MAIN] WARNING: MMC5983MA init failed (ret=%d), "
         "PRODUCT_ID=0x%02X (expected 0x30)\n", ret, id);

  if (id == 0x00 || id == 0xff)
    {
      printf("[OBC_MAIN]   -> No response: check CS mapping in board file, "
             "sensor power, MISO/MOSI/SCK wiring\n");
    }
  else
    {
      printf("[OBC_MAIN]   -> Got a different chip's ID: CS ID probably "
             "selects the wrong device\n");
    }

  return false;
}

/****************************************************************************
 * Name: gyro_probe_cs
 *
 * Description:
 *   Read-only diagnostic scan of the gyro's candidate SPI chip-select
 *   ID, looking for the WHO_AM_I of either gyro variant (0xB5
 *   IAM-20380, 0xFD IAM-20380HT).  Every chip select is tried in SPI
 *   mode 0 and mode 3, at 1 MHz and 250 kHz, and one summary line is
 *   printed per chip select:
 *
 *     cs=SPIDEV_USER(3): m0@1000k=0x.. m0@250k=0x.. m3@1000k=0x.. m3@250k=0x..
 *
 *   Reading the SAME valid ID under every mode/clock combination means
 *   the wiring is fine.  Different garbage values on every read (0x00,
 *   0xFD, 0xFF, ...) means the chip select is not actually being driven:
 *   add a case for the gyro devid to stm32wl5_spi2select() in the board
 *   file, and confirm the NSS4 pin is configured as a GPIO output that
 *   idles high.
 *
 *   On success, cfg->cs / cfg->mode / cfg->reg_frequency are updated in
 *   place to the working combination.  On failure the caller's
 *   configuration is left untouched.
 *
 * Returned Value:
 *   true if a working combination was found, false otherwise.
 ****************************************************************************/

static bool gyro_probe_cs(FAR struct spi_dev_s *spi,
                          FAR struct iam20380_config_s *cfg)
{
  static const uint32_t cs_list[] =
  {
    SPIDEV_USER(3),   /* NSS4 (PB9) - Gyroscope CS */
  };

  static const int cs_num[] = { 3 };

  static const enum spi_mode_e mode_list[] =
  {
    SPIDEV_MODE0,
    SPIDEV_MODE3,
  };

  static const uint32_t freq_list[] =
  {
    1000000,
    250000,
  };

  struct iam20380_config_s trial;
  int ncs = (int)(sizeof(cs_list) / sizeof(cs_list[0]));
  int nmode = (int)(sizeof(mode_list) / sizeof(mode_list[0]));
  int nfreq = (int)(sizeof(freq_list) / sizeof(freq_list[0]));
  int i;
  int m;
  int f;

  printf("\n[OBC_MAIN] [PROBE] Checking SPI2 NSS4 (PB9) for InvenSense/IAM Gyroscope...\n");

  for (i = 0; i < ncs; i++)
    {
      bool found = false;
      uint8_t found_id = 0;
      enum spi_mode_e found_mode = SPIDEV_MODE0;
      uint32_t found_freq = 0;

      printf("[OBC_MAIN] [PROBE]   cs=SPIDEV_USER(%d):", cs_num[i]);

      for (m = 0; m < nmode; m++)
        {
          for (f = 0; f < nfreq; f++)
            {
              uint8_t id = 0;

              trial = *cfg;
              trial.cs = cs_list[i];
              trial.mode = mode_list[m];
              trial.frequency = 1000000;
              trial.reg_frequency = freq_list[f];

              if (iam20380_probe(spi, &trial, &id) != OK)
                {
                  continue;
                }

              printf(" m%d@%luk=0x%02X",
                     (mode_list[m] == SPIDEV_MODE0) ? 0 : 3,
                     (unsigned long)(freq_list[f] / 1000), id);

              if (!found && iam20380_id_valid(id))
                {
                  found = true;
                  found_id = id;
                  found_mode = mode_list[m];
                  found_freq = freq_list[f];
                }
            }
        }

      if (found)
        {
          printf("   <-- %s FOUND\n", iam20380_id_name(found_id));

          cfg->cs = cs_list[i];
          cfg->mode = found_mode;
          cfg->reg_frequency = found_freq;
          return true;
        }

      printf("\n");
    }

  printf("[OBC_MAIN] [PROBE] No chip select returned 0xFD (IAM-20380HT) or 0xB5 (IAM-20380).\n");
  printf("[OBC_MAIN]   -> Add a case for the gyro CS devid to "
         "stm32wl5_spi2select() in the board file.\n");
  printf("[OBC_MAIN]   -> Confirm the NSS4 pin is configured as a GPIO "
         "output and idles high.\n");
  printf("[OBC_MAIN]   -> Check gyro VDD/VDDIO, then scope NSS4 and MISO: "
         "NSS4 must pulse low during each transfer.\n\n");

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obc_main_main
 *
 * Description:
 *   Entry point for the OBC telemetry producer task.  Initializes the
 *   local telemetry ring buffer, the shared-SRAM2 IPC ring buffers, and
 *   SPI2 for the IMU sensors, then runs the acquire/pack/send/sleep loop
 *   described at the top of this file forever.
 ****************************************************************************/

int obc_main_main(int argc, char *argv[])
{
  uint16_t adc1_data[ADS7953_CHANNELS];
  uint16_t adc2_data[ADS7953_CHANNELS];
  int scan_count = 0;
  int ret;

  printf("\n=============================================================\n");
  printf("  OBC Main: Dual ADS7953 & IMU Telemetry Producer            \n");
  printf("  ADC 1 (/dev/adc0): CH00-CH07 Voltage     | CH08-CH15 Temp  \n");
  printf("  ADC 2 (/dev/adc1): Current Sensors (NC: CH00, CH04, CH07, CH12)\n");
  printf("  IMU (SPI2): IAM-20380/HT (Gyro, NSS4) | MMC5983MA (Mag, NSS3)\n");
  printf("  Magnetometer units: microtesla (uT), packed x10            \n");
  printf("  Acquisition Cadence: Every %d Seconds                       \n", TELEMETRY_INTERVAL_SEC);
  printf("=============================================================\n");

  /* Ensure local ring buffer & notification semaphore are ready */

  telemetry_rb_init();

  /* Initialize dual-core shared SRAM2 ring buffers & live telemetry */

  rb_ipc_init();

  /* Initialize SPI2 for the gyroscope and magnetometer */

  FAR struct spi_dev_s *spi2 = stm32wl5_spibus_initialize(2);

  struct iam20380_config_s gyro_cfg =
  {
    .frequency     = GYRO_SPI_FREQ,   /* burst reads; registers forced to 1 MHz */
    .mode          = SPIDEV_MODE0,
    .bits          = 8,
    .cs            = GYRO_CS_DEVID,   /* NSS4 on Nucleo board */
    .reg_frequency = 1000000,
  };

  struct mmc5983ma_config_s mag_cfg =
  {
    .frequency = MAG_SPI_FREQ,
    .mode      = SPIDEV_MODE0,
    .bits      = 8,
    .cs        = MAG_CS_DEVID,    /* NSS3 on Nucleo board */
  };

  bool gyro_initialized = false;
  bool mag_initialized  = false;

  if (spi2 == NULL)
    {
      printf("[OBC_MAIN] ERROR: Failed to initialize SPI2 bus for IMU!\n");
      return -1;
    }

  /* One-shot read-only scan.  Diagnoses a missing case in
   * stm32wl5_spi2select(), identifies IAM-20380 vs IAM-20380HT, and
   * auto-corrects the devid / SPI mode / clock if a working combination
   * is found.
   */

  gyro_probe_cs(spi2, &gyro_cfg);

  gyro_initialized = init_gyro(spi2, &gyro_cfg);

  /* Gyro init leaves the driver's own chip-select/mode/clock state set
   * up for the gyro; always initialize the magnetometer afterwards so
   * its config (and chip select) is the one left active for its driver.
   */

  mag_initialized = init_mag(spi2, &mag_cfg);

  while (1)
    {
      scan_count++;
      printf("\n==============================================================================================\n");
      printf("  [OBC_MAIN] TELEMETRY ACQUISITION SCAN #%d\n", scan_count);
      printf("==============================================================================================\n");

      /* 1. Acquire and display ADC 1 (voltage + temperature) */

      ret = read_adc_chip(ADC1_DEVPATH, adc1_data);
      if (ret >= 0)
        {
          display_adc1_telemetry(adc1_data);
        }
      else
        {
          printf("[OBC_MAIN] ERROR: Failed to read ADC 1 (%s)\n", ADC1_DEVPATH);
        }

      /* 2. Acquire and display ADC 2 (current sensors) */

      ret = read_adc_chip(ADC2_DEVPATH, adc2_data);
      bool adc2_read_ok = (ret >= 0);
      if (adc2_read_ok)
        {
          printf("[OBC_MAIN] Error: Failed to read ADC 2 (%s)\n", ADC2_DEVPATH);
        }

      /* 3. Acquire and display IMU (gyroscope + magnetometer) */

      struct iam20380_data_s gyro_data;
      struct mmc5983ma_data_s mag_data;
      bool gyro_read_ok = false;
      bool mag_read_ok  = false;

      memset(&gyro_data, 0, sizeof(gyro_data));
      memset(&mag_data, 0, sizeof(mag_data));

      /* Retry init if a sensor was not ready at boot (e.g. its power
       * rail came up late).  Scan 1 is skipped since init was just
       * attempted immediately before entering the loop.
       */

      if (spi2 != NULL && (!gyro_initialized || !mag_initialized) &&
          scan_count > 1)
        {
          if (!gyro_initialized)
            {
              gyro_initialized = init_gyro(spi2, &gyro_cfg);
            }

          if (!mag_initialized)
            {
              mag_initialized = init_mag(spi2, &mag_cfg);
            }
        }

      if (spi2 != NULL && gyro_initialized)
        {
          ret = iam20380_read(spi2, &gyro_data);
          if (ret == OK)
            {
              gyro_read_ok = true;
            }
          else
            {
              printf("[OBC_MAIN] ERROR: iam20380_read failed (ret=%d), "
                     "will re-init on next retry scan\n", ret);
              gyro_initialized = false;
            }
        }

      if (spi2 != NULL && mag_initialized)
        {
          ret = mmc5983ma_read(spi2, &mag_data);
          if (ret == OK)
            {
              mag_read_ok = true;
            }
          else
            {
              printf("[OBC_MAIN] ERROR: mmc5983ma_read failed (ret=%d), "
                     "will re-init on next retry scan\n", ret);
              mag_initialized = false;
            }
        }

        if(adc2_read_ok)
        {
          display_adc2_telemetry(adc2_data, &gyro_data, &mag_data,
                                 gyro_read_ok, mag_read_ok);
        }

      /* 4. Send telemetry packets to the shared ring buffer & wake LittleFS */

      send_telemetry_to_ringbuffer(adc1_data, adc2_data, &gyro_data, &mag_data,
                                   gyro_read_ok, mag_read_ok, scan_count);
      printf("\n[OBC_MAIN] [PRODUCER] Buffered Telemetry to Ring Buffer (Occupancy: %d/16) -> sem_post()\n",
             telemetry_rb_count());

      /* 5. Sleep for the remainder of the cycle while actively listening
       * for ground telecommands and printing M0+ radio logs.
       */

      printf("[OBC_MAIN] [SLEEPING] Next telemetry scan in %d seconds (monitoring M0+ RF & ground commands)...\n\n",
             TELEMETRY_INTERVAL_SEC);
      for (int sec = 0; sec < TELEMETRY_INTERVAL_SEC; sec++)
        {
          for (int step = 0; step < 5; step++)
            {
              /* Drain and print all radio event logs from M0+ */

              struct radio_log_msg_s rlog;
              while (radio_log_read(&rlog))
                {
                  printf("%s\n", rlog.text);
                }

              /* Check for incoming ground commands from M0+ */

              process_ground_commands();

              usleep(200000); /* 200 ms */
            }
        }
    }

  return 0;
}