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

/* Architecture SPI initialization prototype */
FAR struct spi_dev_s *stm32wl5_spibus_initialize(int bus);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC1_DEVPATH        "/dev/adc0"
#define ADC2_DEVPATH        "/dev/adc1"
#define ADS7953_CHANNELS    16

/* ADS7953 full-scale: 12-bit (4095 counts) with 2X range (Vref = 2.5V -> 5.0V) */
#define ADC_VREF_FULLSCALE  5.000f
#define ADC_MAX_COUNTS      4095.0f

/* NTC Thermistor Parameters (10k NTC with 10k pull-up to 3.3V) */
#define NTC_R_PULLUP        10000.0f  /* 10k on-board pull-up */
#define NTC_V_SUPPLY        3.300f    /* 3.3V pull-up supply */
#define NTC_R0              10000.0f  /* 10k resistance at 25 deg C */
#define NTC_BETA            3950.0f   /* Beta coefficient (B25/85) */
#define NTC_T0              298.15f   /* 25 deg C in Kelvin */

/* Solar Panel & Sensor Detection Thresholds */
#define SOLAR_V_THRESHOLD   0.250f    /* Solar panel V < 0.25V -> Disconnected / Dark */
#define SOLAR_I_THRESHOLD   0.015f    /* Solar panel I < 15mA  -> Disconnected / No Current */
#define IDLE_I_THRESHOLD    0.010f    /* Current < 10mA       -> Idle / No Load */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

enum adc_channel_type_e
{
  CHANNEL_TYPE_VOLT = 0,   /* General Voltage monitor (V) */
  CHANNEL_TYPE_SOLAR_V,    /* Solar Panel Voltage (V) with disconnect detection */
  CHANNEL_TYPE_TEMP,       /* Temperature sensor (deg C) */
  CHANNEL_TYPE_CURR,       /* General Current sensor (A) */
  CHANNEL_TYPE_SOLAR_I,    /* Solar Panel Current (A) with disconnect detection */
  CHANNEL_TYPE_NC          /* Not Connected / Unused */
};

enum sensor_status_e
{
  STATUS_ACTIVE_OK = 0,
  STATUS_DISCONNECTED,
  STATUS_NO_DATA_DARK,
  STATUS_IDLE_NO_LOAD,
  STATUS_NC_UNUSED,
  STATUS_FAULT
};

struct adc_channel_cfg_s
{
  const char              *name;        /* Signal name from schematic */
  enum adc_channel_type_e  type;        /* Channel type */
  const char              *unit;        /* Engineering unit string */
  float                    slope;       /* Multiplier (m) / Scaling / Divider Ratio */
  float                    offset;      /* Offset correction (c) */
  bool                     is_nc;       /* true if channel is unconnected */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC1 Configuration:
 *   CH00 - CH07: Voltage Monitors (V)
 *   CH08 - CH15: Temperature Sensors (deg C)
 */
static const struct adc_channel_cfg_s g_adc1_cfg[ADS7953_CHANNELS] =
{
  /* CH00: ADC_BAT_MON (Battery Voltage Monitor) */
  { "ADC_BAT_MON",    CHANNEL_TYPE_VOLT,    "V",  1.0f,    0.0f,  false },

  /* CH01: TOTAL_SOLAR_V (Total Solar Voltage) */
  { "TOTAL_SOLAR_V",  CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },

  /* CH02: RAW_VOLT (Raw Bus Voltage) */
  { "RAW_VOLT",       CHANNEL_TYPE_VOLT,    "V",  1.0f,    0.0f,  false },

  /* CH03: SP5_VOLT (Solar Panel 5 Voltage) */
  { "SP5_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },

  /* CH04: SP4_VOLT (Solar Panel 4 Voltage) */
  { "SP4_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },

  /* CH05: SP3_VOLT (Solar Panel 3 Voltage) */
  { "SP3_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },

  /* CH06: SP1_VOLT (Solar Panel 1 Voltage) */
  { "SP1_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },

  /* CH07: SP2_VOLT (Solar Panel 2 Voltage) */
  { "SP2_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f,    0.0f,  false },

  /* CH08: ANT_TEMP (Antenna Temperature) - NTC Thermistor */
  { "ANT_TEMP",       CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },

  /* CH09: BATT_TEMP (Battery Temperature Sense) - NTC Thermistor */
  { "BATT_TEMP",      CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },

  /* CH10: TEMP_BPB (BPB Temperature) - NTC Thermistor */
  { "TEMP_BPB",       CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },

  /* CH11: TEMP1 (Temperature 1) - NTC Thermistor */
  { "TEMP1",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },

  /* CH12: TEMP5 (Temperature 5) - NTC Thermistor */
  { "TEMP5",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },

  /* CH13: TEMP4 (Temperature 4) - NTC Thermistor */
  { "TEMP4",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },

  /* CH14: TEMP3 (Temperature 3) - NTC Thermistor */
  { "TEMP3",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },

  /* CH15: TEMP2 (Temperature 2) - NTC Thermistor */
  { "TEMP2",          CHANNEL_TYPE_TEMP,    "C",  1.0f,    0.0f,  false },
};

/* ADC2 Configuration:
 *   Current Sensors and NC Pins
 */
static const struct adc_channel_cfg_s g_adc2_cfg[ADS7953_CHANNELS] =
{
  /* CH00: NC (Not Connected) */
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },

  /* CH01: UNREG_I (Unregulated Bus Current) */
  { "UNREG_I",        CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },

  /* CH02: SP4_I (Solar Panel 4 Current) */
  { "SP4_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },

  /* CH03: MAIN_3V3_I (Main 3.3V Current) */
  { "MAIN_3V3_I",     CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },

  /* CH04: NC (Not Connected) */
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },

  /* CH05: MISSION_3V3_I (Mission 3.3V Current) */
  { "MISSION_3V3_I",  CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },

  /* CH06: SPT_I (SPT Current) */
  { "SPT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },

  /* CH07: NC (Not Connected) */
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },

  /* CH08: 5V_I (5V Rail Current) */
  { "5V_I",           CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },

  /* CH09: SP2_I (Solar Panel 2 Current) */
  { "SP2_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },

  /* CH10: SP1_I (Solar Panel 1 Current) */
  { "SP1_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },

  /* CH11: RAW_I (Raw Bus Current) */
  { "RAW_I",          CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },

  /* CH12: NC (Not Connected) */
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f,   0.0f,  true  },

  /* CH13: SP5_I (Solar Panel 5 Current) */
  { "SP5_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },

  /* CH14: BAT_I (Battery Current) */
  { "BAT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f,   0.0f,  false },

  /* CH15: SP3_I (Solar Panel 3 Current) */
  { "SP3_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f,   0.0f,  false },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read_adc_chip
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

  /* Read 16 samples from driver FIFO */

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

  /* Calculate input pin voltage (0.000V to 5.000V) */

  v_pin = ((float)raw_counts / ADC_MAX_COUNTS) * ADC_VREF_FULLSCALE;

  if (v_pin_out)
    {
      *v_pin_out = v_pin;
    }

  /* 1. NTC Temperature Sensor Handling */

  if (cfg->type == CHANNEL_TYPE_TEMP)
    {
      if (raw_counts >= 4000 || v_pin >= (NTC_V_SUPPLY - 0.05f))
        {
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

  /* 2. Solar Panel Voltage Handling */

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

  /* 3. Solar Panel Current Handling */

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

  /* 4. General Current Rail Handling */

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

  /* 5. General Voltage Rail Handling */

  if (status_out)
    {
      *status_out = STATUS_ACTIVE_OK;
    }

  return (v_pin * cfg->slope) + cfg->offset;
}

/****************************************************************************
 * Name: send_telemetry_to_ringbuffer
 ****************************************************************************/

static int send_telemetry_to_ringbuffer(uint16_t adc0_raw[ADS7953_CHANNELS],
                                        uint16_t adc1_raw[ADS7953_CHANNELS],
                                        const struct iam20380_data_s *gyro,
                                        const struct mmc5983ma_data_s *mag,
                                        bool gyro_ok,
                                        bool mag_ok)
{
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
  /* ========================================================================= */

  memset(&env_adc1, 0, sizeof(env_adc1));
  env_adc1.len = sizeof(struct adc1_packet_s);
  env_adc1.footer = TELEM_FOOTER_ADC1;

  for (ch = 0; ch < ADC1_CHANNELS_COUNT; ch++)
    {
      float val = convert_channel(adc0_raw[ch], &g_adc1_cfg[ch], &v_pin, &status);
      env_adc1.pkt.adc1.data[ch] = (int16_t)roundf(val * 100.0f);
    }

  env_adc1.pkt.adc1.footer = TELEM_FOOTER_ADC1;

  /* Push ADC 1 packet to Ring Buffer & signal LittleFS */
  telemetry_rb_write(&env_adc1);
  sem_post(&g_telemetry_sem);

  /* ========================================================================= */
  /* 2. Package ADC 2 + IMU: 12 Currents + 3 Gyro + 3 Mag + Footer2 = 38 Bytes */
  /* ========================================================================= */

  memset(&env_adc2, 0, sizeof(env_adc2));
  env_adc2.len = sizeof(struct adc2_packet_s);
  env_adc2.footer = TELEM_FOOTER_ADC2;

  /* 12 Active Current Channels scaled x100 */
  for (i = 0; i < ADC2_ACTIVE_CHANNELS; i++)
    {
      ch = adc2_active_map[i];
      float val = convert_channel(adc1_raw[ch], &g_adc2_cfg[ch], &v_pin, &status);
      env_adc2.pkt.adc2.data[i] = (int16_t)roundf(val * 100.0f);
    }

  /* 3-Axis Gyroscope (IAM-20380) in deg/s scaled x100 */
  if (gyro_ok && gyro != NULL)
    {
      float dps_x = (float)gyro->x / 131.0f;
      float dps_y = (float)gyro->y / 131.0f;
      float dps_z = (float)gyro->z / 131.0f;

      env_adc2.pkt.adc2.gyro_x = (int16_t)roundf(dps_x * 100.0f);
      env_adc2.pkt.adc2.gyro_y = (int16_t)roundf(dps_y * 100.0f);
      env_adc2.pkt.adc2.gyro_z = (int16_t)roundf(dps_z * 100.0f);
    }
  else
    {
      env_adc2.pkt.adc2.gyro_x = 0;
      env_adc2.pkt.adc2.gyro_y = 0;
      env_adc2.pkt.adc2.gyro_z = 0;
    }

  /* 3-Axis Magnetometer (MMC5983MA) in Gauss scaled x100 */
  if (mag_ok && mag != NULL)
    {
      float g_x = (float)mag->x / 16384.0f;
      float g_y = (float)mag->y / 16384.0f;
      float g_z = (float)mag->z / 16384.0f;

      env_adc2.pkt.adc2.mag_x = (int16_t)roundf(g_x * 100.0f);
      env_adc2.pkt.adc2.mag_y = (int16_t)roundf(g_y * 100.0f);
      env_adc2.pkt.adc2.mag_z = (int16_t)roundf(g_z * 100.0f);
    }
  else
    {
      env_adc2.pkt.adc2.mag_x = 0;
      env_adc2.pkt.adc2.mag_y = 0;
      env_adc2.pkt.adc2.mag_z = 0;
    }

  env_adc2.pkt.adc2.footer = TELEM_FOOTER_ADC2;

  /* Push ADC 2 + IMU packet to Ring Buffer & signal LittleFS */
  telemetry_rb_write(&env_adc2);
  sem_post(&g_telemetry_sem);

  /* ========================================================================= */
  /* 3. Dual-Core IPC to Cortex-M0+ (Radio Subsystem in Shared SRAM2)          */
  /* ========================================================================= */

  /* A. Extract Float Battery Voltage & Satellite Temp for CW Morse beacon */
  float v_batt_pin;
  enum sensor_status_e batt_status;
  float batt_v = convert_channel(adc0_raw[0], &g_adc1_cfg[0], &v_batt_pin, &batt_status);

  float v_temp_pin;
  enum sensor_status_e temp_status;
  float temp_c = convert_channel(adc0_raw[8], &g_adc1_cfg[8], &v_temp_pin, &temp_status);

  live_telem_update_float(batt_v, temp_c);

  /* B. Queue Beacon 1 (HK1 34B) into Shared SRAM2 Ring Buffer 1 */
  struct tx_packet_s tx_b1;
  tx_b1.type = PKT_TYPE_B1;
  tx_b1.len  = sizeof(struct adc1_packet_s);
  memcpy(tx_b1.data, &env_adc1.pkt.adc1, tx_b1.len);
  rb_tx_write(&tx_b1);

  /* C. Queue Beacon 2 (HK2 38B) into Shared SRAM2 Ring Buffer 1 */
  struct tx_packet_s tx_b2;
  tx_b2.type = PKT_TYPE_B2;
  tx_b2.len  = sizeof(struct adc2_packet_s);
  memcpy(tx_b2.data, &env_adc2.pkt.adc2, tx_b2.len);
  rb_tx_write(&tx_b2);

  /* D. Fire IPCC Channel 1 Doorbell to Cortex-M0+ */
  ipcc_m4_send(IPCC_CH_BEACON);

  int v_int  = (int)batt_v;
  int v_frac = (int)((batt_v - (float)v_int) * 100.0f);
  if (v_frac < 0) v_frac = -v_frac;
  int t_int  = (int)temp_c;
  int t_frac = (int)((temp_c - (float)t_int) * 10.0f);
  if (t_frac < 0) t_frac = -t_frac;

  printf("\n[OBC_MAIN] [TX -> M0+] CW Telemetry: \"9NS2S2 V%d.%02d T%d.%d\" | Pushed B1 (34B) & B2 (38B) [Sent once for 90s cycle]\n",
         v_int, v_frac, t_int, t_frac);

  return 0;
}

/****************************************************************************
 * Name: check_camera_data_available
 * Description: Checks if /mnt/camera has available image files.
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
 * Description: Checks for incoming ground telecommands from M0+ in SRAM2.
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

      /* Check if Camera Command: 53 04 CC 5E BD 00 00... */
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
          /* Ensure /mnt/camera partition directory exists and create test frame */
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

          struct tx_packet_s resp;
          memset(&resp, 0, sizeof(resp));
          resp.len = CMD_PAYLOAD_LEN;
          memcpy(resp.data, rx_cmd.cmd, CMD_PAYLOAD_LEN);

          if (data_ok)
            {
              printf("[OBC_MAIN] [CAMERA DATA AVAILABLE] Sending ACK (0xAA) to M0+...\n");
              resp.type = PKT_TYPE_ACK;
              resp.data[1] = 0xAA; /* ACK */
            }
          else
            {
              printf("[OBC_MAIN] [CAMERA DATA NOT AVAILABLE] Sending NACK (0xFF) to M0+...\n");
              resp.type = PKT_TYPE_NACK;
              resp.data[1] = 0xFF; /* NACK */
            }

          rb_tx_write(&resp);
          ipcc_m4_send(IPCC_CH_BEACON);
        }
      else if (rx_cmd.cmd[0] == 0x53 && rx_cmd.cmd[1] == CMD_TYPE_HK)
        {
          printf("[OBC_MAIN] Detected HK Request Command (53 01...)\n");
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

      if (cfg->type == CHANNEL_TYPE_TEMP)
        {
          type_str = "Temperature";
        }
      else
        {
          type_str = "Voltage";
        }

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
 ****************************************************************************/

static void display_adc2_telemetry(uint16_t raw_data[ADS7953_CHANNELS])
{
  int ch;

  printf("\n=================================== ADC 2: CURRENT SENSORS ===================================\n");
  printf("  CH  | Signal Name       | Type        | Raw   | Pin Volt | Current Value    | Status        \n");
  printf("------+-------------------+-------------+-------+----------+------------------+---------------\n");

  for (ch = 0; ch < ADS7953_CHANNELS; ch++)
    {
      const struct adc_channel_cfg_s *cfg = &g_adc2_cfg[ch];
      float v_pin;
      float phys_val;
      enum sensor_status_e status;
      const char *status_str;

      if (cfg->is_nc)
        {
          /* Skip NC (Not Connected) channels without printing */
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

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: display_imu_telemetry
 ****************************************************************************/

static void display_imu_telemetry(const struct iam20380_data_s *gyro,
                                  const struct mmc5983ma_data_s *mag,
                                  bool gyro_ok, bool mag_ok)
{
  printf("\n=================================== IMU & MAGNETOMETER =======================================\n");
  printf("  Sensor       | Axis | Raw Count | Physical Value       | Status        \n");
  printf("---------------+------+-----------+----------------------+---------------\n");

  if (gyro_ok)
    {
      float dps_x = (float)gyro->x / 131.0f;
      float dps_y = (float)gyro->y / 131.0f;
      float dps_z = (float)gyro->z / 131.0f;
      float temp  = ((float)gyro->temp / 326.8f) + 25.0f;

      printf("  IAM-20380    | X    | %9d | %9.2f deg/s       | ACTIVE (OK)   \n", gyro->x, dps_x);
      printf("  (Gyroscope)  | Y    | %9d | %9.2f deg/s       | ACTIVE (OK)   \n", gyro->y, dps_y);
      printf("               | Z    | %9d | %9.2f deg/s       | ACTIVE (OK)   \n", gyro->z, dps_z);
      printf("               | Temp | %9d | %9.2f deg C       | ACTIVE (OK)   \n", gyro->temp, temp);
    }
  else
    {
      printf("  IAM-20380    | ---  |       --- |         ---          | [ DISCONNECTED ]\n");
    }

  printf("---------------+------+-----------+----------------------+---------------\n");

  if (mag_ok)
    {
      float g_x = (float)mag->x / 16384.0f;
      float g_y = (float)mag->y / 16384.0f;
      float g_z = (float)mag->z / 16384.0f;

      printf("  MMC5983MA    | X    | %9ld | %9.3f Gauss       | ACTIVE (OK)   \n", (long)mag->x, g_x);
      printf("  (Magneto)    | Y    | %9ld | %9.3f Gauss       | ACTIVE (OK)   \n", (long)mag->y, g_y);
      printf("               | Z    | %9ld | %9.3f Gauss       | ACTIVE (OK)   \n", (long)mag->z, g_z);
    }
  else
    {
      printf("  MMC5983MA    | ---  |       --- |         ---          | [ DISCONNECTED ]\n");
    }

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obc_main_main
 ****************************************************************************/

#define TELEMETRY_INTERVAL_SEC  90

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
  printf("  IMU (SPI2): IAM-20380 (Gyro, NSS4) | MMC5983MA (Mag, NSS3) \n");
  printf("  Acquisition Cadence: Every %d Seconds                       \n", TELEMETRY_INTERVAL_SEC);
  printf("=============================================================\n");

  /* Ensure Local Ring Buffer & Notification Semaphore are ready */
  telemetry_rb_init();

  /* Initialize Dual-Core Shared SRAM2 Ring Buffers & Live Telemetry */
  rb_ipc_init();

  /* Initialize SPI2 for Gyroscope and Magnetometer */
  FAR struct spi_dev_s *spi2 = stm32wl5_spibus_initialize(2);
  struct iam20380_config_s gyro_cfg =
  {
    .frequency = 10000000,
    .mode      = SPIDEV_MODE0,
    .bits      = 8,
    .cs        = SPIDEV_USER(3), /* NSS4 on Nucleo board */
  };

  struct mmc5983ma_config_s mag_cfg =
  {
    .frequency = 10000000,
    .mode      = SPIDEV_MODE0,
    .bits      = 8,
    .cs        = SPIDEV_USER(2), /* NSS3 on Nucleo board */
  };

  bool gyro_initialized = false;
  bool mag_initialized  = false;

  if (spi2)
    {
      if (iam20380_initialize(spi2, &gyro_cfg) == OK)
        {
          gyro_initialized = true;
          printf("[OBC_MAIN] IAM-20380 Gyroscope initialized successfully!\n");
        }
      else
        {
          printf("[OBC_MAIN] WARNING: IAM-20380 Gyroscope not detected.\n");
        }

      if (mmc5983ma_initialize(spi2, &mag_cfg) == OK)
        {
          mag_initialized = true;
          printf("[OBC_MAIN] MMC5983MA Magnetometer initialized successfully!\n");
        }
      else
        {
          printf("[OBC_MAIN] WARNING: MMC5983MA Magnetometer not detected.\n");
        }
    }
  else
    {
      printf("[OBC_MAIN] ERROR: Failed to initialize SPI2 bus for IMU!\n");
    }

  while (1)
    {
      scan_count++;
      printf("\n==============================================================================================\n");
      printf("  [OBC_MAIN] TELEMETRY ACQUISITION SCAN #%d\n", scan_count);
      printf("==============================================================================================\n");

      /* 1. Acquire and display ADC 1 (Voltage + Temperature) */
      ret = read_adc_chip(ADC1_DEVPATH, adc1_data);
      if (ret >= 0)
        {
          display_adc1_telemetry(adc1_data);
        }
      else
        {
          printf("[OBC_MAIN] ERROR: Failed to read ADC 1 (%s)\n", ADC1_DEVPATH);
        }

      /* 2. Acquire and display ADC 2 (Current Sensors) */
      ret = read_adc_chip(ADC2_DEVPATH, adc2_data);
      if (ret >= 0)
        {
          display_adc2_telemetry(adc2_data);
        }
      else
        {
          printf("[OBC_MAIN] ERROR: Failed to read ADC 2 (%s)\n", ADC2_DEVPATH);
        }

      /* 3. Acquire and display IMU (Gyroscope + Magnetometer) */
      struct iam20380_data_s gyro_data;
      struct mmc5983ma_data_s mag_data;
      bool gyro_read_ok = false;
      bool mag_read_ok  = false;

      memset(&gyro_data, 0, sizeof(gyro_data));
      memset(&mag_data, 0, sizeof(mag_data));

      if (spi2 && gyro_initialized && iam20380_read(spi2, &gyro_data) == OK)
        {
          gyro_read_ok = true;
        }

      if (spi2 && mag_initialized && mmc5983ma_read(spi2, &mag_data) == OK)
        {
          mag_read_ok = true;
        }

      display_imu_telemetry(&gyro_data, &mag_data, gyro_read_ok, mag_read_ok);

      /* 4. Send telemetry packets to Shared Ring Buffer & wake LittleFS */
      send_telemetry_to_ringbuffer(adc1_data, adc2_data, &gyro_data, &mag_data,
                                   gyro_read_ok, mag_read_ok);
      printf("\n[OBC_MAIN] [PRODUCER] Buffered Telemetry to Ring Buffer (Occupancy: %d/16) -> sem_post()\n",
             telemetry_rb_count());

      /* 5. Sleep 90 seconds while actively listening for ground telecommands & printing M0+ radio logs */
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

              /* Check incoming ground commands from M0+ */
              process_ground_commands();

              usleep(200000); /* 200 ms */
            }
        }
    }

  return 0;
}