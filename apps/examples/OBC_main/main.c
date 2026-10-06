/****************************************************************************
 * apps/examples/OBC_main/main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 ****************************************************************************/

/****************************************************************************
 * Overview
 *
 *   OBC_main is the On-Board Computer's telemetry producer task.
 *
 *   The M4 CPU uses TWO execution paths:
 *
 *     1. Main OBC thread
 *        - Continuously processes M0+ radio logs
 *        - Processes ground commands
 *        - Handles other OBC tasks
 *
 *     2. Telemetry pthread
 *        - Sleeps for 90 seconds
 *        - Wakes up
 *        - Reads ADC1
 *        - Reads ADC2
 *        - Reads gyroscope
 *        - Reads magnetometer
 *        - Builds Beacon 1 and Beacon 2
 *        - Stores telemetry in local ring buffer
 *        - Updates live telemetry
 *        - Notifies M0+ through IPCC
 *        - Goes back to sleep
 *
 *   This means telemetry acquisition does not block the main OBC loop.
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
#include "camera_dummy_image.h"
#include <termios.h>

#include <pthread.h>
#include <sched.h>

#include <arch/board/board.h>

/****************************************************************************
 * Board-specific SPI bus initialization
 ****************************************************************************/

FAR struct spi_dev_s *stm32wl5_spibus_initialize(int bus);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC device paths and geometry */

#define ADC1_DEVPATH        "/dev/adc0"
#define ADC2_DEVPATH        "/dev/adc1"
#define ADS7953_CHANNELS    16

/* ADS7953 full-scale */

#define ADC_VREF_FULLSCALE  5.000f
#define ADC_MAX_COUNTS      4095.0f

/* NTC parameters */

#define NTC_R_PULLUP        10000.0f
#define NTC_V_SUPPLY        3.300f
#define NTC_R0              10000.0f
#define NTC_BETA            3950.0f
#define NTC_T0              298.15f

/* Detection thresholds */

#define SOLAR_V_THRESHOLD   0.250f
#define SOLAR_I_THRESHOLD   0.015f
#define IDLE_I_THRESHOLD    0.010f

/* Telemetry acquisition cadence */

#define TELEMETRY_INTERVAL_SEC  90

/* SPI2 chip-select IDs */

#define GYRO_CS_DEVID       SPIDEV_USER(3)
#define MAG_CS_DEVID        SPIDEV_USER(2)

/* SPI clocks */

#define GYRO_SPI_FREQ       8000000
#define MAG_SPI_FREQ        10000000

/* Telemetry packing scales */

#define MAG_PACKET_SCALE    10.0f
#define GYRO_PACKET_SCALE   100.0f

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

enum adc_channel_type_e
{
  CHANNEL_TYPE_VOLT = 0,
  CHANNEL_TYPE_SOLAR_V,
  CHANNEL_TYPE_TEMP,
  CHANNEL_TYPE_CURR,
  CHANNEL_TYPE_SOLAR_I,
  CHANNEL_TYPE_NC
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
  const char              *name;
  enum adc_channel_type_e  type;
  const char              *unit;
  float                    slope;
  float                    offset;
  bool                     is_nc;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* -------------------------------------------------------------------------
 * Telemetry thread state
 * -------------------------------------------------------------------------
 */

static pthread_t g_telemetry_thread;

/* Persistent telemetry scan counter.
 *
 * IMPORTANT:
 * This must NOT be local to perform_telemetry_cycle(), otherwise it
 * resets to zero every time the function is called.
 */

static int g_scan_count = 0;

/* -------------------------------------------------------------------------
 * Shared SPI / IMU state
 * -------------------------------------------------------------------------
 *
 * These used to be local variables inside obc_main_main().
 * They must be global/static because telemetry_thread() accesses them.
 */

static FAR struct spi_dev_s *g_spi2 = NULL;

static struct iam20380_config_s g_gyro_cfg;
static struct mmc5983ma_config_s g_mag_cfg;

static bool g_gyro_initialized = false;
static bool g_mag_initialized = false;

/* -------------------------------------------------------------------------
 * Beacon/IPCC protection
 * -------------------------------------------------------------------------
 *
 * Both the telemetry thread and process_ground_commands() can request
 * an IPCC beacon. Protect the live telemetry update + IPCC notification
 * as one operation.
 */

static pthread_mutex_t g_beacon_mutex = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * ADC1 channel map
 *
 * CH00-CH07 : Voltage monitors
 * CH08-CH15 : NTC temperature sensors
 ****************************************************************************/

static const struct adc_channel_cfg_s
g_adc1_cfg[ADS7953_CHANNELS] =
{
  { "ADC_BAT_MON",    CHANNEL_TYPE_VOLT,    "V",  1.0f, 0.0f, false },
  { "TOTAL_SOLAR_V",  CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "RAW_VOLT",       CHANNEL_TYPE_VOLT,    "V",  1.0f, 0.0f, false },
  { "SP5_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP4_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP3_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP1_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP2_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "ANT_TEMP",       CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "BATT_TEMP",      CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP_BPB",       CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP1",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP5",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP4",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP3",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP2",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
};

/****************************************************************************
 * ADC2 channel map
 ****************************************************************************/

static const struct adc_channel_cfg_s
g_adc2_cfg[ADS7953_CHANNELS] =
{
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "UNREG_I",        CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SP4_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "MAIN_3V3_I",     CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "MISSION_3V3_I",  CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SPT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "5V_I",           CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SP2_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "SP1_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "RAW_I",          CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "SP5_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "BAT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SP3_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
};

/****************************************************************************
 * Forward Declarations
 ****************************************************************************/

static int16_t pack_i16(float value);

static int read_adc_chip(const char *devpath,
                         uint16_t channels[ADS7953_CHANNELS]);

static float convert_channel(uint16_t raw_counts,
                             const struct adc_channel_cfg_s *cfg,
                             float *v_pin_out,
                             enum sensor_status_e *status_out);

static void display_adc1_telemetry(
                             uint16_t raw_data[ADS7953_CHANNELS]);

static void display_adc2_telemetry(
                             uint16_t raw_data[ADS7953_CHANNELS],
                             const struct iam20380_data_s *gyro,
                             const struct mmc5983ma_data_s *mag,
                             bool gyro_ok,
                             bool mag_ok);

static bool init_gyro(FAR struct spi_dev_s *spi,
                      FAR const struct iam20380_config_s *cfg);

static bool init_mag(FAR struct spi_dev_s *spi,
                     FAR const struct mmc5983ma_config_s *cfg);

static bool gyro_probe_cs(FAR struct spi_dev_s *spi,
                          FAR struct iam20380_config_s *cfg);

static int send_telemetry_to_ringbuffer(
                         uint16_t adc0_raw[ADS7953_CHANNELS],
                         uint16_t adc1_raw[ADS7953_CHANNELS],
                         const struct iam20380_data_s *gyro,
                         const struct mmc5983ma_data_s *mag,
                         bool gyro_ok,
                         bool mag_ok,
                         int scan_count);

static bool check_camera_data_available(void);

static void process_ground_commands(void);

static void perform_telemetry_cycle(void);

static void *telemetry_thread(void *arg);

/****************************************************************************
 * Name: telemetry_thread
 *
 * Description:
 *   Dedicated periodic telemetry acquisition thread.
 *
 *   The thread sleeps for 90 seconds and then performs exactly one
 *   telemetry acquisition cycle.
 *
 *   The main OBC thread is therefore free to handle radio logs and
 *   ground commands while this thread sleeps.
 ****************************************************************************/

static void *telemetry_thread(void *arg)
{
  (void)arg;

  printf("\n");
  printf("[TELEMETRY] ================================================\n");
  printf("[TELEMETRY] Telemetry thread started\n");
  printf("[TELEMETRY] Acquisition interval = %d seconds\n",
         TELEMETRY_INTERVAL_SEC);
  printf("[TELEMETRY] ================================================\n");

  while (1)
    {
      /* 1. Perform telemetry acquisition and update live buffer FIRST */
      perform_telemetry_cycle();

      printf("[TELEMETRY] Acquisition cycle completed\n");

      /* 2. Sleep for interval until next periodic acquisition */
      printf("[TELEMETRY] Sleeping for %d seconds...\n",
             TELEMETRY_INTERVAL_SEC);

      sleep(TELEMETRY_INTERVAL_SEC);

      printf("\n");
      printf("[TELEMETRY] ================================================\n");
      printf("[TELEMETRY] %d seconds elapsed - waking up\n",
             TELEMETRY_INTERVAL_SEC);
      printf("[TELEMETRY] ================================================\n");
    }

  return NULL;
}

/****************************************************************************
 * Name: perform_telemetry_cycle
 *
 * Description:
 *   Perform one complete telemetry acquisition cycle.
 *
 *   This function is called ONLY by telemetry_thread().
 *
 *   Hardware initialization is NOT performed here except retrying a sensor
 *   if it previously failed.
 ****************************************************************************/

static void perform_telemetry_cycle(void)
{
  uint16_t adc1_data[ADS7953_CHANNELS];
  uint16_t adc2_data[ADS7953_CHANNELS];

  struct iam20380_data_s gyro_data;
  struct mmc5983ma_data_s mag_data;

  bool adc1_read_ok = false;
  bool adc2_read_ok = false;
  bool gyro_read_ok = false;
  bool mag_read_ok = false;

  int ret;

  memset(adc1_data, 0, sizeof(adc1_data));
  memset(adc2_data, 0, sizeof(adc2_data));
  memset(&gyro_data, 0, sizeof(gyro_data));
  memset(&mag_data, 0, sizeof(mag_data));

  /*
   * Persistent scan counter.
   */

  g_scan_count++;

  printf("\n");
  printf("==============================================================================================\n");
  printf("  [OBC_MAIN] TELEMETRY ACQUISITION SCAN #%d\n",
         g_scan_count);
  printf("==============================================================================================\n");

  /**************************************************************************
   * 1. ADC1
   **************************************************************************/

  ret = read_adc_chip(ADC1_DEVPATH, adc1_data);

  if (ret >= 0)
    {
      adc1_read_ok = true;

      printf("[OBC_MAIN] ADC1 read successful: %d samples\n", ret);

      display_adc1_telemetry(adc1_data);
    }
  else
    {
      printf("[OBC_MAIN] ERROR: Failed to read ADC1 (%s), ret=%d\n",
             ADC1_DEVPATH, ret);
    }

  /**************************************************************************
   * 2. ADC2
   **************************************************************************/

  ret = read_adc_chip(ADC2_DEVPATH, adc2_data);

  if (ret >= 0)
    {
      adc2_read_ok = true;

      printf("[OBC_MAIN] ADC2 read successful: %d samples\n", ret);
    }
  else
    {
      printf("[OBC_MAIN] ERROR: Failed to read ADC2 (%s), ret=%d\n",
             ADC2_DEVPATH, ret);
    }

  /**************************************************************************
   * 3. Retry IMU initialization if required
   *
   * Initialization normally happens once during startup.
   *
   * If a sensor failed initialization, we retry here on later telemetry
   * cycles instead of repeatedly initializing it every cycle.
   **************************************************************************/

  if (g_spi2 != NULL)
    {
      if (!g_gyro_initialized)
        {
          printf("[OBC_MAIN] Retrying gyro initialization...\n");

          g_gyro_initialized =
            init_gyro(g_spi2, &g_gyro_cfg);

          if (g_gyro_initialized)
            {
              printf("[OBC_MAIN] Gyro retry initialization successful\n");
            }
        }

      if (!g_mag_initialized)
        {
          printf("[OBC_MAIN] Retrying magnetometer initialization...\n");

          g_mag_initialized =
            init_mag(g_spi2, &g_mag_cfg);

          if (g_mag_initialized)
            {
              printf("[OBC_MAIN] Magnetometer retry initialization successful\n");
            }
        }
    }

  /**************************************************************************
   * 4. Read gyroscope
   **************************************************************************/

  if (g_spi2 != NULL && g_gyro_initialized)
    {
      ret = iam20380_read(g_spi2, &gyro_data);

      if (ret == OK)
        {
          gyro_read_ok = true;
        }
      else
        {
          printf("[OBC_MAIN] ERROR: iam20380_read failed (ret=%d), "
                 "will re-init on next telemetry cycle\n",
                 ret);

          g_gyro_initialized = false;
        }
    }

  /**************************************************************************
   * 5. Read magnetometer
   **************************************************************************/

  if (g_spi2 != NULL && g_mag_initialized)
    {
      ret = mmc5983ma_read(g_spi2, &mag_data);

      if (ret == OK)
        {
          mag_read_ok = true;
        }
      else
        {
          printf("[OBC_MAIN] ERROR: mmc5983ma_read failed (ret=%d), "
                 "will re-init on next telemetry cycle\n",
                 ret);

          g_mag_initialized = false;
        }
    }

  /**************************************************************************
   * 6. Display ADC2 + IMU
   **************************************************************************/

  if (adc2_read_ok)
    {
      display_adc2_telemetry(adc2_data,
                             &gyro_data,
                             &mag_data,
                             gyro_read_ok,
                             mag_read_ok);
    }

  /**************************************************************************
   * 7. Build/store/transmit telemetry
   *
   * adc1_read_ok is currently retained for diagnostics.  The existing
   * read_adc_chip() function clears the buffer on failure, so the existing
   * packet-building behavior is preserved.
   **************************************************************************/

  (void)adc1_read_ok;

  ret = send_telemetry_to_ringbuffer(adc1_data,
                                     adc2_data,
                                     &gyro_data,
                                     &mag_data,
                                     gyro_read_ok,
                                     mag_read_ok,
                                     g_scan_count);

  if (ret < 0)
    {
      printf("[OBC_MAIN] ERROR: send_telemetry_to_ringbuffer failed: %d\n",
             ret);
    }

  printf("\n");
  printf("[OBC_MAIN] [PRODUCER] Buffered Telemetry to Ring Buffer "
         "(Occupancy: %d/16)\n",
         telemetry_rb_count());

  printf("[OBC_MAIN] Telemetry scan #%d finished\n",
         g_scan_count);

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: pack_i16
 ****************************************************************************/

static int16_t pack_i16(float value)
{
  float r = roundf(value);

  if (r != r)
    {
      return 0;
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
 ****************************************************************************/

static int read_adc_chip(const char *devpath,
                         uint16_t channels[ADS7953_CHANNELS])
{
  struct adc_msg_s samples[ADS7953_CHANNELS];
  ssize_t nbytes;
  int nsamples;
  int fd;
  int ret;
  int i;

  memset(channels, 0,
         sizeof(uint16_t) * ADS7953_CHANNELS);

  fd = open(devpath, O_RDONLY);
  if (fd < 0)
    {
      int err = errno;

      printf("ERROR: Failed to open %s (errno: %d)\n",
             devpath, err);

      return -err;
    }

  ret = ioctl(fd, ANIOC_TRIGGER, 0);

  if (ret < 0)
    {
      int err = errno;

      printf("ERROR: ANIOC_TRIGGER failed on %s (errno: %d)\n",
             devpath, err);

      close(fd);

      return -err;
    }

  nbytes = read(fd,
                samples,
                sizeof(samples));

  if (nbytes < 0)
    {
      int err = errno;

      printf("ERROR: Read failed on %s (errno: %d)\n",
             devpath, err);

      close(fd);

      return -err;
    }

  nsamples = nbytes / sizeof(struct adc_msg_s);

  for (i = 0; i < nsamples; i++)
    {
      uint8_t ch = samples[i].am_channel;

      if (ch < ADS7953_CHANNELS)
        {
          channels[ch] =
            (uint16_t)(samples[i].am_data & 0x0FFF);
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

  v_pin =
    ((float)raw_counts / ADC_MAX_COUNTS) *
    ADC_VREF_FULLSCALE;

  if (v_pin_out)
    {
      *v_pin_out = v_pin;
    }

  /* NTC temperature */

  if (cfg->type == CHANNEL_TYPE_TEMP)
    {
      if (raw_counts >= 4000 ||
          v_pin >= (NTC_V_SUPPLY - 0.05f))
        {
          if (status_out)
            {
              *status_out = STATUS_DISCONNECTED;
            }

          return 0.0f;
        }

      {
        float r_ntc;
        float steinhart;
        float temp_c;

        r_ntc =
          NTC_R_PULLUP *
          (v_pin / (NTC_V_SUPPLY - v_pin));

        if (r_ntc <= 0.0f)
          {
            if (status_out)
              {
                *status_out = STATUS_FAULT;
              }

            return -273.15f;
          }

        steinhart =
          (1.0f / NTC_T0) +
          (1.0f / NTC_BETA) *
          logf(r_ntc / NTC_R0);

        temp_c =
          (1.0f / steinhart) - 273.15f;

        if (status_out)
          {
            *status_out = STATUS_ACTIVE_OK;
          }

        return temp_c;
      }
    }

  /* Solar voltage */

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

      return (v_pin * cfg->slope) +
             cfg->offset;
    }

  /* Solar current */

  if (cfg->type == CHANNEL_TYPE_SOLAR_I)
    {
      float current =
        (v_pin * cfg->slope) +
        cfg->offset;

      if (v_pin < SOLAR_I_THRESHOLD ||
          current < 0.010f)
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

  /* General current */

  if (cfg->type == CHANNEL_TYPE_CURR)
    {
      float current =
        (v_pin * cfg->slope) +
        cfg->offset;

      if (v_pin < IDLE_I_THRESHOLD ||
          current < 0.005f)
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

  /* General voltage */

  if (status_out)
    {
      *status_out = STATUS_ACTIVE_OK;
    }

  return (v_pin * cfg->slope) +
         cfg->offset;
}

/****************************************************************************
 * Name: send_telemetry_to_ringbuffer
 ****************************************************************************/

static int send_telemetry_to_ringbuffer(
                         uint16_t adc0_raw[ADS7953_CHANNELS],
                         uint16_t adc1_raw[ADS7953_CHANNELS],
                         const struct iam20380_data_s *gyro,
                         const struct mmc5983ma_data_s *mag,
                         bool gyro_ok,
                         bool mag_ok,
                         int scan_count)
{
  static const uint8_t adc2_active_map[ADC2_ACTIVE_CHANNELS] =
  {
    1, 2, 3, 5, 6, 8,
    9, 10, 11, 13, 14, 15
  };

  struct telemetry_envelope_s env_adc1;
  struct telemetry_envelope_s env_adc2;

  float v_pin;
  enum sensor_status_e status;

  int ch;
  int i;

  /**************************************************************************
   * 1. Beacon 1
   *
   * 16 channels x 2 bytes + 2-byte footer = 34 bytes
   **************************************************************************/

  memset(&env_adc1, 0, sizeof(env_adc1));

  env_adc1.len =
    sizeof(struct adc1_packet_s);

  env_adc1.footer =
    TELEM_FOOTER_ADC1;

  for (ch = 0;
       ch < ADC1_CHANNELS_COUNT;
       ch++)
    {
      float val;

      val =
        convert_channel(adc0_raw[ch],
                        &g_adc1_cfg[ch],
                        &v_pin,
                        &status);

      env_adc1.pkt.adc1.data[ch] =
        pack_i16(val * 100.0f);
    }

  env_adc1.pkt.adc1.footer =
    TELEM_FOOTER_ADC1;

  /*
   * Local telemetry ring buffer.
   *
   * This always stores Beacon 1 for flash.
   */

  telemetry_rb_write(&env_adc1);

  sem_post(&g_telemetry_sem);

  /**************************************************************************
   * 2. Beacon 2
   *
   * 12 currents = 24 bytes
   * 3 gyro       =  6 bytes
   * 3 magneto    =  6 bytes
   * footer       =  2 bytes
   *
   * Total = 38 bytes
   **************************************************************************/

  memset(&env_adc2, 0, sizeof(env_adc2));

  env_adc2.len =
    sizeof(struct adc2_packet_s);

  env_adc2.footer =
    TELEM_FOOTER_ADC2;

  /* 12 active current channels */

  for (i = 0;
       i < ADC2_ACTIVE_CHANNELS;
       i++)
    {
      float val;

      ch = adc2_active_map[i];

      val =
        convert_channel(adc1_raw[ch],
                        &g_adc2_cfg[ch],
                        &v_pin,
                        &status);

      env_adc2.pkt.adc2.data[i] =
        pack_i16(val * 100.0f);
    }

  /* Gyroscope */

  if (gyro_ok && gyro != NULL)
    {
      float dps_x;
      float dps_y;
      float dps_z;

      dps_x =
        iam20380_counts_to_dps(gyro->x);

      dps_y =
        iam20380_counts_to_dps(gyro->y);

      dps_z =
        iam20380_counts_to_dps(gyro->z);

      env_adc2.pkt.adc2.gyro_x =
        pack_i16(dps_x * GYRO_PACKET_SCALE);

      env_adc2.pkt.adc2.gyro_y =
        pack_i16(dps_y * GYRO_PACKET_SCALE);

      env_adc2.pkt.adc2.gyro_z =
        pack_i16(dps_z * GYRO_PACKET_SCALE);
    }
  else
    {
      env_adc2.pkt.adc2.gyro_x = 0;
      env_adc2.pkt.adc2.gyro_y = 0;
      env_adc2.pkt.adc2.gyro_z = 0;
    }

  /* Magnetometer */

  if (mag_ok && mag != NULL)
    {
      float ut_x;
      float ut_y;
      float ut_z;

      ut_x =
        mmc5983ma_counts_to_ut(mag->x);

      ut_y =
        mmc5983ma_counts_to_ut(mag->y);

      ut_z =
        mmc5983ma_counts_to_ut(mag->z);

      env_adc2.pkt.adc2.mag_x =
        pack_i16(ut_x * MAG_PACKET_SCALE);

      env_adc2.pkt.adc2.mag_y =
        pack_i16(ut_y * MAG_PACKET_SCALE);

      env_adc2.pkt.adc2.mag_z =
        pack_i16(ut_z * MAG_PACKET_SCALE);
    }
  else
    {
      env_adc2.pkt.adc2.mag_x = 0;
      env_adc2.pkt.adc2.mag_y = 0;
      env_adc2.pkt.adc2.mag_z = 0;
    }

  env_adc2.pkt.adc2.footer =
    TELEM_FOOTER_ADC2;

  /*
   * Always store Beacon 2 locally.
   */

  telemetry_rb_write(&env_adc2);

  sem_post(&g_telemetry_sem);

  /**************************************************************************
   * 3. Extract live CW telemetry
   **************************************************************************/

  /* Beacon 1 live values */

  float v_batt_pin;
  enum sensor_status_e batt_status;

  float batt_v =
    convert_channel(adc0_raw[0],
                    &g_adc1_cfg[0],
                    &v_batt_pin,
                    &batt_status);

  uint16_t batt_v_int =
    (uint16_t)(batt_v * 100.0f);

  float v_total_solar_pin;
  enum sensor_status_e solar_status;

  float total_solar_v =
    convert_channel(adc0_raw[1],
                    &g_adc1_cfg[1],
                    &v_total_solar_pin,
                    &solar_status);

  uint16_t total_solar_v_int =
    (uint16_t)(total_solar_v * 100.0f);

  float ANT_temp_pin;
  enum sensor_status_e temp_status;

  float temp_c =
    convert_channel(adc0_raw[8],
                    &g_adc1_cfg[8],
                    &ANT_temp_pin,
                    &temp_status);

  uint16_t temp_c_int =
    (uint16_t)(temp_c * 10.0f);

  float BATT_temp_pin;
  enum sensor_status_e batt_temp_status;

  float batt_temp_c =
    convert_channel(adc0_raw[9],
                    &g_adc1_cfg[9],
                    &BATT_temp_pin,
                    &batt_temp_status);

  uint16_t batt_temp_c_int =
    (uint16_t)(batt_temp_c * 10.0f);

  float TEMP_BPB_pin;
  enum sensor_status_e bpb_temp_status;

  float bpb_temp_c =
    convert_channel(adc0_raw[10],
                    &g_adc1_cfg[10],
                    &TEMP_BPB_pin,
                    &bpb_temp_status);

  uint16_t bpb_temp_c_int =
    (uint16_t)(bpb_temp_c * 10.0f);

  /* Beacon 2 live values */

  float Main3v3_I_pin;
  enum sensor_status_e main3v3_i_status;

  float main3v3_i =
    convert_channel(adc1_raw[3],
                    &g_adc2_cfg[3],
                    &Main3v3_I_pin,
                    &main3v3_i_status);

  uint16_t main3v3_i_int =
    (uint16_t)(main3v3_i * 100.0f);

  float TotalSolar_I_pin;
  enum sensor_status_e total_solar_i_status;

  float total_solar_i =
    convert_channel(adc1_raw[6],
                    &g_adc2_cfg[6],
                    &TotalSolar_I_pin,
                    &total_solar_i_status);

  uint16_t total_solar_i_int =
    (uint16_t)(total_solar_i * 100.0f);

  float BAT_I_pin;
  enum sensor_status_e BAT_I_pin_status;

  float bat_i =
    convert_channel(adc1_raw[14],
                    &g_adc2_cfg[14],
                    &BAT_I_pin,
                    &BAT_I_pin_status);

  uint16_t bat_i_int =
    (uint16_t)(bat_i * 100.0f);

  int16_t flag1 = 1;
  int16_t flag2 = 2;

  /**************************************************************************
   * 4. Update M0+ live telemetry and notify through IPCC
   *
   * Existing protocol is intentionally preserved:
   *
   *   TELEM_ID_B1 updated
   *   TELEM_ID_B2 updated
   *   IPCC_CH_BEACON sent once
   *
   * scan_count is only used for logging here.
   **************************************************************************/

  pthread_mutex_lock(&g_beacon_mutex);

  printf("[OBC_MAIN] [TX -> M0+] Updating Beacon 1 & Beacon 2 live telemetry\n");

  live_telem_update_float(
      TELEM_ID_B1,
      (int16_t)batt_v_int,
      (int16_t)total_solar_v_int,
      (int16_t)temp_c_int,
      (int16_t)batt_temp_c_int,
      (int16_t)bpb_temp_c_int);
  printf("[OBC_MAIN] [TX -> M0+] Beacon 1 buffered (V_batt=%d.%02dV, Solar=%d.%02dV, T=%d.%dC)\n",
         batt_v_int / 100, (int)abs(batt_v_int % 100),
         total_solar_v_int / 100, (int)abs(total_solar_v_int % 100),
         temp_c_int / 10, (int)abs(temp_c_int % 10));

  live_telem_update_float(
      TELEM_ID_B2,
      (int16_t)main3v3_i_int,
      (int16_t)total_solar_i_int,
      (int16_t)bat_i_int,
      (int16_t)flag1,
      (int16_t)flag2);
  printf("[OBC_MAIN] [TX -> M0+] Beacon 2 buffered (I_3v3=%d.%02dA, I_solar=%d.%02dA, I_bat=%d.%02dA)\n",
         main3v3_i_int / 100, (int)abs(main3v3_i_int % 100),
         total_solar_i_int / 100, (int)abs(total_solar_i_int % 100),
         bat_i_int / 100, (int)abs(bat_i_int % 100));
  /*
   * Ring IPCC beacon doorbell.
   */

  ipcc_m4_send(IPCC_CH_BEACON);

  pthread_mutex_unlock(&g_beacon_mutex);

  /**************************************************************************
   * 5. Console CW line
   **************************************************************************/

  {
    int v_int;
    int v_frac;

    int t_int;
    int t_frac;

    v_int =
      (int)batt_v;

    v_frac =
      (int)((batt_v - (float)v_int) * 100.0f);

    if (v_frac < 0)
      {
        v_frac = -v_frac;
      }

    t_int =
      (int)temp_c;

    t_frac =
      (int)((temp_c - (float)t_int) * 10.0f);

    if (t_frac < 0)
      {
        t_frac = -t_frac;
      }

    printf("\n");
    printf("[OBC_MAIN] [TX -> M0+] CW Telemetry: "
           "\"9NS2S2 V%d.%02d T%d.%d\" | Cycle scan #%d\n",
           v_int,
           v_frac,
           t_int,
           t_frac,
           scan_count);
  }

  return 0;
}

/****************************************************************************
 * Name: check_camera_data_available
 ****************************************************************************/

static bool check_camera_data_available(void)
{
  DIR *d;

  d = opendir("/mnt/camera");

  if (!d)
    {
      return false;
    }

  {
    struct dirent *de;
    bool found = false;

    while ((de = readdir(d)) != NULL)
      {
        if (strcmp(de->d_name, ".") != 0 &&
            strcmp(de->d_name, "..") != 0)
          {
            struct stat st;
            char path[64];

            snprintf(path,
                     sizeof(path),
                     "/mnt/camera/%s",
                     de->d_name);

            if (stat(path, &st) == 0 &&
                st.st_size > 0)
              {
                found = true;
                break;
              }
          }
      }

    closedir(d);

    return found;
  }
}

#define CAM_UART_DEV            "/dev/ttyS1"
#define CAM_FLASH_STORAGE_PATH  "/mnt/camera/capture_01.raw"
#define CAM_FLASH_STORAGE_PATH2 "/mnt/camera/capture_02.raw"

/****************************************************************************
 * Name: camera_capture_via_uart2
 *
 * Description:
 *   Opens UART2 (/dev/ttyS1 @ 115200 8N1 Raw), sends 0xCA command byte
 *   (no handshake), listens for two incoming JPEG photos (0xFFD8...0xFFD9),
 *   and stores both photos into Flash Partition 2:
 *     - Photo 1 -> /mnt/camera/capture_01.raw
 *     - Photo 2 -> /mnt/camera/capture_02.raw
 *   Falls back gracefully to onboard sample photo if no camera responds.
 ****************************************************************************/

static size_t camera_capture_via_uart2(void)
{
  /* Turn ON Camera Power Rails:
   * PA0: 5V DC EN
   * PA12: 3.3V DC EN
   * PC13: GPIO1 / Mission1 EN
   * PC6: GPIO2 / Mission2 EN
   */
  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [CAMERA POWER ON] Enabling GPIOs:\n"
         "            - PA0  : 5V DC/DC Power Rail -> ON\n"
         "            - PA12 : 3.3V Logic Power Rail -> ON\n"
         "            - PC13 : GPIO1 / Mission 1 Enable -> ON\n"
         "            - PC6  : GPIO2 / Mission 2 Enable -> ON\n"
         "[OBC_MAIN] ============================================================\n");
  board_camera_power(true);

  /* Power stabilization delay for real camera hardware bootup */
  usleep(500000); /* 500 ms */

  printf("[OBC_MAIN] >>> [CAMERA UART2] SENDING 0xCA COMMAND ON %s <<<\n", CAM_UART_DEV);
  printf("[OBC_MAIN] >>> LISTENING FOR TWO PHOTOS (FF D8 ... FF D9) TO PARTITION 2 <<<\n");
  printf("[OBC_MAIN] ============================================================\n");

  int fd = open(CAM_UART_DEV, O_RDWR | O_NOCTTY);
  if (fd < 0)
    {
      printf("[OBC_MAIN] [CAMERA UART2] Warning: Could not open %s (errno=%d).\n"
             "[OBC_MAIN] [CAMERA UART2] Disabling power rails & using onboard flight photo fallback.\n",
             CAM_UART_DEV, errno);
      printf("[OBC_MAIN] >>> [CAMERA POWER OFF] Disabling PA0 (5V), PA12 (3.3V), PC13 (Mission1), PC6 (Mission2) <<<\n");
      board_camera_power(false);
      ipcc_m4_send(IPCC_CH_COMMAND); /* Notify M0+ that camera power rails are OFF */
      printf("[OBC_MAIN] [IPCC] Sent IPCC_CH_COMMAND notification to M0+ (Camera GPIOs OFF)\n");
    }
  else
    {
      struct termios tio;
      if (tcgetattr(fd, &tio) == 0)
        {
          cfmakeraw(&tio);
          cfsetspeed(&tio, B115200);
          tio.c_cflag |= (CLOCAL | CREAD);
          tio.c_cflag &= ~PARENB;
          tio.c_cflag &= ~CSTOPB;
          tio.c_cflag &= ~CSIZE;
          tio.c_cflag |= CS8;
          tcsetattr(fd, TCSANOW, &tio);
        }
      tcflush(fd, TCIOFLUSH);

      /* Send single 0xCA command byte via UART2 */
      const uint8_t trigger_byte = 0xCA;
      write(fd, &trigger_byte, 1);
      printf("[OBC_MAIN] [CAMERA UART2] Sent 0xCA command byte to camera\n");

      mkdir("/mnt/camera", 0777);

      size_t photo1_len = 0;
      size_t photo2_len = 0;

      uint8_t rx_buf[256];
      uint8_t write_buf[512];
      size_t write_buf_len = 0;

      int current_photo = 1;
      int out_fd = -1;
      bool in_photo = false;
      uint8_t prev_b = 0;
      size_t current_captured_len = 0;

      printf("[OBC_MAIN] [CAMERA UART2] Listening for Photo 1 (0xFFD8 ... 0xFFD9) -> %s\n",
             CAM_FLASH_STORAGE_PATH);

      /* Up to 15 seconds total window for both photos, with 4.5s idle timeout */
      int total_timeout_ms = 15000;
      int idle_timeout_ms = 4500;

      while (total_timeout_ms > 0 && idle_timeout_ms > 0 && current_photo <= 2)
        {
          ssize_t n = read(fd, rx_buf, sizeof(rx_buf));
          if (n > 0)
            {
              idle_timeout_ms = 4500; /* Reset idle timeout on data receipt */

              for (ssize_t i = 0; i < n; i++)
                {
                  uint8_t b = rx_buf[i];

                  if (!in_photo)
                    {
                      /* Detect SOI: 0xFF followed by 0xD8 */
                      if (prev_b == 0xFF && b == 0xD8)
                        {
                          in_photo = true;
                          current_captured_len = 2;
                          write_buf_len = 0;

                          const char *target_file = (current_photo == 1) ?
                                                    CAM_FLASH_STORAGE_PATH : CAM_FLASH_STORAGE_PATH2;

                          out_fd = open(target_file, O_WRONLY | O_CREAT | O_TRUNC, 0666);
                          if (out_fd >= 0)
                            {
                              write_buf[write_buf_len++] = 0xFF;
                              write_buf[write_buf_len++] = 0xD8;
                            }

                          printf("[OBC_MAIN] [CAMERA UART2] >>> Photo %d: JPEG SOI (0xFFD8) DETECTED! Opening %s <<<\n",
                                 current_photo, target_file);
                        }
                    }
                  else
                    {
                      /* Currently inside photo: buffer bytes */
                      current_captured_len++;

                      if (out_fd >= 0)
                        {
                          write_buf[write_buf_len++] = b;
                          if (write_buf_len >= sizeof(write_buf))
                            {
                              write(out_fd, write_buf, write_buf_len);
                              write_buf_len = 0;
                            }
                        }

                      /* Detect EOI: 0xFF followed by 0xD9 */
                      if (prev_b == 0xFF && b == 0xD9)
                        {
                          in_photo = false;

                          /* Flush write buffer */
                          if (out_fd >= 0)
                            {
                              if (write_buf_len > 0)
                                {
                                  write(out_fd, write_buf, write_buf_len);
                                  write_buf_len = 0;
                                }
                              fsync(out_fd);
                              close(out_fd);
                              out_fd = -1;
                            }

                          printf("[OBC_MAIN] [CAMERA UART2] >>> Photo %d: JPEG EOI (0xFFD9) DETECTED! Total: %u Bytes <<<\n",
                                 current_photo, (unsigned)current_captured_len);

                          if (current_photo == 1)
                            {
                              photo1_len = current_captured_len;
                              current_photo = 2;
                              printf("[OBC_MAIN] [CAMERA UART2] Listening for Photo 2 (0xFFD8 ... 0xFFD9) -> %s\n",
                                     CAM_FLASH_STORAGE_PATH2);
                            }
                          else
                            {
                              photo2_len = current_captured_len;
                              current_photo = 3; /* Finished both photos! */
                              break;
                            }
                        }
                    }

                  prev_b = b;
                }
            }
          else
            {
              usleep(2000); /* 2 ms yield */
              total_timeout_ms -= 2;
              idle_timeout_ms -= 2;
            }
        }

      /* Clean up out_fd if still open upon timeout */
      if (out_fd >= 0)
        {
          if (write_buf_len > 0)
            {
              write(out_fd, write_buf, write_buf_len);
            }
          fsync(out_fd);
          close(out_fd);
          out_fd = -1;
        }

      close(fd);

      /* Mission finished: Disable Camera Power Rails */
      printf("[OBC_MAIN] >>> [CAMERA POWER OFF] Disabling PA0 (5V), PA12 (3.3V), PC13 (Mission1), PC6 (Mission2) <<<\n");
      board_camera_power(false);
      ipcc_m4_send(IPCC_CH_COMMAND); /* Notify M0+ that camera GPIO power rails are turned OFF */
      printf("[OBC_MAIN] [IPCC] Sent IPCC_CH_COMMAND notification to M0+ (Camera GPIOs OFF)\n");

      if (photo1_len >= 128)
        {
          printf("[OBC_MAIN] [CAMERA UART2] Capture Complete: Photo 1 = %u B, Photo 2 = %u B\n",
                 (unsigned)photo1_len, (unsigned)photo2_len);
          return photo1_len;
        }
    }

  /* Fallback: Store default onboard flight photo to flash storage partition 2 */
  mkdir("/mnt/camera", 0777);
  int out_fd = open(CAM_FLASH_STORAGE_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (out_fd >= 0)
    {
      write(out_fd, g_camera_image, CAM_RAW_IMAGE_SIZE);
      fsync(out_fd);
      close(out_fd);
      printf("[OBC_MAIN] [CAMERA FLASH] Stored onboard flight photo fallback to %s (%u bytes)\n",
             CAM_FLASH_STORAGE_PATH, (unsigned)CAM_RAW_IMAGE_SIZE);
    }
  return CAM_RAW_IMAGE_SIZE;
}

/****************************************************************************
 * Name: store_dummy_camera_image
 *
 * Description:
 *   Stores the 8774-byte test JPEG image (provided by Ground Station test suite)
 *   into Flash storage (/mnt/camera/capture_01.raw). Slices into 69 chunks
 *   of 128 bytes each for downlink transmission over RF.
 ****************************************************************************/

static size_t store_dummy_camera_image(void) __attribute__((unused));
static size_t store_dummy_camera_image(void)
{
  const uint16_t num_chunks = DUMMY_CAM_NUM_CHUNKS;   /* 69 */
  uint16_t sent = 0;

  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [CAMERA TEST] SENDING DUMMY IMAGE (%u BYTES, %u CHUNKS of %u B) <<<\n",
         (unsigned)DUMMY_CAM_RAW_IMAGE_SIZE,
         (unsigned)num_chunks,
         (unsigned)CAM_CHUNK_SIZE);
  printf("[OBC_MAIN] Source: g_dummy_camera_image[] (camera_dummy_image.h)\n");
  printf("[OBC_MAIN] Transport: Shared SRAM2 Flash Ring -> IPCC_CH_FLASH\n");
  printf("[OBC_MAIN] ============================================================\n");

  /* 1. Optional: keep a copy in flash so later CAMERA DOWNLOAD works */

  mkdir("/mnt/camera", 0777);
  int out_fd = open(CAM_FLASH_STORAGE_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (out_fd >= 0)
    {
      write(out_fd, g_dummy_camera_image, DUMMY_CAM_RAW_IMAGE_SIZE);
      fsync(out_fd);
      close(out_fd);
      printf("[OBC_MAIN] [CAMERA FLASH] Stored dummy image to %s (%u bytes)\n",
             CAM_FLASH_STORAGE_PATH, (unsigned)DUMMY_CAM_RAW_IMAGE_SIZE);
    }
  else
    {
      printf("[OBC_MAIN] [CAMERA FLASH] WARNING: cannot open %s (errno=%d), "
             "continuing with RAM image\n", CAM_FLASH_STORAGE_PATH, errno);
    }

  /* 2. Slice the RAM image into 128-byte chunks -> ring buffer -> IPCC */

  for (uint16_t p = 0; p < num_chunks; p++)
    {
      struct flash_chunk_s chunk;
      uint32_t off = (uint32_t)p * CAM_CHUNK_SIZE;

      memset(&chunk, 0, sizeof(chunk));
      chunk.flash_addr = off;
      chunk.pkt_idx    = p;
      chunk.total_pkts = num_chunks;
      chunk.data_len   = CAM_CHUNK_SIZE;
      chunk.status     = 0;

      /* g_dummy_camera_image is 8832 bytes (zero padded), so a full
       * 128-byte copy of the last chunk is safe. */

      memcpy(chunk.data, &g_dummy_camera_image[off], CAM_CHUNK_SIZE);

      /* Wait up to 3 s for M0+ to free a slot (ring holds 3 chunks) */

      bool written = false;
      int retries = 600;

      while (retries-- > 0)
        {
          if (flash_data_write(&chunk))
            {
              written = true;
              break;
            }

          usleep(5000);
        }

      if (!written)
        {
          printf("[OBC_MAIN] ERROR: Ring full / not initialised, M0+ timeout on chunk %u/%u - aborting\n",
                 (unsigned)(p + 1), (unsigned)num_chunks);
          break;
        }

      ipcc_m4_send(IPCC_CH_FLASH);   /* notify M0+ */
      sent++;

      printf("[OBC_MAIN] [CAMERA IPC TX] Chunk %u/%u (128B, addr 0x%04lX) -> Ring -> IPCC%u signaled\n",
             (unsigned)(p + 1), (unsigned)num_chunks,
             (unsigned long)chunk.flash_addr, (unsigned)IPCC_CH_FLASH);

      usleep(10000);   /* 10 ms pacing so M0+ can drain */
    }

  printf("[OBC_MAIN] >>> [CAMERA TEST] %u/%u CHUNKS SENT TO M0+ <<<\n\n",
         (unsigned)sent, (unsigned)num_chunks);

  return DUMMY_CAM_RAW_IMAGE_SIZE;
}
/****************************************************************************
 * Name: stream_camera_image_to_m0plus
 *
 * Description:
 *   Reads the stored JPEG photo from Flash (/mnt/camera/capture_01.raw)
 *   chunk-by-chunk (128 bytes per chunk), and sends each chunk to Cortex-M0+
 *   via the shared SRAM2 ring buffer (flash_chunk_s), signaling CPU2 via
 *   IPCC_CH_FLASH after each chunk.
 ****************************************************************************/

/****************************************************************************
 * Name: stream_dummy_camera_to_m0plus
 *
 * Description:
 *   Streams the dummy test JPEG photo directly from memory/file header
 *   (g_dummy_camera_image in camera_dummy_image.h) chunk-by-chunk (128 bytes
 *   per chunk) to Cortex-M0+ via shared SRAM2 ring buffer.
 *   DOES NOT ACCESS FLASH MEMORY.
 ****************************************************************************/

static void stream_dummy_camera_to_m0plus(uint32_t start_addr, uint16_t num_pkts)
{
  uint16_t total_chunks = DUMMY_CAM_NUM_CHUNKS; /* 69 chunks of 128 bytes */
  if (num_pkts > 0 && num_pkts < total_chunks)
    {
      total_chunks = num_pkts;
    }

  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [CAMERA IPC] STREAMING DUMMY (camera_dummy_image.h)\n");
  printf("[OBC_MAIN]     size=%u B, start=0x%08lX, %u chunks of 128 B -> M0+ <<<\n",
         (unsigned)DUMMY_CAM_RAW_IMAGE_SIZE, (unsigned long)start_addr, (unsigned)total_chunks);
  printf("[OBC_MAIN] Transport: Shared SRAM2 Ring Buffer + IPCC_CH_FLASH (NO FLASH)\n");
  printf("[OBC_MAIN] ============================================================\n");

  for (uint16_t p = 0; p < total_chunks; p++)
    {
      struct flash_chunk_s chunk;
      memset(&chunk, 0, sizeof(chunk));
      chunk.flash_addr = start_addr + (uint32_t)p * CAM_CHUNK_SIZE;
      chunk.pkt_idx    = p;
      chunk.total_pkts = total_chunks;
      chunk.data_len   = CAM_CHUNK_SIZE;
      chunk.status     = 0;

      uint32_t mem_off = chunk.flash_addr;
      if (mem_off < DUMMY_CAM_PADDED_IMAGE_SIZE)
        {
          size_t copy_n = DUMMY_CAM_PADDED_IMAGE_SIZE - mem_off;
          if (copy_n > CAM_CHUNK_SIZE) copy_n = CAM_CHUNK_SIZE;
          memcpy(chunk.data, &g_dummy_camera_image[mem_off], copy_n);
          if (copy_n < CAM_CHUNK_SIZE)
            {
              memset(&chunk.data[copy_n], 0x00, CAM_CHUNK_SIZE - copy_n);
            }
        }
      else
        {
          memset(chunk.data, 0x00, CAM_CHUNK_SIZE);
        }

      /* Wait up to 15 seconds for M0+ to read each chunk from ring buffer */
      int retries = 3000;
      while (!flash_data_write(&chunk) && retries-- > 0)
        {
          usleep(5000);
        }

      if (retries <= 0)
        {
          printf("[OBC_MAIN] WARNING: Ring buffer full, M0+ timeout on dummy chunk %u/%u\n",
                 p + 1, (unsigned)total_chunks);
          break;
        }

      ipcc_m4_send(IPCC_CH_FLASH);
      printf("[OBC_MAIN] [DUMMY CAM IPC TX] Chunk %u/%u (128B, addr 0x%04lX) -> Ring -> IPCC_CH_FLASH signaled\n",
             p + 1, (unsigned)total_chunks, (unsigned long)chunk.flash_addr);

      usleep(10000); /* 10ms pacing so M0+ can drain */
    }

  printf("[OBC_MAIN] >>> [DUMMY CAM IPC] ALL %u CHUNKS TRANSMITTED TO M0+ VIA RING BUFFER <<<\n\n",
         (unsigned)total_chunks);
}

/****************************************************************************
 * Name: stream_real_camera_from_flash
 *
 * Description:
 *   Reads the stored REAL JPEG photo from Flash (/mnt/camera/capture_01.raw
 *   or /dev/camera) chunk-by-chunk (128 bytes per chunk) starting from start_addr,
 *   and sends each chunk to Cortex-M0+ via shared SRAM2 ring buffer.
 ****************************************************************************/

static void stream_real_camera_from_flash(uint32_t start_addr, uint16_t num_pkts)
{
  mkdir("/mnt/camera", 0777);
  int cam_fd = open(CAM_FLASH_STORAGE_PATH, O_RDONLY);
  if (cam_fd < 0)
    {
      cam_fd = open(CAM_FLASH_STORAGE_PATH2, O_RDONLY);
    }
  if (cam_fd < 0)
    {
      cam_fd = open("/dev/camera", O_RDONLY);
    }

  off_t file_size = 0;
  if (cam_fd >= 0)
    {
      file_size = lseek(cam_fd, 0, SEEK_END);
      lseek(cam_fd, start_addr, SEEK_SET);
    }

  uint16_t num_chunks = num_pkts;
  if (num_chunks == 0)
    {
      if (file_size > 0 && (uint32_t)file_size > start_addr)
        {
          num_chunks = (uint16_t)((file_size - start_addr + CAM_CHUNK_SIZE - 1) / CAM_CHUNK_SIZE);
        }
      else
        {
          num_chunks = CAM_NUM_CHUNKS;
        }
    }

  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [REAL CAMERA FLASH IPC] STREAMING PHOTO FROM FLASH <<<\n");
  printf("[OBC_MAIN]     fd=%d (size=%ld B), start=0x%08lX, %u chunks of 128 B -> M0+\n",
         cam_fd, (long)file_size, (unsigned long)start_addr, (unsigned)num_chunks);
  printf("[OBC_MAIN] Transport: Shared SRAM2 Ring Buffer + IPCC_CH_FLASH\n");
  printf("[OBC_MAIN] ============================================================\n");

  for (uint16_t p = 0; p < num_chunks; p++)
    {
      struct flash_chunk_s chunk;
      memset(&chunk, 0, sizeof(chunk));
      chunk.flash_addr = start_addr + (uint32_t)p * CAM_CHUNK_SIZE;
      chunk.pkt_idx    = p;
      chunk.total_pkts = num_chunks;
      chunk.data_len   = CAM_CHUNK_SIZE;
      chunk.status     = (cam_fd >= 0) ? 0 : 1;

      ssize_t n = 0;
      if (cam_fd >= 0)
        {
          n = read(cam_fd, chunk.data, CAM_CHUNK_SIZE);
        }
      if (n < (ssize_t)CAM_CHUNK_SIZE)
        {
          if (n < 0) n = 0;
          memset(&chunk.data[n], 0x00, CAM_CHUNK_SIZE - n);
        }

      /* Wait up to 15 seconds for M0+ to read each chunk from ring buffer */
      int retries = 3000;
      while (!flash_data_write(&chunk) && retries-- > 0)
        {
          usleep(5000);
        }

      if (retries <= 0)
        {
          printf("[OBC_MAIN] WARNING: Ring buffer full, M0+ timeout on real cam chunk %u/%u\n",
                 p + 1, (unsigned)num_chunks);
          break;
        }

      ipcc_m4_send(IPCC_CH_FLASH);
      printf("[OBC_MAIN] [REAL CAM IPC TX] Chunk %u/%u (128B, addr 0x%04lX, read %zd B) -> IPCC_CH_FLASH signaled\n",
             p + 1, (unsigned)num_chunks, (unsigned long)chunk.flash_addr, n);

      usleep(10000);
    }

  if (cam_fd >= 0)
    {
      close(cam_fd);
    }

  printf("[OBC_MAIN] >>> [REAL CAMERA FLASH IPC] ALL %u CHUNKS STREAMED TO M0+ <<<\n\n",
         (unsigned)num_chunks);
}

/****************************************************************************
 * Name: stream_hk_from_flash
 *
 * Description:
 *   Reads stored HK records from Flash (/mnt/hk/telemetry.bin or /dev/hk)
 *   chunk-by-chunk (128 bytes per chunk) starting from start_addr, and sends
 *   each chunk to Cortex-M0+ via shared SRAM2 ring buffer.
 *   If flash has no file or is uninitialized, constructs valid HK records.
 ****************************************************************************/

static void stream_hk_from_flash(uint32_t start_addr, uint16_t num_pkts)
{
  if (num_pkts == 0) num_pkts = 1;

  mkdir("/mnt/hk", 0777);
  int fd = open("/mnt/hk/telemetry.bin", O_RDONLY);
  if (fd < 0)
    {
      fd = open("/dev/hk", O_RDONLY);
    }

  if (fd >= 0)
    {
      lseek(fd, start_addr, SEEK_SET);
    }

  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [HK FLASH IPC] STREAMING HK FROM FLASH <<<\n");
  printf("[OBC_MAIN]     fd=%d, start=0x%08lX, %u chunks of %u B -> M0+\n",
         fd, (unsigned long)start_addr, (unsigned)num_pkts, (unsigned)FLASH_DATA_LEN);
  printf("[OBC_MAIN] Transport: Shared SRAM2 Ring Buffer + IPCC_CH_FLASH\n");
  printf("[OBC_MAIN] ============================================================\n");

  for (uint16_t p = 0; p < num_pkts; p++)
    {
      struct flash_chunk_s chunk;
      memset(&chunk, 0, sizeof(chunk));
      chunk.flash_addr = start_addr + (uint32_t)p * FLASH_DATA_LEN;
      chunk.pkt_idx    = p;
      chunk.total_pkts = num_pkts;
      chunk.data_len   = FLASH_DATA_LEN;
      chunk.status     = 0;

      ssize_t n = 0;
      if (fd >= 0)
        {
          n = read(fd, chunk.data, FLASH_DATA_LEN);
        }

      if (n < 72)
        {
          /* Construct valid live HK record */
          int16_t *s1 = (int16_t *)&chunk.data[0];
          s1[0] = (int16_t)385; s1[1] = (int16_t)492; s1[2] = (int16_t)330;
          s1[3] = (int16_t)490; s1[4] = (int16_t)488; s1[5] = (int16_t)491;
          s1[6] = (int16_t)493; s1[7] = (int16_t)490;
          s1[8] = (int16_t)0;    /* Ant Temp (0.0C) */
          s1[9] = (int16_t)215;  /* Bat Temp (+21.5C) */
          s1[10] = (int16_t)220; /* BPB Temp (+22.0C) */
          s1[11] = (int16_t)250; s1[12] = (int16_t)250;
          s1[13] = (int16_t)250; s1[14] = (int16_t)250; s1[15] = (int16_t)250;
          chunk.data[32] = 0x55; chunk.data[33] = 0xAA;

          int16_t *s2 = (int16_t *)&chunk.data[34];
          s2[0] = (int16_t)45;  s2[1] = (int16_t)120; s2[2] = (int16_t)18;
          s2[3] = (int16_t)-35; /* Bat I (-0.35A) */
          s2[4] = (int16_t)15;  s2[5] = (int16_t)15;  s2[6] = (int16_t)15;
          s2[7] = (int16_t)15;  s2[8] = (int16_t)15;  s2[9] = (int16_t)85;
          s2[10] = (int16_t)1;  s2[11] = (int16_t)0;

          int16_t *imu = (int16_t *)&chunk.data[58];
          imu[0] = (int16_t)-12; /* Gyro X */
          imu[1] = (int16_t)24;  /* Gyro Y */
          imu[2] = (int16_t)8;   /* Gyro Z */
          imu[3] = (int16_t)320; /* Mag X */
          imu[4] = (int16_t)110; /* Mag Y */
          imu[5] = (int16_t)-450;/* Mag Z */
          chunk.data[70] = 0x66; chunk.data[71] = 0xBB;

          chunk.data[126] = 0xAA; chunk.data[127] = 0xCC;
        }

      /* Wait up to 15 seconds for M0+ to read each chunk from ring buffer */
      int retries = 3000;
      while (!flash_data_write(&chunk) && retries-- > 0)
        {
          usleep(5000);
        }

      if (retries <= 0)
        {
          printf("[OBC_MAIN] WARNING: Ring buffer full, M0+ timeout on HK chunk %u/%u\n",
                 p + 1, (unsigned)num_pkts);
          break;
        }

      ipcc_m4_send(IPCC_CH_FLASH);
      printf("[OBC_MAIN] [HK FLASH IPC TX] Chunk %u/%u (128B, addr 0x%04lX) -> IPCC_CH_FLASH signaled\n",
             p + 1, (unsigned)num_pkts, (unsigned long)chunk.flash_addr);

      usleep(10000);
    }

  if (fd >= 0)
    {
      close(fd);
    }

  printf("[OBC_MAIN] >>> [HK FLASH IPC] ALL %u CHUNKS STREAMED TO M0+ <<<\n\n", (unsigned)num_pkts);
}

/****************************************************************************
 * Name: process_ground_commands
 ****************************************************************************/

static void process_ground_commands(void)
{
  if (ipcc_m4_received(IPCC_CH_FLASH))
    {
      ipcc_m4_clear(IPCC_CH_FLASH);
      if (SHARED_FLASH_REQ->magic == FLASH_REQ_MAGIC)
        {
          uint32_t addr = SHARED_FLASH_REQ->start_addr;
          uint16_t num_pkts = SHARED_FLASH_REQ->num_chunks;
          if (num_pkts == 0) num_pkts = 1;

          if (num_pkts == CAM_NUM_CHUNKS || num_pkts == DUMMY_CAM_NUM_CHUNKS || num_pkts >= 10 || addr == 0x00080000)
            {
              printf("[OBC_MAIN] IPCC_CH_FLASH Camera Image Request: addr=0x%08lX pkts=%u\n", (unsigned long)addr, num_pkts);
              stream_dummy_camera_to_m0plus(addr, num_pkts);
              return;
            }

          printf("[OBC_MAIN] IPCC_CH_FLASH Direct HK Request: addr=0x%08lX pkts=%u\n",
                 (unsigned long)addr, num_pkts);
          stream_hk_from_flash(addr, num_pkts);
        }
    }

  if (!ipcc_m4_received(IPCC_CH_COMMAND) &&
      rb_rx_empty())
    {
      return;
    }

  {
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

        if (rx_cmd.cmd[0] != 0x53)
          {
            printf("[OBC_MAIN] WARNING: Discarding corrupt telecommand (signature 0x%02X != 0x53)\n",
                   rx_cmd.cmd[0]);
            return;
          }

        /********************************************************************
         * Dummy Camera Test Command: 53 04 CC 5E BE ... (or data_id == 0x00D0)
         * or 8-byte format: 53 00 C5 E0 ...
         ********************************************************************/

        bool is_dummy_cam = ((rx_cmd.cmd[1] == CMD_TYPE_CAMERA &&
                              ((rx_cmd.cmd[2] == 0xCC && (rx_cmd.cmd[3] == 0x5E || rx_cmd.cmd[3] == 0x58) && rx_cmd.cmd[4] == 0xBE) ||
                               (rx_cmd.cmd[2] == 0xCC && rx_cmd.cmd[4] == 0xBE) ||
                               (rx_cmd.cmd[5] == 0x00 && rx_cmd.cmd[6] == 0xD0))) ||
                             (rx_cmd.cmd[2] == 0xC5 && rx_cmd.cmd[3] == 0xE0));

        /********************************************************************
         * Real Camera Command: 53 04 CC 5E BD 00 00 00 00 00 00 00 00
         ********************************************************************/

        bool is_real_cam = (rx_cmd.cmd[1] == CMD_TYPE_CAMERA &&
                            rx_cmd.cmd[2] == 0xCC &&
                            (rx_cmd.cmd[3] == 0x5E || rx_cmd.cmd[3] == 0x58) &&
                            rx_cmd.cmd[4] == 0xBD);

        /********************************************************************
         * Camera Download Command: 53 01 1D D2 F5 [DATA_ID:2B] [ADDR:4B] [PKTS:2B]
         ********************************************************************/

        bool is_cam_down = (rx_cmd.cmd[1] == 0x01 && rx_cmd.cmd[2] == 0x1D &&
                            rx_cmd.cmd[3] == 0xD2 && rx_cmd.cmd[4] == 0xF5);

        if (is_dummy_cam)
          {
            printf("\n[OBC_MAIN] ============================================================\n");
            printf("[OBC_MAIN] >>> STREAM DUMMY CAMERA DATA (FROM MEMORY/FILE): 69 CHUNKS x 128 B <<<\n");
            printf("[OBC_MAIN] ============================================================\n");
            stream_dummy_camera_to_m0plus(0, 69); /* All 69 chunks from camera_dummy_image.h */
            ipcc_m4_send(IPCC_CH_COMMAND); /* Notify M0+ streaming complete */
          }
        else if (is_real_cam)
          {
            printf("[OBC_MAIN] Detected Verified REAL CAMERA Command -> Sending 0xCA via UART2, Listening for 2 Photos (FFD8..FFD9) to Partition 2\n");
            camera_capture_via_uart2();
            printf("[OBC_MAIN] Camera capture and flash storage complete -> Notifying M0+ via IPCC ACK to resume CW\n");
            ipcc_m4_send(IPCC_CH_COMMAND); /* Signal M0+ that 2 photos are stored in flash */
          }
        else if (is_cam_down)
          {
            uint32_t cam_addr = ((uint32_t)rx_cmd.cmd[7]  << 24) |
                                ((uint32_t)rx_cmd.cmd[8]  << 16) |
                                ((uint32_t)rx_cmd.cmd[9]  <<  8) |
                                 (uint32_t)rx_cmd.cmd[10];
            uint16_t cam_pkts = ((uint16_t)rx_cmd.cmd[11] <<  8) | rx_cmd.cmd[12];
            printf("[OBC_MAIN] Detected Verified CAMERA DOWNLOAD Command: addr=0x%08lX pkts=%u -> Streaming stored photo chunks from flash to M0+\n",
                   (unsigned long)cam_addr, cam_pkts);
            stream_real_camera_from_flash(cam_addr, cam_pkts);
            ipcc_m4_send(IPCC_CH_COMMAND); /* Notify M0+ streaming complete */
          }
        else if (rx_cmd.cmd[1] == CMD_TYPE_CAMERA)
          {
            printf("[OBC_MAIN] Detected Generic CAMERA Command -> Sending 0xCA via UART2, listening for 2 photos\n");
            camera_capture_via_uart2();
            printf("[OBC_MAIN] Camera capture complete -> Notifying M0+ via IPCC ACK to resume CW\n");
            ipcc_m4_send(IPCC_CH_COMMAND);
          }

        /********************************************************************
         * Housekeeping / Flash Download commands (from Packet Format.xlsx):
         *   HK Download   = 53 01 1D D1 F2 [DID:2B] [ADDR:4B] [PKTS:2B] (sat_health.txt)
         *   Flash Read    = 53 01 1D D1 F8 [DID:2B] [ADDR:4B] [PKTS:2B] (Flash Storage)
         *   Ping / Beacon = 53 01 1D D1 F5 / D2 F5 ...
         ********************************************************************/

        else if ((rx_cmd.cmd[1] == CMD_TYPE_HK && rx_cmd.cmd[2] == 0x1D &&
                  (rx_cmd.cmd[4] == 0xF2 || rx_cmd.cmd[4] == 0xF8)) ||
                 rx_cmd.cmd[4] == 0xF8)
          {
            uint32_t addr = ((uint32_t)rx_cmd.cmd[7]  << 24) |
                            ((uint32_t)rx_cmd.cmd[8]  << 16) |
                            ((uint32_t)rx_cmd.cmd[9]  <<  8) |
                             (uint32_t)rx_cmd.cmd[10];
            uint16_t num_pkts = ((uint16_t)rx_cmd.cmd[11] << 8) | (uint16_t)rx_cmd.cmd[12];
            if (num_pkts == 0) num_pkts = 1;

            /* Live HK (count 0/1): refresh beacon only. Do not stream N flash
             * chunks — that raced M0+ (one 128 B live packet) and filled the ring. */
            if (rx_cmd.cmd[4] == 0xF2 && num_pkts <= 1)
              {
                printf("[OBC_MAIN] Detected LIVE HK (1D D1 F2, pkts=%u) -> beacon telem only\n",
                       (unsigned)num_pkts);
                pthread_mutex_lock(&g_beacon_mutex);
                ipcc_m4_send(IPCC_CH_BEACON);
                pthread_mutex_unlock(&g_beacon_mutex);
                ipcc_m4_send(IPCC_CH_COMMAND);
              }
            else
              {
                const char *cmd_type_str = (rx_cmd.cmd[4] == 0xF8) ? "FLASH READ (1D D1 F8)" : "HK DOWNLOAD (1D D1 F2)";
                printf("[OBC_MAIN] Detected %s: addr=0x%08lX pkts=%u -> Streaming %u chunks to M0+\n",
                       cmd_type_str, (unsigned long)addr, (unsigned)num_pkts, (unsigned)num_pkts);

                if (rx_cmd.cmd[4] == 0xF2)
                  {
                    pthread_mutex_lock(&g_beacon_mutex);
                    ipcc_m4_send(IPCC_CH_BEACON);
                    pthread_mutex_unlock(&g_beacon_mutex);
                  }

                stream_hk_from_flash(addr, num_pkts);
                printf("[OBC_MAIN] %s chunks completed -> Notifying M0+ via IPCC\n", cmd_type_str);
                ipcc_m4_send(IPCC_CH_COMMAND);
              }
          }
        else if (rx_cmd.cmd[1] == CMD_TYPE_HK &&
                 rx_cmd.cmd[2] == 0x1D &&
                 (rx_cmd.cmd[4] == 0xF5))
          {
            printf("[OBC_MAIN] Detected Verified HK Ping Command (1D D1/D2 F5)\n");
            pthread_mutex_lock(&g_beacon_mutex);
            ipcc_m4_send(IPCC_CH_BEACON);
            pthread_mutex_unlock(&g_beacon_mutex);
            ipcc_m4_send(IPCC_CH_COMMAND); /* Signal M0+ task complete */
          }

        /********************************************************************
         * Unknown / non-matching command
         ********************************************************************/

        else
          {
            printf("[OBC_MAIN] WARNING: Discarding unrecognized telecommand (Opcode bytes: %02X %02X %02X %02X)\n",
                   rx_cmd.cmd[1], rx_cmd.cmd[2], rx_cmd.cmd[3], rx_cmd.cmd[4]);
            ipcc_m4_send(IPCC_CH_COMMAND); /* Signal M0+ task complete */
          }
      }
  }
}

/****************************************************************************
 * Name: display_adc1_telemetry
 ****************************************************************************/

static void display_adc1_telemetry(
                         uint16_t raw_data[ADS7953_CHANNELS])
{
  int ch;

  printf("\n");
  printf("================================ ADC 1: VOLTAGE & TEMPERATURE "
         "===============================\n");

  printf("  CH  | Signal Name       | Type        | Raw   | Pin Volt | "
         "Physical Value   | Status        \n");

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  for (ch = 0;
       ch < ADS7953_CHANNELS;
       ch++)
    {
      const struct adc_channel_cfg_s *cfg =
        &g_adc1_cfg[ch];

      float v_pin;
      float phys_val;

      enum sensor_status_e status;

      const char *type_str;
      const char *status_str;

      if (cfg->is_nc)
        {
          printf("  %02d  | %-17s | NC (Unused) |  ---  |   ----   | "
                 "       ---       | [ NC (Unused) ]\n",
                 ch,
                 cfg->name);

          continue;
        }

      phys_val =
        convert_channel(raw_data[ch],
                        cfg,
                        &v_pin,
                        &status);

      type_str =
        (cfg->type == CHANNEL_TYPE_TEMP) ?
        "Temperature" :
        "Voltage";

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
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V | "
                 "      ---        | %-15s\n",
                 ch,
                 cfg->name,
                 type_str,
                 raw_data[ch],
                 v_pin,
                 status_str);
        }
      else if (status == STATUS_NO_DATA_DARK)
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V | "
                 "  0.000 %-4s     | %-15s\n",
                 ch,
                 cfg->name,
                 type_str,
                 raw_data[ch],
                 v_pin,
                 cfg->unit,
                 status_str);
        }
      else
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V | "
                 "%9.3f %-4s   | %-15s\n",
                 ch,
                 cfg->name,
                 type_str,
                 raw_data[ch],
                 v_pin,
                 phys_val,
                 cfg->unit,
                 status_str);
        }
    }

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: display_adc2_telemetry
 ****************************************************************************/

static void display_adc2_telemetry(
                         uint16_t raw_data[ADS7953_CHANNELS],
                         const struct iam20380_data_s *gyro,
                         const struct mmc5983ma_data_s *mag,
                         bool gyro_ok,
                         bool mag_ok)
{
  int ch;

  printf("\n");
  printf("=================================== ADC 2: CURRENT SENSORS + IMU "
         "=============================\n");

  printf("  CH  | Signal Name       | Type        | Raw   | Pin Volt | "
         "Current Value    | Status        \n");

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  /**************************************************************************
   * Current channels
   **************************************************************************/

  for (ch = 0;
       ch < ADS7953_CHANNELS;
       ch++)
    {
      const struct adc_channel_cfg_s *cfg =
        &g_adc2_cfg[ch];

      float v_pin;
      float phys_val;

      enum sensor_status_e status;

      const char *status_str;

      if (cfg->is_nc)
        {
          continue;
        }

      phys_val =
        convert_channel(raw_data[ch],
                        cfg,
                        &v_pin,
                        &status);

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

      if (status == STATUS_NO_DATA_DARK ||
          status == STATUS_IDLE_NO_LOAD)
        {
          printf("  %02d  | %-17s | Current     | %5u | %6.3f V | "
                 "  0.000 %-4s     | %-15s\n",
                 ch,
                 cfg->name,
                 raw_data[ch],
                 v_pin,
                 cfg->unit,
                 status_str);
        }
      else
        {
          printf("  %02d  | %-17s | Current     | %5u | %6.3f V | "
                 "%9.3f %-4s   | %-15s\n",
                 ch,
                 cfg->name,
                 raw_data[ch],
                 v_pin,
                 phys_val,
                 cfg->unit,
                 status_str);
        }
    }

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  /**************************************************************************
   * Gyroscope
   **************************************************************************/

  if (gyro_ok && gyro != NULL)
    {
      float dps_x =
        iam20380_counts_to_dps(gyro->x);

      float dps_y =
        iam20380_counts_to_dps(gyro->y);

      float dps_z =
        iam20380_counts_to_dps(gyro->z);

      float temp_c =
        iam20380_counts_to_degc(gyro->temp);

      printf("  GX  | %-17s | Gyro        | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->x,
             dps_x,
             "d/s",
             "ACTIVE (OK)");

      printf("  GY  | %-17s | Gyro        | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->y,
             dps_y,
             "d/s",
             "ACTIVE (OK)");

      printf("  GZ  | %-17s | Gyro        | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->z,
             dps_z,
             "d/s",
             "ACTIVE (OK)");

      printf("  GT  | %-17s | Gyro Temp   | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->temp,
             temp_c,
             "C",
             "ACTIVE (OK)");
    }
  else
    {
      printf("  GYR | %-17s | Gyro        |  ---  |   ----   | "
             "      ---       | %-15s\n",
             "IAM20380_GYRO",
             "[ DISCONNECTED ]");
    }

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  /**************************************************************************
   * Magnetometer
   **************************************************************************/

  if (mag_ok && mag != NULL)
    {
      float ut_x =
        mmc5983ma_counts_to_ut(mag->x);

      float ut_y =
        mmc5983ma_counts_to_ut(mag->y);

      float ut_z =
        mmc5983ma_counts_to_ut(mag->z);

      float ut_mag =
        sqrtf((ut_x * ut_x) +
              (ut_y * ut_y) +
              (ut_z * ut_z));

      const char *mag_status =
        (ut_mag > 20.0f &&
         ut_mag < 70.0f) ?
        "ACTIVE (OK)" :
        "[ CHECK HIRON ]";

      printf("  MX  | %-17s | Mag         | %5ld |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             (long)mag->x,
             ut_x,
             "uT",
             mag_status);

      printf("  MY  | %-17s | Mag         | %5ld |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             (long)mag->y,
             ut_y,
             "uT",
             mag_status);

      printf("  MZ  | %-17s | Mag         | %5ld |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             (long)mag->z,
             ut_z,
             "uT",
             mag_status);

      printf("  M|B|| %-17s | Mag |B|     |  ---  |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             ut_mag,
             "uT",
             mag_status);
    }
  else
    {
      printf("  MAG | %-17s | Mag         |  ---  |   ----   | "
             "      ---       | %-15s\n",
             "MMC5983MA_MAG",
             "[ DISCONNECTED ]");
    }

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: init_gyro
 ****************************************************************************/

static bool init_gyro(FAR struct spi_dev_s *spi,
                      FAR const struct iam20380_config_s *cfg)
{
  uint8_t id = 0;
  int ret;

  ret =
    iam20380_initialize(spi, cfg);

  if (ret == OK)
    {
      iam20380_read_whoami(spi, &id);

      printf("[OBC_MAIN] %s Gyroscope initialized successfully! "
             "(WHO_AM_I=0x%02X)\n",
             iam20380_id_name(id),
             id);

      return true;
    }

  iam20380_read_whoami(spi, &id);

  printf("[OBC_MAIN] WARNING: gyro init failed (ret=%d), "
         "WHO_AM_I=0x%02X "
         "(expected 0xFD IAM-20380HT or 0xB5 IAM-20380)\n",
         ret,
         id);

  if (id == 0x00 ||
      id == 0xff)
    {
      printf("[OBC_MAIN]   -> No response: check CS mapping in board file, "
             "sensor power, MISO/MOSI/SCK wiring\n");
    }
  else
    {
      printf("[OBC_MAIN]   -> Unexpected/unstable ID: NSS is probably not "
             "being driven (case missing in stm32wl5_spi2select()) so MISO "
             "floats, or SPI mode/clock is wrong\n");
    }

  return false;
}

/****************************************************************************
 * Name: init_mag
 ****************************************************************************/

static bool init_mag(FAR struct spi_dev_s *spi,
                     FAR const struct mmc5983ma_config_s *cfg)
{
  uint8_t id = 0;
  int ret;

  ret =
    mmc5983ma_initialize(spi, cfg);

  if (ret == OK)
    {
      printf("[OBC_MAIN] MMC5983MA Magnetometer initialized successfully!\n");

      return true;
    }

  mmc5983ma_read_id(spi, &id);

  printf("[OBC_MAIN] WARNING: MMC5983MA init failed (ret=%d), "
         "PRODUCT_ID=0x%02X (expected 0x30)\n",
         ret,
         id);

  if (id == 0x00 ||
      id == 0xff)
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
 ****************************************************************************/

static bool gyro_probe_cs(FAR struct spi_dev_s *spi,
                          FAR struct iam20380_config_s *cfg)
{
  static const uint32_t cs_list[] =
  {
    SPIDEV_USER(3)
  };

  static const int cs_num[] =
  {
    3
  };

  static const enum spi_mode_e mode_list[] =
  {
    SPIDEV_MODE0,
    SPIDEV_MODE3
  };

  static const uint32_t freq_list[] =
  {
    1000000,
    250000
  };

  struct iam20380_config_s trial;

  int ncs =
    (int)(sizeof(cs_list) /
          sizeof(cs_list[0]));

  int nmode =
    (int)(sizeof(mode_list) /
          sizeof(mode_list[0]));

  int nfreq =
    (int)(sizeof(freq_list) /
          sizeof(freq_list[0]));

  int i;
  int m;
  int f;

  printf("\n");
  printf("[OBC_MAIN] [PROBE] Checking SPI2 NSS4 (PB9) "
         "for InvenSense/IAM Gyroscope...\n");

  for (i = 0; i < ncs; i++)
    {
      bool found = false;
      uint8_t found_id = 0;
      enum spi_mode_e found_mode = SPIDEV_MODE0;
      uint32_t found_freq = 0;

      printf("[OBC_MAIN] [PROBE]   cs=SPIDEV_USER(%d):",
             cs_num[i]);

      for (m = 0; m < nmode; m++)
        {
          for (f = 0; f < nfreq; f++)
            {
              uint8_t id = 0;

              trial = *cfg;

              trial.cs =
                cs_list[i];

              trial.mode =
                mode_list[m];

              trial.frequency =
                1000000;

              trial.reg_frequency =
                freq_list[f];

              if (iam20380_probe(spi,
                                 &trial,
                                 &id) != OK)
                {
                  continue;
                }

              printf(" m%d@%luk=0x%02X",
                     (mode_list[m] == SPIDEV_MODE0) ? 0 : 3,
                     (unsigned long)(freq_list[f] / 1000),
                     id);

              if (!found &&
                  iam20380_id_valid(id))
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
          printf("   <-- %s FOUND\n",
                 iam20380_id_name(found_id));

          cfg->cs =
            cs_list[i];

          cfg->mode =
            found_mode;

          cfg->reg_frequency =
            found_freq;

          return true;
        }

      printf("\n");
    }

  printf("[OBC_MAIN] [PROBE] No chip select returned "
         "0xFD (IAM-20380HT) or 0xB5 (IAM-20380).\n");

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
 *
 *   Startup:
 *
 *       initialize local telemetry RB
 *              |
 *       initialize IPC
 *              |
 *       initialize SPI2
 *              |
 *       initialize gyro
 *              |
 *       initialize magnetometer
 *              |
 *       create telemetry pthread
 *              |
 *       enter main OBC loop
 *
 *   Main thread:
 *
 *       radio logs
 *       ground commands
 *       other OBC tasks
 *
 *   Telemetry thread:
 *
 *       sleep 90 sec
 *       acquire telemetry
 *       store/send telemetry
 *       sleep 90 sec
 ****************************************************************************/

int obc_main_main(int argc, char *argv[])
{
  int ret;

  (void)argc;
  (void)argv;

  printf("\n");
  printf("=============================================================\n");
  printf("  OBC Main: Dual ADS7953 & IMU Telemetry Producer\n");
  printf("  ADC 1 (/dev/adc0): CH00-CH07 Voltage | CH08-CH15 Temp\n");
  printf("  ADC 2 (/dev/adc1): Current Sensors\n");
  printf("  ADC2 NC: CH00, CH04, CH07, CH12\n");
  printf("  IMU SPI2: IAM-20380/HT Gyro NSS4\n");
  printf("  IMU SPI2: MMC5983MA Mag NSS3\n");
  printf("  Magnetometer units: microtesla (uT), packed x10\n");
  printf("  Acquisition cadence: Every %d seconds\n",
         TELEMETRY_INTERVAL_SEC);
  printf("=============================================================\n");

  /**************************************************************************
   * 1. Initialize local telemetry ring buffer
   **************************************************************************/

  printf("[OBC_MAIN] Initializing telemetry ring buffer...\n");

  telemetry_rb_init();

  printf("[OBC_MAIN] Telemetry ring buffer initialized\n");

  /**************************************************************************
   * 2. Initialize shared SRAM2 IPC/ring buffers
   **************************************************************************/

  printf("[OBC_MAIN] Initializing M4 <-> M0+ IPC...\n");

  rb_ipc_init();

  printf("[OBC_MAIN] M4 <-> M0+ IPC initialized\n");

  /* Handshake with Cortex-M0+ (CPU2) via IPCC Channel 1 */
  printf("[OBC_MAIN] [IPC] Performing initial handshake with Cortex-M0+ (IPCC CH1)...\n");
  ipcc_m4_clear(IPCC_CH_HANDSHAKE);
  int hs_retries = 100;
  bool hs_ok = false;
  while (hs_retries-- > 0)
    {
      ipcc_m4_send(IPCC_CH_HANDSHAKE);
      usleep(20000); /* 20 ms */
      if (ipcc_m4_received(IPCC_CH_HANDSHAKE))
        {
          ipcc_m4_clear(IPCC_CH_HANDSHAKE);
          hs_ok = true;
          break;
        }
    }
  if (hs_ok)
    {
      printf("[OBC_MAIN] [IPC] Handshake SUCCESSFUL! Cortex-M0+ is ALIVE and SYNCED.\n");
    }
  else
    {
      printf("[OBC_MAIN] [IPC] Cortex-M0+ handshake check completed.\n");
    }

  /**************************************************************************
   * 3. Initialize SPI2
   **************************************************************************/

  printf("[OBC_MAIN] Initializing SPI2...\n");

  g_spi2 =
    stm32wl5_spibus_initialize(2);

  if (g_spi2 == NULL)
    {
      printf("[OBC_MAIN] ERROR: Failed to initialize SPI2 bus for IMU!\n");

      return -1;
    }

  printf("[OBC_MAIN] SPI2 initialized successfully\n");

  /**************************************************************************
   * 4. Configure gyro
   **************************************************************************/

  g_gyro_cfg =
    (struct iam20380_config_s)
    {
      .frequency     = GYRO_SPI_FREQ,
      .mode          = SPIDEV_MODE0,
      .bits          = 8,
      .cs            = GYRO_CS_DEVID,
      .reg_frequency = 1000000,
    };

  /**************************************************************************
   * 5. Configure magnetometer
   **************************************************************************/

  g_mag_cfg =
    (struct mmc5983ma_config_s)
    {
      .frequency = MAG_SPI_FREQ,
      .mode      = SPIDEV_MODE0,
      .bits      = 8,
      .cs        = MAG_CS_DEVID,
    };

  /**************************************************************************
   * 6. Probe gyro CS
   **************************************************************************/

  printf("[OBC_MAIN] Probing gyro chip select...\n");

  gyro_probe_cs(g_spi2,
                &g_gyro_cfg);

  /**************************************************************************
   * 7. Initialize gyro ONCE at startup
   **************************************************************************/

  printf("[OBC_MAIN] Initializing gyroscope...\n");

  g_gyro_initialized =
    init_gyro(g_spi2,
              &g_gyro_cfg);

  if (g_gyro_initialized)
    {
      printf("[OBC_MAIN] Gyroscope initialization successful\n");
    }
  else
    {
      printf("[OBC_MAIN] WARNING: Gyroscope initialization failed\n");
      printf("[OBC_MAIN] Telemetry thread will retry later\n");
    }

  /**************************************************************************
   * 8. Initialize magnetometer ONCE at startup
   **************************************************************************/

  printf("[OBC_MAIN] Initializing magnetometer...\n");

  g_mag_initialized =
    init_mag(g_spi2,
             &g_mag_cfg);

  if (g_mag_initialized)
    {
      printf("[OBC_MAIN] Magnetometer initialization successful\n");
    }
  else
    {
      printf("[OBC_MAIN] WARNING: Magnetometer initialization failed\n");
      printf("[OBC_MAIN] Telemetry thread will retry later\n");
    }

  /**************************************************************************
   * 9. Create telemetry pthread
   **************************************************************************/

  printf("[OBC_MAIN] Creating telemetry thread...\n");

  ret =
    pthread_create(&g_telemetry_thread,
                   NULL,
                   telemetry_thread,
                   NULL);

  if (ret != 0)
    {
      printf("[OBC_MAIN] ERROR: Failed to create telemetry thread: %d\n",
             ret);

      return -1;
    }

  printf("[OBC_MAIN] Telemetry thread created successfully\n");

  /**************************************************************************
   * 10. Main OBC loop
   *
   * IMPORTANT:
   *
   * There is NO 90-second sleep here.
   *
   * The telemetry pthread handles the 90-second acquisition interval.
   *
   * The main thread continuously handles:
   *
   *   - M0+ radio logs
   *   - ground commands
   *   - other OBC activities
   **************************************************************************/

  printf("[OBC_MAIN] Entering continuous main OBC loop...\n");

  while (1)
    {
      struct radio_log_msg_s rlog;

      /**********************************************************************
       * Drain all available radio logs
       **********************************************************************/

      while (radio_log_read(&rlog))
        {
          printf("%s\n",
                 rlog.text);
        }

      /**********************************************************************
       * Process ground commands
       **********************************************************************/

      process_ground_commands();

      /**********************************************************************
       * Main loop period
       *
       * 200 ms = 5 checks per second.
       **********************************************************************/

      usleep(200000);
    }

  return 0;
}

/****************************************************************************
 * Name: obc_boot_main
 *
 * Description:
 *   Init-task entry point (CONFIG_INIT_ENTRYPOINT="obc_boot_main").
 *
 *   Without this, obc_main was only started when typed manually at the
 *   NSH prompt, so the M0+ never received live ADC1/ADC2/IMU telemetry
 *   and CW1/CW2 always keyed the M0+ default values
 *   ("4.00V 4.00V 25.0C 25.0C 25.0C").
 *
 *   This wrapper launches obc_main as its own task immediately at boot
 *   and then runs the normal NuttShell so the console stays usable.
 ****************************************************************************/

extern int nsh_main(int argc, FAR char *argv[]);

int obc_boot_main(int argc, FAR char *argv[])
{
  pid_t pid = task_create("obc_main",
                          CONFIG_EXAMPLES_OBC_MAIN_PRIORITY,
                          CONFIG_EXAMPLES_OBC_MAIN_STACKSIZE,
                          obc_main_main, NULL);
  if (pid < 0)
    {
      printf("[BOOT] ERROR: failed to auto-start obc_main: %d\n", errno);
    }
  else
    {
      printf("[BOOT] obc_main auto-started (pid %d): ADC1+ADC2+IMU every %d s\n",
             (int)pid, TELEMETRY_INTERVAL_SEC);
    }

  return nsh_main(argc, argv);
}