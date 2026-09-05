/****************************************************************************
 * apps/examples/camera_launcher/launcher.c
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
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <telemetry_rb.h>
#include <Sring_buffer.h>

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

extern int littlefs_main(int argc, char *argv[]);
extern int obc_main_main(int argc, char *argv[]);

#ifdef CONFIG_EXAMPLES_CAMERA_MSN2
extern int camera_MSN2_main(int argc, char *argv[]);
#elif defined(CONFIG_EXAMPLES_CAMERA)
extern int camera_main(int argc, char *argv[]);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: launcher_main
 ****************************************************************************/

int launcher_main(int argc, char *argv[])
{
  int littlefs_pid;
  int obc_pid;
#if defined(CONFIG_EXAMPLES_CAMERA_MSN2) || defined(CONFIG_EXAMPLES_CAMERA)
  int cam_pid;
#endif

  /* Ensure standard I/O (stdin, stdout, stderr) are connected to /dev/console */
  if (isatty(1) == 0)
    {
      int cfd = open("/dev/console", 02 /* O_RDWR */);
      if (cfd >= 0)
        {
          if (cfd != 0) dup2(cfd, 0);
          if (cfd != 1) dup2(cfd, 1);
          if (cfd != 2) dup2(cfd, 2);
          if (cfd > 2) close(cfd);
        }
    }

  printf("\n=======================================================\n");
  printf("  OBC Master Launcher: Initializing Satellite Subsystems\n");
  printf("=======================================================\n");

  /* 1. Initialize Dual-Core Shared SRAM2 IPC and Local Ring Buffer */
  telemetry_rb_init();
  rb_ipc_init();

  /* 2. Handshake with Cortex-M0+ (CPU2) via IPCC Channel 3 */
  printf("[LAUNCHER] [IPC] Performing initial handshake with Cortex-M0+ (IPCC CH3)...\n");

  ipcc_m4_clear(IPCC_CH_HANDSHAKE);

  int timeout_ms = 5000;
  bool handshake_ok = false;
  while (timeout_ms > 0)
    {
      if ((timeout_ms % 100) == 0)
        {
          ipcc_m4_send(IPCC_CH_HANDSHAKE);
        }

      if (ipcc_m4_received(IPCC_CH_HANDSHAKE))
        {
          ipcc_m4_clear(IPCC_CH_HANDSHAKE);
          handshake_ok = true;
          break;
        }
      usleep(10000); /* 10 ms */
      timeout_ms -= 10;
    }

  if (handshake_ok)
    {
      printf("[LAUNCHER] [IPC] Handshake SUCCESSFUL! Cortex-M0+ is ALIVE and SYNCED.\n");
      struct radio_log_msg_s rlog;
      while (radio_log_read(&rlog))
        {
          printf("%s\n", rlog.text);
        }
    }
  else
    {
      printf("[LAUNCHER] [IPC] WARNING: Cortex-M0+ handshake timeout (proceeding with OBC bring-up).\n");
    }

  /* 3. Launch OBC Main Sensor Acquisition Task (Stack: 1536 Bytes) */
  printf("[LAUNCHER] Spawning OBC Main sensor acquisition task...\n");
  obc_pid = task_create("obc_main", 100, 1536, obc_main_main, NULL);
  if (obc_pid < 0)
    {
      printf("[LAUNCHER] ERROR: Failed to start OBC Main task (ret: %d, errno: %d)\n", obc_pid, errno);
    }
  else
    {
      printf("[LAUNCHER] OBC Main task running (PID: %d)\n", obc_pid);
    }

  /* 3. Launch LittleFS Telemetry Storage Task (Stack: 2048 Bytes) */
  printf("[LAUNCHER] Spawning LittleFS storage daemon...\n");
  littlefs_pid = task_create("littlefs", 100, 2048, littlefs_main, NULL);
  if (littlefs_pid < 0)
    {
      printf("[LAUNCHER] ERROR: Failed to start LittleFS daemon (ret: %d, errno: %d)\n", littlefs_pid, errno);
    }
  else
    {
      printf("[LAUNCHER] LittleFS daemon running (PID: %d)\n", littlefs_pid);
    }

  /* 4. Launch Camera Task (if configured) */
#if defined(CONFIG_EXAMPLES_CAMERA_MSN2)
  printf("[LAUNCHER] Spawning Camera MSN2 capture task...\n");
  cam_pid = task_create("camera", 100, 2048, camera_MSN2_main, NULL);
  if (cam_pid < 0)
    {
      printf("[LAUNCHER] ERROR: Failed to start Camera task (errno: %d)\n", errno);
    }
  else
    {
      printf("[LAUNCHER] Camera task running (PID: %d)\n", cam_pid);
    }
#elif defined(CONFIG_EXAMPLES_CAMERA)
  printf("[LAUNCHER] Spawning Camera capture task...\n");
  cam_pid = task_create("camera", 100, 2048, camera_main, NULL);
  if (cam_pid < 0)
    {
      printf("[LAUNCHER] ERROR: Failed to start Camera task (errno: %d)\n", errno);
    }
  else
    {
      printf("[LAUNCHER] Camera task running (PID: %d)\n", cam_pid);
    }
#else
  printf("[LAUNCHER] Camera task not enabled in configuration.\n");
#endif

  printf("=======================================================\n");
  printf("  Subsystem Launch Complete! (Services running in background)\n");
  printf("=======================================================\n\n");

  return 0;
}