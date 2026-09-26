/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_boot.c
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

#include <nuttx/debug.h>

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/input/buttons.h>

#include <arch/board/board.h>

#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_VIDEO_FB
#include <nuttx/video/fb.h>
#endif

#include <stm32wl5.h>
#include <stm32wl5_uart.h>
#include <stm32wl5_pwr.h>

#include "arm_internal.h"
#include "nucleo-wl55jc.h"

#if defined(CONFIG_ADC_ADS7953)
#include <nuttx/analog/ads7953.h>
#endif

#if defined(CONFIG_STM32WL5_SPI2S2)
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define proc mountpoint in case procfs is used but nsh is not */

#ifndef CONFIG_NSH_PROC_MOUNTPOINT
#define CONFIG_NSH_PROC_MOUNTPOINT "/proc"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_board_initialize
 *
 * Description:
 *   All STM32WL5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/
#ifdef CONFIG_MTD_M25P

static int stm32wl5_mt25q_initialize(void)
{
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  uint8_t id[3] = {0};

  syslog(LOG_INFO, "Initializing external MT25Q NOR Flash...\n");

  /* Initialize SPI1 */

  spi = stm32wl5_spibus_initialize(1);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: SPI1 initialization failed\n");
      return -ENODEV;
    }

  /* Query JEDEC ID directly over SPI1 (RDID 0x9F) */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SEND(spi, 0x9f);
  id[0] = SPI_SEND(spi, 0xff); /* Manufacturer ID (0x20 = Micron) */
  id[1] = SPI_SEND(spi, 0xff); /* Memory Type (0xBA = MT25QL 3V, 0xBB = MT25QU 1.8V) */
  id[2] = SPI_SEND(spi, 0xff); /* Capacity (0x21 = 1 Gbit / 128 MB) */
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  syslog(LOG_INFO, "MT25Q JEDEC ID: Mfg=0x%02X, Type=0x%02X, Cap=0x%02X\n",
         id[0], id[1], id[2]);

  if (id[0] == 0x20 && (id[1] == 0xba || id[1] == 0xbb) && id[2] == 0x21)
    {
      syslog(LOG_INFO, "Detected Micron MT25QL01G (1 Gbit / 128 MB)\n");
    }
  else
    {
      syslog(LOG_WARNING,
             "WARNING: Unexpected JEDEC ID (check SPI wiring/power)\n");
    }

  /* Bind SPI1 to M25P/MT25Q driver */

  mtd = m25p_initialize(spi);
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: m25p_initialize() failed\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "MT25Q MTD initialized successfully\n");

    /* Now initialize the MTD partitions */
  #ifdef CONFIG_MTD_PARTITION
  /* MT25Q Page size is 256 bytes (1 MTD block = 256 bytes) */
  #define MTD_BLOCK_SIZE      256

  /* DIE 0: Housekeeping (HK) - 32 MB */
  /* ADC1 temperature and voltage health for beacon  */

  #define HK1_START_ADDR       0x00000000
  #define HK1_SIZE             (32 * 1024 * 1024) /* 32 MB = 33, 554, 432 bytes */
  #define HK1_START_BLOCK      (HK1_START_ADDR / MTD_BLOCK_SIZE) /* Block 0 */
  #define HK1_NUM_BLOCKS       (HK1_SIZE / MTD_BLOCK_SIZE)       /* 262144 blocks */

  /* ADC2: current information of stallite and IMU data  */
  #define HK2_START_ADDR       0x02000000
  #define HK2_SIZE             (32 * 1024 * 1024)
  #define HK2_START_BLOCK      (HK2_START_ADDR / MTD_BLOCK_SIZE) /* Block 0 */
  #define HK2_NUM_BLOCKS       (HK2_SIZE / MTD_BLOCK_SIZE)       /* 262144 blocks */

  /* DIE 1: Camera Images - 64 MB */
  #define CAM_START_ADDR      0x04000000                       /* 64 MB offset */
  #define CAM_SIZE            (64 * 1024 * 1024)
  #define CAM_START_BLOCK     (CAM_START_ADDR / MTD_BLOCK_SIZE) /* Block 262144 */
  #define CAM_NUM_BLOCKS      (CAM_SIZE / MTD_BLOCK_SIZE)       /* 262144 blocks */

  struct mtd_dev_s *part_hk;
  struct mtd_dev_s *part_cam;

  /* 1. Register Die 0 as /dev/hk (Full 32 MB) */
  /*HK1 ADC1 partition*/
  part_hk = mtd_partition(mtd, HK1_START_BLOCK, HK1_NUM_BLOCKS);
  if (part_hk != NULL)
    {
      register_mtddriver("/dev/hk1", part_hk, 0666, NULL);
      syslog(LOG_INFO, "Registered /dev/hk1 (32 MB - Die 0)\n");
    }
  /*HK2 ADC2 partition*/
  part_hk = mtd_partition(mtd, HK2_START_BLOCK, HK2_NUM_BLOCKS);
  if (part_hk != NULL)
    {
      register_mtddriver("/dev/hk2", part_hk, 0666, NULL);
      syslog(LOG_INFO, "Registered /dev/hk2 (32 MB - Die 0)\n");
    }
  /* 2. Register Die 1 as /dev/camera (Full 64 MB) */
  part_cam = mtd_partition(mtd, CAM_START_BLOCK, CAM_NUM_BLOCKS);
  if (part_cam != NULL)
    {
      register_mtddriver("/dev/camera", part_cam, 0666, NULL);
      syslog(LOG_INFO, "Registered /dev/camera (64 MB - Die 1)\n");
    }

  printf("MT25Q Flash Partitions:\n");
  printf("  /dev/hk1:    Start Block: %d, Num Blocks: %d (Size: %d MB)\n",
         HK1_START_BLOCK, HK1_NUM_BLOCKS, HK1_SIZE / (1024 * 1024));
  printf("  /dev/hk2:    Start Block: %d, Num Blocks: %d (Size: %d MB)\n",
         HK2_START_BLOCK, HK2_NUM_BLOCKS, HK2_SIZE / (1024 * 1024));
  printf("  /dev/camera: Start Block: %d, Num Blocks: %d (Size: %d MB)\n",
         CAM_START_BLOCK, CAM_NUM_BLOCKS, CAM_SIZE / (1024 * 1024));
#endif

  return OK;
}

#endif

void stm32wl5_board_initialize(void)
{
  /* Configure on-board LEDs, which are always enabled */

  board_leds_initialize();
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_BOARD_ENABLE_CPU2)
/****************************************************************************
 * Name: stm32wl5_ensure_cpu2_boot_vector
 *
 * Description:
 *   Ensures that the silicon Option Bytes (FLASH_SRRVR) point CPU2
 *   (Cortex-M0+) to 0x08032000 in Flash. If unconfigured or at factory
 *   default, unlocks Option Bytes, writes the correct boot vector, and
 *   triggers an Option Byte reload (system reset).
 *
 ****************************************************************************/

static void stm32wl5_ensure_cpu2_boot_vector(void)
{
  /* SBRV for 0x08032000:
   * (0x08032000 - 0x08000000) >> 2 = 0x32000 >> 2 = 0xC800.
   * Bit 31 (C2OPT) = 1 (Boot from Flash).
   */

  const uint32_t target_sbrv  = (0x00032000UL >> 2); /* 0xC800 */
  const uint32_t target_srrvr = (1UL << 31) | target_sbrv;
  uint32_t srrvr;

  srrvr = getreg32(STM32WL5_FLASH_SRRVR);

  /* Check if SBRV and C2OPT are already properly set */

  if ((srrvr & 0x8000FFFFUL) == target_srrvr)
    {
      return; /* Hardware already points CPU2 to 0x08032000 */
    }

  printf("\n[SYSTEM] [AUTO-CONFIG] CPU2 boot vector mismatch! (Current SRRVR: 0x%08lx, Target: 0x%08lx)\n",
         (unsigned long)srrvr, (unsigned long)target_srrvr);
  printf("[SYSTEM] [AUTO-CONFIG] Automatically configuring silicon Option Bytes to 0x08032000...\n");
  fflush(stdout);

  /* 1. Wait for any active flash operation to finish */

  while (getreg32(STM32WL5_FLASH_SR) & (FLASH_SR_BSY | FLASH_SR_CFGBSY))
    {
      up_udelay(10);
    }

  /* 2. Unlock Flash Control Register if locked */

  if (getreg32(STM32WL5_FLASH_CR) & FLASH_CR_LOCK)
    {
      putreg32(0x45670123UL, STM32WL5_FLASH_KEYR);
      putreg32(0xCDEF89ABUL, STM32WL5_FLASH_KEYR);
    }

  /* 3. Unlock Option Bytes if locked */

  if (getreg32(STM32WL5_FLASH_CR) & FLASH_CR_OPTLOCK)
    {
      putreg32(0x08192A3BUL, STM32WL5_FLASH_OPTKEYR);
      putreg32(0x4C5D6E7FUL, STM32WL5_FLASH_OPTKEYR);
    }

  /* 4. Write new CPU2 boot vector into FLASH_SRRVR */

  modifyreg32(STM32WL5_FLASH_SRRVR, 0x8000FFFFUL, target_srrvr);

  /* 5. Start Option Byte modification */

  modifyreg32(STM32WL5_FLASH_CR, 0, FLASH_CR_OPTSTRT);

  /* 6. Wait until modification finishes */

  while (getreg32(STM32WL5_FLASH_SR) & (FLASH_SR_BSY | FLASH_SR_CFGBSY))
    {
      up_udelay(10);
    }

  printf("[SYSTEM] [AUTO-CONFIG] Option Bytes programmed! Reloading OBL (resetting system)...\n");
  fflush(stdout);
  up_mdelay(100);

  /* 7. Launch Option Byte reload (forces hardware reset to apply new boot address) */

  modifyreg32(STM32WL5_FLASH_CR, 0, FLASH_CR_OBL_LAUNCH);

  /* Wait for reset to occur */

  for (;;)
    {
      __asm__ volatile ("nop");
    }
}
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#if defined(CONFIG_STM32WL5_SPI1) || defined(CONFIG_STM32WL5_SPI2S2)
  stm32wl5_spidev_initialize();
#endif

#if defined(CONFIG_LCD_SSD1680) && !defined(CONFIG_VIDEO_FB)
  board_lcd_initialize();
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d\n",
             ret);
      return;
    }
#endif

#if defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_INPUT_BUTTONS_LOWER)
  /* Register the Button driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_FLASH_MOUNT)
  /* Register partition table for on-board FLASH memory */

  ret = stm32wl5_flash_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32wl5_flash_init() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_IPCC)
  /* Register IPCC driver */

  ret = ipcc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ipcc_init() failed\n");
    }
#endif

#if defined(CONFIG_MTD_M25P)
  /* Initialize external MT25Q NOR flash and partitions */

  ret = stm32wl5_mt25q_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32wl5_mt25q_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_ENABLE_CPU2)
  /* Ensure Option Bytes point CPU2 to 0x08032000 (Self-configuring) */

  stm32wl5_ensure_cpu2_boot_vector();

  /* Start second CPU (Cortex-M0+) */

  /* Ensure CPU2 peripheral clocks for Flash, IPCC, HSEM, and GPIOA/B/C are pre-enabled */
  modifyreg32(0x58000150, 0, (1 << 20) | (1 << 25) | (1 << 19)); /* C2AHB3ENR: IPCCEN | FLASHEN | HSEMEN */
  modifyreg32(0x5800014c, 0, (1 << 0) | (1 << 1) | (1 << 2));   /* C2AHB2ENR: GPIOA/B/C */

  printf("[SYSTEM] Booting Cortex-M0+ (CPU2) at 0x08032000 (M4: 200KB, M0+: 56KB)...\n");
  fflush(stdout);
  up_mdelay(10);
  stm32wl5_pwr_boot_c2();
#endif

#if defined(CONFIG_STM32WL5_SPI2S2) && defined(CONFIG_ADC_ADS7953)
  /* Initialize ADS7953 ADC1 and ADC2 on SPI2 */

  if (g_spi2 != NULL)
    {
      ret = ads7953_register("/dev/adc0", g_spi2, SPIDEV_USER(0));
      if (ret < 0)
        {
          printf("ERROR: ads7953_register(/dev/adc0) failed: %d\n", ret);
        }
      else
        {
          printf("ADS7953 ADC1 registered at /dev/adc0\n");
        }

      ret = ads7953_register("/dev/adc1", g_spi2, SPIDEV_USER(1));
      if (ret < 0)
        {
          printf("ERROR: ads7953_register(/dev/adc1) failed: %d\n", ret);
        }
      else
        {
          printf("ADS7953 ADC2 registered at /dev/adc1\n");
        }
    }
  else
    {
      printf("ERROR: g_spi2 is NULL\n");
    }
#endif

  UNUSED(ret);
}
#endif

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  return -ENOTTY;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32wl5_get_uniqueid(uniqueid);
  return OK;
}
#endif
