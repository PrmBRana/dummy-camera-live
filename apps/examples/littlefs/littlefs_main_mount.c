/****************************************************************************
 * apps/examples/littlefs/littlefs_main_mount.c
 *
 * Step 1: SPI Initialization Test
 * Target: STM32WL5 (Nucleo-WL55JC) - SPI1
 *
 ****************************************************************************/

/****************************************************************************
 * Overview
 *
 *   This file talks to the SPI NOR flash on SPI1 (MT25Q family), mounted
 *   as three LittleFS partitions:
 *
 *     /dev/hk1     -> /mnt/hk1     ADC1 housekeeping telemetry (34B/pkt)
 *     /dev/hk2     -> /mnt/hk2     ADC2 + IMU housekeeping telemetry (38B/pkt)
 *     /dev/camera  -> /mnt/camera  Captured camera frames
 *
 *   It has two roles:
 *
 *     1. Low-level flash bring-up / self-test helpers (init_spi,
 *        read_flash_id, read_flash_status, write/erase commands,
 *        littlefs_hardware_test) used to validate the SPI link and the
 *        flash chip before trusting it with real telemetry.
 *
 *     2. The `littlefs_telemetry_daemon`, which is the storage consumer
 *        for OBC_main.c's telemetry producer: it blocks on
 *        g_telemetry_sem, drains packets from the shared ring buffer
 *        (telemetry_rb_read), and appends each one to the matching
 *        LittleFS file, verifying every write with an immediate
 *        read-back.
 *
 *   `littlefs_main` is the NSH-callable entry point: with no arguments it
 *   runs the daemon; with "read"/"clean"/"format" [hk1|hk2|camera] it
 *   performs the corresponding one-shot maintenance operation instead.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>

#include <nuttx/spi/spi.h>
#include <telemetry_rb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_PORT          1          /* Using SPI1                        */
#define SPI_FREQUENCY     20000000   /* 20 MHz                            */

/* MT25Q-family SPI NOR flash opcodes used by the erase helpers below */

#define ERASE_4KB         0x20       /* 4KB Sector Erase command          */
#define ERASE_32KB        0x52       /* 32KB Block Erase command          */
#define ERASE_64KB        0xD8       /* 64KB Block Erase command          */
#define ERASE_ALL         0xC7       /* Chip Erase command (128MB erase)  */
#define Die_erase          0xC4      /* Die Erase command (64MB erase)    */

#define HK_Download_Cmd 53 01 1D D1 F2
/****************************************************************************
 * Forward Declarations
 ****************************************************************************/

/* Board-specific SPI bus initialization (implemented in board.c) */

FAR struct spi_dev_s *stm32wl5_spibus_initialize(int bus);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_spi
 *
 * Description:
 *   Initialize the given SPI bus and configure it for the flash chip:
 *   Mode 0, 8-bit frames, 20 MHz clock.
 ****************************************************************************/

static FAR struct spi_dev_s *init_spi(int bus)
{
  FAR struct spi_dev_s *spi;

  printf("Initializing SPI bus %d...\n", bus);

  /* 1. Initialize the SPI hardware bus */

  spi = stm32wl5_spibus_initialize(bus);
  if (spi == NULL)
    {
      printf("ERROR: stm32wl5_spibus_initialize(%d) returned NULL!\n", bus);
      return NULL;
    }

  /* 2. Configure bus parameters (Mode 0, 8 bits, 20 MHz) */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);        /* Mode 0: CPOL=0, CPHA=0 */
  SPI_SETBITS(spi, 8);                  /* 8-bit data frame */
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, SPI_FREQUENCY);  /* 20 MHz clock */
  SPI_LOCK(spi, false);

  printf("SPI%d initialized and configured successfully.\n", bus);
  return spi;
}

/****************************************************************************
 * Name: read_flash_id
 *
 * Description:
 *   Read and print the MT25Q flash's JEDEC ID (command 0x9F: manufacturer
 *   ID, memory type, capacity code).  Command 0x9E reads the same fields
 *   plus extended data, if ever needed.
 ****************************************************************************/

static int read_flash_id(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x9F; /* Read ID (JEDEC ID) command */
  uint8_t id[3] = {0};

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  /* Send the Read ID command */
  SPI_SEND(spi, cmd);

  /* Receive the ID bytes */
  SPI_RECVBLOCK(spi, id, sizeof(id));

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Manufacture ID: %02X, Memory Type: %02X, Capacity: %02X\n", id[0], id[1], id[2]);
  return OK;
}

/****************************************************************************
 * Name: read_flash_status
 *
 * Description:
 *   Read the flash's Status Register (command 0x05).  Bit 0 (WIP -
 *   "write in progress") is 0 when the flash is idle/ready and 1 while
 *   it is busy erasing or writing.
 ****************************************************************************/

static int read_flash_status(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x05; /* Read Status Register command */
  uint8_t status = 0;

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  /* Send the Read Status Register command */
  SPI_SEND(spi, cmd);

  /* Mode 0 only shifts a byte out in response to a byte sent in, so a
   * dummy byte must be clocked out to receive the status byte back.
   */
  status = SPI_SEND(spi, 0xAA);

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Read Flash Status Register: %02X\n", status);
  return status;
}

/****************************************************************************
 * Name: write_flash_enable
 *
 * Description:
 *   Send the Write Enable command (0x06).  Required before any
 *   program/erase command; the flash clears this latch automatically
 *   after the operation completes.
 ****************************************************************************/

static int write_flash_enable(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x06; /* Write Enable command */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  SPI_SEND(spi, cmd);

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Flash Write Enable command sent.\n");
  return OK;
}

/****************************************************************************
 * Name: write_flash_disable
 *
 * Description:
 *   Send the Write Disable command (0x04), clearing the write-enable
 *   latch set by write_flash_enable().
 ****************************************************************************/

static int write_flash_disable(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x04; /* Write Disable command */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  SPI_SEND(spi, cmd);

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Flash Write Disable command sent.\n");
  return OK;
}

/****************************************************************************
 * Name: erase_flash_4kb
 *
 * Description:
 *   Erase a single 4KB sector at `address` (command 0x20) and poll the
 *   status register until the operation completes or times out
 *   (~5 seconds).
 ****************************************************************************/

static int erase_flash_4kb(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[4];
  uint8_t status;
  int count = 0;
  const int max_retries_4kb = 500; /* 500 tries * 10ms = 5000 ms total */

  cmd[0] = ERASE_4KB;              /* 0x20: 4KB Sector Erase */
  cmd[1] = (address >> 16) & 0xFF; /* Address byte 1 (MSB) */
  cmd[2] = (address >> 8)  & 0xFF; /* Address byte 2 */
  cmd[3] = address & 0xFF;         /* Address byte 3 (LSB) */

  /* 1. Write Enable MUST be sent first, as its own transaction */
  write_flash_enable(spi);

  /* 2. Send the erase command and address together */
  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Flash Erase command sent for address: 0x%06lX\n", (unsigned long)address);

  /* 3. Poll the status register (WIP bit) until the erase finishes */
  printf("Waiting for erase to complete...\n");
  do
    {
      usleep(1000);  /* 1 ms so the CPU can breathe between polls */
      count++;
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (count < max_retries_4kb));

  if (count >= max_retries_4kb)
    {
      printf("RESULT: [FAIL] Erase timed out after %d retries!\n", count);
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 4KB Erase completed in %d iterations (~%d ms)!\n",
         count, count * 10);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_32kb
 *
 * Description:
 *   Erase a single 32KB block at `address` (command 0x52) and poll the
 *   status register until complete or timed out (~1 second).
 ****************************************************************************/

static int erase_flash_32kb(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[4];
  uint8_t status;
  int count = 0;
  const int max_retries_32kb = 1000; /* 1000 * 1ms = 1,000 ms (1 second) */

  cmd[0] = ERASE_32KB;             /* 0x52: 32KB Subsector Erase */
  cmd[1] = (address >> 16) & 0xFF;
  cmd[2] = (address >> 8)  & 0xFF;
  cmd[3] = address & 0xFF;

  write_flash_enable(spi);

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Erasing 32KB at address 0x%06lX...\n", (unsigned long)address);

  do
    {
      usleep(1000); /* 1ms */
      count++;
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (count < max_retries_32kb));

  if (count >= max_retries_32kb)
    {
      printf("RESULT: [FAIL] 32KB Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 32KB Erased in ~%d ms!\n", count * 10);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_64kb
 *
 * Description:
 *   Erase a single 64KB sector at `address` (command 0xD8) and poll the
 *   status register until complete or timed out (~2 seconds).
 ****************************************************************************/

static int erase_flash_64kb(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[4];
  uint8_t status;
  int count = 0;
  const int max_retries_64kb = 2000; /* 2000 * 1ms = 2,000 ms (2 seconds) */

  cmd[0] = ERASE_64KB;             /* 0xD8: 64KB Sector Erase */
  cmd[1] = (address >> 16) & 0xFF;
  cmd[2] = (address >> 8)  & 0xFF;
  cmd[3] = address & 0xFF;

  write_flash_enable(spi);

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Erasing 64KB sector at address 0x%06lX...\n", (unsigned long)address);

  do
    {
      usleep(10000); /* 10ms */
      count++;
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (count < max_retries_64kb));

  if (count >= max_retries_64kb)
    {
      printf("RESULT: [FAIL] 64KB Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 64KB Erased in ~%d ms!\n", count * 10);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_all
 *
 * Description:
 *   Erase the entire 128MB flash chip (command 0xC7, no address bytes).
 *   Takes several minutes; polls the status register with a heartbeat
 *   dot printed once per second, up to an ~8 minute timeout.
 ****************************************************************************/

static int erase_flash_all(FAR struct spi_dev_s *spi)
{
  uint8_t status;
  int seconds = 0;
  const int max_seconds_all = 500; /* ~8 minutes timeout */

  printf("WARNING: Erasing ENTIRE 128MB Flash Chip (takes ~2-4 minutes)...\n");

  write_flash_enable(spi);

  /* Send ONLY the opcode: chip erase takes no address bytes */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SEND(spi, ERASE_ALL);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  do
    {
      sleep(1); /* 1 second */
      seconds++;
      printf(".");
      fflush(stdout);
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (seconds < max_seconds_all));

  printf("\n");

  if (seconds >= max_seconds_all)
    {
      printf("RESULT: [FAIL] Bulk Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] Entire 128MB Chip Erased in %d seconds!\n", seconds);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_die
 *
 * Description:
 *   Erase a single 64MB die at `address` (command 0xC4).  Polls the
 *   status register with a heartbeat dot printed once per second, up to
 *   an ~5 minute timeout.
 ****************************************************************************/

static int erase_flash_die(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[4];
  uint8_t status;
  int seconds = 0;
  const int max_seconds_die = 300; /* 5 minutes timeout */

  cmd[0] = Die_erase;              /* 0xC4: Die Erase */
  cmd[1] = (address >> 16) & 0xFF;
  cmd[2] = (address >> 8)  & 0xFF;
  cmd[3] = address & 0xFF;

  write_flash_enable(spi);

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Erasing 64MB Die for address 0x%06lX (takes ~1-2 minutes)...\n",
         (unsigned long)address);

  do
    {
      sleep(1); /* 1 second */
      seconds++;
      printf(".");
      fflush(stdout);
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (seconds < max_seconds_die));

  printf("\n");

  if (seconds >= max_seconds_die)
    {
      printf("RESULT: [FAIL] Die Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 64MB Die Erased in %d seconds!\n", seconds);
  return OK;
}

/****************************************************************************
 * Name: mount_drive
 *
 * Description:
 *   Mount a LittleFS partition, creating the mount point directory if
 *   needed.  Tries "autoformat" first (mounts an existing filesystem, or
 *   formats only if the partition is blank); if that fails for a reason
 *   other than "already mounted", falls back to "forceformat" to
 *   guarantee a clean, mountable filesystem.
 *
 * Returned Value:
 *   OK on success (including "already mounted"), negative errno on
 *   failure.
 ****************************************************************************/

static int mount_drive(FAR const char *dev_path, FAR const char *mount_path)
{
  struct stat st;
  int ret;

  printf("Mounting drive %s to %s...\n", dev_path, mount_path);

  /* 1. Create the mount directory if it doesn't already exist */
  if (stat(mount_path, &st) != 0)
    {
      mkdir(mount_path, 0777);
    }

  /* 2. Try autoformat mount first */
  ret = mount(dev_path, mount_path, "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      if (errno == EBUSY || errno == ENOTDIR || errno == EEXIST)
        {
          /* Already mounted: perfectly normal, not an error */
          return OK;
        }

      printf("WARN: autoformat failed (errno=%d), trying forceformat...\n", errno);

      /* If unformatted, forceformat will format the partition cleanly */
      ret = mount(dev_path, mount_path, "littlefs", 0, "forceformat");
      if (ret < 0)
        {
          if (errno == EBUSY || errno == ENOTDIR || errno == EEXIST)
            {
              return OK;
            }

          printf("ERROR: mount failed (ret=%d, errno=%d)\n", ret, errno);
          return ret;
        }
    }

  printf("RESULT: [PASS] Drive %s mounted at %s!\n", dev_path, mount_path);
  return OK;
}

/****************************************************************************
 * Name: unmount_drive
 *
 * Description:
 *   Safely flush and unmount the LittleFS filesystem at `mount_path`.
 ****************************************************************************/

static int unmount_drive(FAR const char *mount_path)
{
  int ret;

  printf("Unmounting %s...\n", mount_path);

  ret = umount(mount_path);
  if (ret < 0)
    {
      printf("ERROR: umount() failed (ret=%d, errno=%d)\n", ret, errno);
      return ret;
    }

  printf("RESULT: [PASS] Drive at %s safely unmounted!\n", mount_path);
  return OK;
}

static int clean_one_partition(FAR const char *dev_path, FAR const char *mount_path);

/****************************************************************************
 * Name: format_one_partition
 *
 * Description:
 *   Unconditionally reformat one partition ("forceformat"), discarding
 *   any existing contents. If partition is in use by telemetry daemon,
 *   falls back to wiping all files on the partition.
 ****************************************************************************/

static int format_one_partition(FAR const char *dev_path, FAR const char *mount_path)
{
  printf("[LITTLEFS] Formatting %s at %s with 256B page geometry...\n", dev_path, mount_path);
  umount(mount_path);

  int ret = mount(dev_path, mount_path, "littlefs", 0, "forceformat");
  if (ret < 0)
    {
      if (errno == EBUSY)
        {
          printf("[LITTLEFS] Partition %s is active (storage daemon running).\n", mount_path);
          printf("[LITTLEFS] Erasing all files on %s instead...\n", mount_path);
          return clean_one_partition(dev_path, mount_path);
        }

      printf("[LITTLEFS] ERROR: Failed to format %s (ret=%d, errno=%d)\n", dev_path, ret, errno);
      return ret;
    }

  printf("[LITTLEFS] [PASS] %s successfully formatted and mounted at %s!\n", dev_path, mount_path);
  return OK;
}

/****************************************************************************
 * Name: clean_one_partition
 *
 * Description:
 *   Mount one partition (if not already mounted) and delete every file
 *   in its root directory, without reformatting the filesystem itself.
 ****************************************************************************/

static int clean_one_partition(FAR const char *dev_path, FAR const char *mount_path)
{
  mount_drive(dev_path, mount_path);

  DIR *d = opendir(mount_path);
  if (d)
    {
      struct dirent *de;
      char filepath[64];
      int count = 0;

      while ((de = readdir(d)) != NULL)
        {
          if (strcmp(de->d_name, ".") == 0 || strcmp(de->d_name, "..") == 0)
            {
              continue;
            }

          snprintf(filepath, sizeof(filepath), "%s/%s", mount_path, de->d_name);
          unlink(filepath);
          count++;
        }

      closedir(d);
      printf("[LITTLEFS] [PASS] Cleaned %s: removed %d file(s).\n", mount_path, count);
    }
  else
    {
      printf("[LITTLEFS] Could not open %s (errno=%d)\n", mount_path, errno);
      return -errno;
    }

  return OK;
}

/****************************************************************************
 * Name: write_file
 *
 * Description:
 *   Write `len` bytes from `data` to `filepath`, truncating/creating as
 *   needed, and fsync() before closing so the data is flushed from the
 *   RAM cache to the physical SPI NOR flash.
 ****************************************************************************/

static int write_file(FAR const char *filepath, FAR const void *data, size_t len)
{
  int fd;
  ssize_t nwritten;

  printf("Opening %s to write %zu bytes...\n", filepath, len);

  fd = open(filepath, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0)
    {
      printf("ERROR: open() failed for writing (errno=%d)\n", errno);
      return -errno;
    }

  nwritten = write(fd, data, len);
  if (nwritten != len)
    {
      printf("ERROR: write() failed! (wrote %zd / %zu bytes)\n", nwritten, len);
      close(fd);
      return -EIO;
    }

  fsync(fd);
  close(fd);

  printf("RESULT: [PASS] Successfully wrote %zd bytes to %s!\n", nwritten, filepath);
  return OK;
}

/****************************************************************************
 * Name: read_file
 *
 * Description:
 *   Read up to `max_len` bytes from `filepath` into `buffer`.
 *
 * Returned Value:
 *   Number of bytes read (>= 0) on success, negative errno on failure.
 ****************************************************************************/

static int read_file(FAR const char *filepath, FAR void *buffer, size_t max_len)
{
  int fd;
  ssize_t nread;

  printf("Opening %s to read...\n", filepath);

  fd = open(filepath, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: open() failed for reading (errno=%d)\n", errno);
      return -errno;
    }

  nread = read(fd, buffer, max_len);
  close(fd);

  if (nread < 0)
    {
      printf("ERROR: read() failed (errno=%d)\n", errno);
      return -errno;
    }

  printf("RESULT: [PASS] Successfully read %zd bytes from %s!\n", nread, filepath);
  return (int)nread;
}

/****************************************************************************
 * Name: littlefs_hardware_test
 *
 * Description:
 *   One-shot bench self-test: brings up SPI1, reads the flash JEDEC ID,
 *   exercises the status/write-enable/write-disable commands, then
 *   mounts each die, writes a small test file, reads it back, and
 *   unmounts.  Not part of the normal flight data path; useful when
 *   bringing up new hardware or verifying a flash chip is alive.
 ****************************************************************************/

static int littlefs_hardware_test(void)
{
  FAR struct spi_dev_s *spi;
  int ret;
  uint8_t status_R;
  uint8_t status_W;
  char read_buf[128];

  printf("\n");
  printf("=========================================\n");
  printf(" STEP 1: Test SPI Initialization\n");
  printf("=========================================\n");

  /* Call the SPI initialization function */

  spi = init_spi(SPI_PORT);
  if (spi == NULL)
    {
      printf("RESULT: [FAIL] SPI initialization failed!\n\n");
      return EXIT_FAILURE;
    }

  printf("RESULT: [PASS] SPI%d is ready for communication.\n", SPI_PORT);
  printf("=========================================\n\n");

  ret = read_flash_id(spi);
  if (ret != OK)
    {
      printf("RESULT: [FAIL] Failed to read Flash ID!\n\n");
      return EXIT_FAILURE;
    }
  printf("RESULT: Flash ID read successfully.\n");

  /* Check whether the flash is busy or idle before touching it */

  printf("First Reading Flash Status Register...\n");
  status_R = read_flash_status(spi);
  if (status_R & 0x01)
    {
      printf("RESULT: Flash is busy (Status Register: %02X)\n", status_R);
    }
  else
    {
      printf("RESULT: Flash is ready/idle (Status Register: %02X)\n", status_R);
    }

  /* Exercise the write-enable latch and confirm it toggles as expected */

  printf("Enabling Flash Write Operation...\n");
  write_flash_enable(spi);
  status_W = read_flash_status(spi);
  if (status_W & 0x02)
    {
      printf("RESULT: Enabled (Status Register: %02X)\n", status_W);
    }
  else
    {
      printf("RESULT: Disabled (Status Register: %02X)\n", status_W);
    }

  printf("Disabling Flash Write Operation...\n");
  write_flash_disable(spi);
  status_W = read_flash_status(spi);
  if (status_W & 0x02)
    {
      printf("RESULT: Enabled (Status Register: %02X)\n", status_W);
    }
  else
    {
      printf("RESULT: Disabled (Status Register: %02X)\n", status_W);
    }

  /* Sector-erase test is disabled by default; uncomment to exercise it.
   *
   *   printf("Starting 64KB Sector Erase Test...\n");
   *   ret = erase_flash_64kb(spi, 0x020000);
   *   if (ret != OK)
   *     {
   *       printf("RESULT: [FAIL] 64KB Sector Erase failed!\n\n");
   *       return EXIT_FAILURE;
   *     }
   */

  printf("Starting write dummy data to LittleFS test...\n");
  printf("Mounting LittleFS...\n");

  /* ======================================================================= */
  /* PART 1: TEST DIE 0 (Housekeeping - /dev/hk)                             */
  /* ======================================================================= */

  printf("\n--- Testing Die 0 (Housekeeping) ---\n");

  ret = mount_drive("/dev/hk", "/mnt/hk");
  if (ret != OK)
    {
      printf("Failed to mount /dev/hk!\n");
      return EXIT_FAILURE;
    }

  /* Die 0 write/read-back exercise is currently disabled; uncomment to
   * write and verify a sample telemetry log line, then unmount:
   *
   *   const char *hk_data = "HK_LOG: Batt=4.1V, Temp=22C, SolarCurrent=1.2A, Status=NOMINAL";
   *   write_file("/mnt/hk/telemetry.log", hk_data, strlen(hk_data));
   *   memset(read_buf, 0, sizeof(read_buf));
   *   read_file("/mnt/hk/telemetry.log", read_buf, sizeof(read_buf) - 1);
   *   printf("Die 0 Data Read: \"%s\"\n\n", read_buf);
   *   unmount_drive("/mnt/hk");
   */

  /* ======================================================================= */
  /* PART 2: TEST DIE 1 (Camera - /dev/camera)                               */
  /* ======================================================================= */

  printf("\n--- Testing Die 1 (Camera Images) ---\n");

  /* 1. Mount Die 1 to /mnt/camera */
  ret = mount_drive("/dev/camera", "/mnt/camera");
  if (ret != OK)
    {
      printf("Failed to mount /dev/camera!\n");
      return EXIT_FAILURE;
    }

  /* 2. Write a dummy image file to Die 1 */
  const char *cam_data = "CAMERA_RAW_IMAGE: Width=640, Height=480, Format=RGB565, Frame=1";
  write_file("/mnt/camera/frame001.raw", cam_data, strlen(cam_data));

  /* 3. Read it back from Die 1 */
  memset(read_buf, 0, sizeof(read_buf));
  read_file("/mnt/camera/frame001.raw", read_buf, sizeof(read_buf) - 1);
  printf("Die 1 Data Read: \"%s\"\n\n", read_buf);

  /* 4. Unmount Die 1 */
  unmount_drive("/mnt/camera");

  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: littlefs_telemetry_daemon
 *
 * Description:
 *   Storage consumer for OBC_main's telemetry producer.  Mounts both
 *   housekeeping partitions, then loops forever:
 *
 *     1. Sleep on g_telemetry_sem (0% CPU while idle; posted once per
 *        packet by send_telemetry_to_ringbuffer() in OBC_main).
 *     2. Drain every pending packet from the shared ring buffer.
 *     3. Route each packet by its footer value to the matching file
 *        (ADC1 -> /mnt/hk1/telemetry.bin, ADC2+IMU -> /mnt/hk2/telemetry.bin),
 *        append it, fsync, then read the same bytes back from flash and
 *        compare against what was written as an integrity check.
 *     4. On a write() failure that looks like a stale/old flash format
 *        (EFAULT), automatically force-reformat the partition and retry
 *        the append once.
 ****************************************************************************/

static int littlefs_telemetry_daemon(void)
{
  struct telemetry_envelope_s env;
  struct adc1_packet_s verify_adc1;
  struct adc2_packet_s verify_adc2;
  struct stat st;
  const char *filename;
  int fd;
  ssize_t written;
  ssize_t nread;
  int ret;
  uint32_t packet_num = 0;

  printf("\n=======================================================\n");
  printf("  [LITTLEFS DAEMON] Telemetry Storage Consumer Active  \n");
  printf("  Mount 1: /dev/hk1 (32 MB) -> /mnt/hk1 (ADC1, Footer 0xAA55, 34B)\n");
  printf("  Mount 2: /dev/hk2 (32 MB) -> /mnt/hk2 (ADC2, Footer 0xBB66, 38B)\n");
  printf("=======================================================\n");

  /* 1. Mount Partition 1: /dev/hk1 (32 MB) to /mnt/hk1 */
  ret = mount_drive("/dev/hk1", "/mnt/hk1");
  if (ret != OK)
    {
      printf("[LITTLEFS DAEMON] ERROR: mount_drive(/dev/hk1) failed!\n");
      return ret;
    }

  /* 2. Mount Partition 2: /dev/hk2 (32 MB) to /mnt/hk2 */
  ret = mount_drive("/dev/hk2", "/mnt/hk2");
  if (ret != OK)
    {
      printf("[LITTLEFS DAEMON] ERROR: mount_drive(/dev/hk2) failed!\n");
      return ret;
    }

  printf("[LITTLEFS DAEMON] Ready! Both /mnt/hk1 and /mnt/hk2 mounted successfully!\n");
  printf("[LITTLEFS DAEMON] Waiting for OBC_main telemetry signals...\n");

  while (1)
    {
      /* 3. Sleep until notified by OBC_main (0% CPU while waiting) */
      sem_wait(&g_telemetry_sem);

      /* 4. Pop all pending packets from the ring buffer */
      while (telemetry_rb_read(&env) == 0)
        {
          packet_num++;

          /* ================================================================= */
          /* ROUTE 1: Footer 0xAA55 -> /mnt/hk1/telemetry.bin (ADC 1: 34 Bytes) */
          /* ================================================================= */

          if (env.footer == TELEM_FOOTER_ADC1)
            {
              filename = "/mnt/hk1/telemetry.bin";
              printf("\n[LITTLEFS DAEMON] ===> RECEIVED ADC1 ENVELOPE (Len: %d B, Footer: 0x%04X) <===\n",
                     env.len, env.footer);

              fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
              if (fd >= 0)
                {
                  written = write(fd, &env.pkt.adc1, sizeof(struct adc1_packet_s));
                  fsync(fd);
                  close(fd);

                  if (written < 0)
                    {
                      printf("[LITTLEFS DAEMON] ERROR: write() failed on %s (written=%zd, errno=%d)\n",
                             filename, written, errno);
                      if (errno == EFAULT)
                        {
                          printf("[LITTLEFS DAEMON] Old flash format detected (EFAULT). Auto-formatting /dev/hk1 with 256B geometry...\n");
                          umount("/mnt/hk1");
                          mount("/dev/hk1", "/mnt/hk1", "littlefs", 0, "forceformat");
                          fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
                          if (fd >= 0)
                            {
                              written = write(fd, &env.pkt.adc1, sizeof(struct adc1_packet_s));
                              fsync(fd);
                              close(fd);
                              printf("[LITTLEFS DAEMON] [RECOVERED HK1] Appended %zd Bytes to %s\n", written, filename);
                            }
                        }
                      continue;
                    }

                  printf("[LITTLEFS DAEMON] [STORED HK1] Appended %zd Bytes to %s\n", written, filename);

                  /* Read back and verify from physical Flash */
                  fd = open(filename, O_RDONLY);
                  if (fd >= 0)
                    {
                      lseek(fd, -(off_t)sizeof(struct adc1_packet_s), SEEK_END);
                      memset(&verify_adc1, 0, sizeof(verify_adc1));
                      nread = read(fd, &verify_adc1, sizeof(struct adc1_packet_s));
                      close(fd);

                      if (nread == sizeof(struct adc1_packet_s) &&
                          verify_adc1.footer == TELEM_FOOTER_ADC1 &&
                          memcmp(&env.pkt.adc1, &verify_adc1, sizeof(struct adc1_packet_s)) == 0)
                        {
                          static const char * const adc1_names[ADC1_CHANNELS_COUNT] =
                          {
                            "ADC_BAT_MON", "TOTAL_SOLAR_V", "RAW_VOLT", "SP5_VOLT",
                            "SP4_VOLT",    "SP3_VOLT",      "SP1_VOLT", "SP2_VOLT",
                            "ANT_TEMP",    "BATT_TEMP",     "TEMP_BPB", "TEMP1",
                            "TEMP5",       "TEMP4",         "TEMP3",    "TEMP2"
                          };

                          stat(filename, &st);
                          printf("[LITTLEFS DAEMON] [VERIFIED IN FLASH /mnt/hk1 - ALL 16 CHANNELS]:\n");
                          for (int k = 0; k < ADC1_CHANNELS_COUNT; k++)
                            {
                              const char *unit = (k < 8) ? "V" : "C";
                              printf("  [%02d] %-14s : %6.2f %s   (x100: %6d)\n",
                                     k, adc1_names[k], (float)verify_adc1.data[k] / 100.0f,
                                     unit, verify_adc1.data[k]);
                            }
                          printf("  [--] Footer Check    : 0x%04X   [MATCHED 100%%]\n", verify_adc1.footer);
                          printf("[LITTLEFS DAEMON] [PASS HK1] File Size: %ld B (%ld Packets in /mnt/hk1)\n",
                                 (long)st.st_size, (long)(st.st_size / sizeof(struct adc1_packet_s)));
                        }
                      else
                        {
                          printf("[LITTLEFS DAEMON] [FAIL] Read-back mismatch in %s!\n", filename);
                        }
                    }
                  else
                    {
                      printf("[LITTLEFS DAEMON] ERROR opening %s for read (errno=%d)\n", filename, errno);
                    }
                }
              else
                {
                  printf("[LITTLEFS DAEMON] ERROR opening %s for write (errno=%d)\n", filename, errno);
                }
            }

          /* ================================================================= */
          /* ROUTE 2: Footer 0xBB66 -> /mnt/hk2/telemetry.bin (ADC 2: 38 Bytes) */
          /* ================================================================= */

          else if (env.footer == TELEM_FOOTER_ADC2)
            {
              filename = "/mnt/hk2/telemetry.bin";
              printf("\n[LITTLEFS DAEMON] ===> RECEIVED ADC2 ENVELOPE (Len: %d B, Footer: 0x%04X) <===\n",
                     env.len, env.footer);

              fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
              if (fd >= 0)
                {
                  written = write(fd, &env.pkt.adc2, sizeof(struct adc2_packet_s));
                  fsync(fd);
                  close(fd);

                  if (written < 0)
                    {
                      printf("[LITTLEFS DAEMON] ERROR: write() failed on %s (written=%zd, errno=%d)\n",
                             filename, written, errno);
                      if (errno == EFAULT)
                        {
                          printf("[LITTLEFS DAEMON] Old flash format detected (EFAULT). Auto-formatting /dev/hk2 with 256B geometry...\n");
                          umount("/mnt/hk2");
                          mount("/dev/hk2", "/mnt/hk2", "littlefs", 0, "forceformat");
                          fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
                          if (fd >= 0)
                            {
                              written = write(fd, &env.pkt.adc2, sizeof(struct adc2_packet_s));
                              fsync(fd);
                              close(fd);
                              printf("[LITTLEFS DAEMON] [RECOVERED HK2] Appended %zd Bytes to %s\n", written, filename);
                            }
                        }
                      continue;
                    }

                  printf("[LITTLEFS DAEMON] [STORED HK2] Appended %zd Bytes to %s\n", written, filename);

                  /* Read back and verify from physical Flash */
                  fd = open(filename, O_RDONLY);
                  if (fd >= 0)
                    {
                      lseek(fd, -(off_t)sizeof(struct adc2_packet_s), SEEK_END);
                      memset(&verify_adc2, 0, sizeof(verify_adc2));
                      nread = read(fd, &verify_adc2, sizeof(struct adc2_packet_s));
                      close(fd);

                      if (nread == sizeof(struct adc2_packet_s) &&
                          verify_adc2.footer == TELEM_FOOTER_ADC2 &&
                          memcmp(&env.pkt.adc2, &verify_adc2, sizeof(struct adc2_packet_s)) == 0)
                        {
                          static const char * const adc2_names[ADC2_ACTIVE_CHANNELS] =
                          {
                            "UNREG_I",       "SP4_I",         "MAIN_3V3_I", "MISSION_3V3_I",
                            "SPT_I",         "5V_I",          "SP2_I",      "SP1_I",
                            "RAW_I",         "SP5_I",         "BAT_I",      "SP3_I"
                          };

                          stat(filename, &st);
                          printf("[LITTLEFS DAEMON] [VERIFIED IN FLASH /mnt/hk2 - ALL 12 CHANNELS + IMU]:\n");
                          for (int k = 0; k < ADC2_ACTIVE_CHANNELS; k++)
                            {
                              printf("  [%02d] %-14s : %6.2f A   (x100: %6d)\n",
                                     k, adc2_names[k], (float)verify_adc2.data[k] / 100.0f,
                                     verify_adc2.data[k]);
                            }
                          printf("  [GYRO] X: %6.2f dps | Y: %6.2f dps | Z: %6.2f dps (x100: %d, %d, %d)\n",
                                 (float)verify_adc2.gyro_x / 100.0f,
                                 (float)verify_adc2.gyro_y / 100.0f,
                                 (float)verify_adc2.gyro_z / 100.0f,
                                 verify_adc2.gyro_x, verify_adc2.gyro_y, verify_adc2.gyro_z);
                          printf("  [MAG]  X: %6.3f G   | Y: %6.3f G   | Z: %6.3f G   (x100: %d, %d, %d)\n",
                                 (float)verify_adc2.mag_x / 100.0f,
                                 (float)verify_adc2.mag_y / 100.0f,
                                 (float)verify_adc2.mag_z / 100.0f,
                                 verify_adc2.mag_x, verify_adc2.mag_y, verify_adc2.mag_z);
                          printf("  [--] Footer Check    : 0x%04X   [MATCHED 100%%]\n", verify_adc2.footer);
                          printf("[LITTLEFS DAEMON] [PASS HK2] File Size: %ld B (%ld Packets in /mnt/hk2)\n",
                                 (long)st.st_size, (long)(st.st_size / sizeof(struct adc2_packet_s)));
                        }
                      else
                        {
                          printf("[LITTLEFS DAEMON] [FAIL] Read-back mismatch in %s! (nread=%zd, footer=0x%04X vs 0x%04X, memcmp=%d)\n",
                                 filename, nread, verify_adc2.footer, TELEM_FOOTER_ADC2,
                                 memcmp(&env.pkt.adc2, &verify_adc2, sizeof(struct adc2_packet_s)));
                        }
                    }
                  else
                    {
                      printf("[LITTLEFS DAEMON] ERROR opening %s for read (errno=%d)\n", filename, errno);
                    }
                }
              else
                {
                  printf("[LITTLEFS DAEMON] ERROR opening %s for write (errno=%d)\n", filename, errno);
                }
            }
        }

      printf("[LITTLEFS DAEMON] All packets stored and verified. Sleeping...\n\n");
    }

  return 0;
}

/****************************************************************************
 * Name: littlefs_read_hk1
 *
 * Description:
 *   Mount /dev/hk1 if needed and print every stored ADC1 telemetry
 *   record in /mnt/hk1/telemetry.bin.
 ****************************************************************************/

static int littlefs_read_hk1(void)
{
  struct adc1_packet_s pkt;
  static const char * const names[ADC1_CHANNELS_COUNT] =
  {
    "ADC_BAT_MON", "TOTAL_SOLAR_V", "RAW_VOLT", "SP5_VOLT",
    "SP4_VOLT",    "SP3_VOLT",      "SP1_VOLT", "SP2_VOLT",
    "ANT_TEMP",    "BATT_TEMP",     "TEMP_BPB", "TEMP1",
    "TEMP5",       "TEMP4",         "TEMP3",    "TEMP2"
  };

  /* Auto-mount /dev/hk1 if not already mounted */
  mount_drive("/dev/hk1", "/mnt/hk1");

  int fd = open("/mnt/hk1/telemetry.bin", O_RDONLY);
  if (fd < 0)
    {
      if (errno == ENOENT)
        {
          printf("[INFO] /mnt/hk1/telemetry.bin does not exist yet.\n");
          printf("[INFO] Please run 'launcher' first to record telemetry packets!\n");
        }
      else
        {
          printf("ERROR: Cannot open /mnt/hk1/telemetry.bin (errno=%d)\n", errno);
        }
      return -errno;
    }

  uint32_t count = 0;
  ssize_t nread;
  printf("\n======================= READING /mnt/hk1/telemetry.bin =======================\n");
  while ((nread = read(fd, &pkt, sizeof(pkt))) == sizeof(pkt))
    {
      count++;
      printf("\n>>> HK1 Telemetry Record #%lu (34 Bytes, Footer: 0x%04X) <<<\n",
             (unsigned long)count, pkt.footer);
      for (int i = 0; i < ADC1_CHANNELS_COUNT; i++)
        {
          const char *unit = (i < 8) ? "V" : "C";
          printf("  [%02d] %-15s : %6.2f %s   (Raw x100: %6d)\n",
                 i, names[i], (float)pkt.data[i] / 100.0f, unit, pkt.data[i]);
        }
    }

  close(fd);
  printf("\n[TOTAL] Read %lu HK1 records from /mnt/hk1/telemetry.bin\n", (unsigned long)count);
  return 0;
}

/****************************************************************************
 * Name: littlefs_read_hk2
 *
 * Description:
 *   Mount /dev/hk2 if needed and print every stored ADC2+IMU telemetry
 *   record in /mnt/hk2/telemetry.bin.
 ****************************************************************************/

static int littlefs_read_hk2(void)
{
  struct adc2_packet_s pkt;
  static const char * const names[ADC2_ACTIVE_CHANNELS] =
  {
    "UNREG_I",       "SP4_I",         "MAIN_3V3_I", "MISSION_3V3_I",
    "SPT_I",         "5V_I",          "SP2_I",      "SP1_I",
    "RAW_I",         "SP5_I",         "BAT_I",      "SP3_I"
  };

  /* Auto-mount /dev/hk2 if not already mounted */
  mount_drive("/dev/hk2", "/mnt/hk2");

  int fd = open("/mnt/hk2/telemetry.bin", O_RDONLY);
  if (fd < 0)
    {
      if (errno == ENOENT)
        {
          printf("[INFO] /mnt/hk2/telemetry.bin does not exist yet.\n");
          printf("[INFO] Please run 'launcher' first to record telemetry packets!\n");
        }
      else
        {
          printf("ERROR: Cannot open /mnt/hk2/telemetry.bin (errno=%d)\n", errno);
        }
      return -errno;
    }

  uint32_t count = 0;
  ssize_t nread;
  printf("\n======================= READING /mnt/hk2/telemetry.bin =======================\n");
  while ((nread = read(fd, &pkt, sizeof(pkt))) == sizeof(pkt))
    {
      count++;
      printf("\n>>> HK2 Telemetry Record #%lu (38 Bytes, Footer: 0x%04X) <<<\n",
             (unsigned long)count, pkt.footer);
      for (int i = 0; i < ADC2_ACTIVE_CHANNELS; i++)
        {
          printf("  [%02d] %-15s : %6.2f A   (Raw x100: %6d)\n",
                 i, names[i], (float)pkt.data[i] / 100.0f, pkt.data[i]);
        }
      printf("  [GYRO] X: %6.2f dps | Y: %6.2f dps | Z: %6.2f dps   (x100: %d, %d, %d)\n",
             (float)pkt.gyro_x / 100.0f, (float)pkt.gyro_y / 100.0f, (float)pkt.gyro_z / 100.0f,
             pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);
      printf("  [MAG]  X: %6.3f G   | Y: %6.3f G   | Z: %6.3f G   (x100: %d, %d, %d)\n",
             (float)pkt.mag_x / 100.0f, (float)pkt.mag_y / 100.0f, (float)pkt.mag_z / 100.0f,
             pkt.mag_x, pkt.mag_y, pkt.mag_z);
    }

  close(fd);
  printf("\n[TOTAL] Read %lu HK2 records from /mnt/hk2/telemetry.bin\n", (unsigned long)count);
  return 0;
}
/*sends the HK1, HK2 and IMU data brust if command received*/
static void __attribute__((unused)) send_brust_HK(uint32_t start_address, uint16_t packet_count)
{
  (void)start_address;
  (void)packet_count;
}

/****************************************************************************
 * Name: print_partition_sizes
 *
 * Description:
 *   Display MT25Q NOR flash partition table, capacity, and current file sizes.
 ****************************************************************************/

static int print_partition_sizes(void)
{
  struct stat st;
  printf("\n=================================================================\n");
  printf("  MT25Q SPI NOR FLASH PARTITION TABLE & CURRENT SIZES\n");
  printf("  Total Flash Capacity: 1 Gbit / 128 MB (Die 0: 64MB, Die 1: 64MB)\n");
  printf("=================================================================\n");

  /* Auto-mount partitions to inspect sizes */
  mount_drive("/dev/hk1", "/mnt/hk1");
  mount_drive("/dev/hk2", "/mnt/hk2");
  mount_drive("/dev/camera", "/mnt/camera");

  printf("  Partition 1: /dev/hk1    (32 MB - Die 0) -> Mount: /mnt/hk1\n");
  if (stat("/mnt/hk1/telemetry.bin", &st) == 0)
    {
      printf("    File: /mnt/hk1/telemetry.bin | Size: %ld Bytes (%ld Packets, 34B/pkt)\n",
             (long)st.st_size, (long)(st.st_size / sizeof(struct adc1_packet_s)));
    }
  else
    {
      printf("    File: /mnt/hk1/telemetry.bin | Size: 0 Bytes (empty / not recorded yet)\n");
    }

  printf("  Partition 2: /dev/hk2    (32 MB - Die 0) -> Mount: /mnt/hk2\n");
  if (stat("/mnt/hk2/telemetry.bin", &st) == 0)
    {
      printf("    File: /mnt/hk2/telemetry.bin | Size: %ld Bytes (%ld Packets, 38B/pkt)\n",
             (long)st.st_size, (long)(st.st_size / sizeof(struct adc2_packet_s)));
    }
  else
    {
      printf("    File: /mnt/hk2/telemetry.bin | Size: 0 Bytes (empty / not recorded yet)\n");
    }

  printf("  Partition 3: /dev/camera (64 MB - Die 1) -> Mount: /mnt/camera\n");
  if (stat("/mnt/camera/capture_01.raw", &st) == 0)
    {
      printf("    File: /mnt/camera/capture_01.raw | Size: %ld Bytes\n", (long)st.st_size);
    }
  else
    {
      printf("    File: /mnt/camera/capture_01.raw | Size: 0 Bytes (empty / not captured yet)\n");
    }
  printf("=================================================================\n\n");
  return 0;
}

/****************************************************************************
 * Name: littlefs_erase_cmd
 *
 * Description:
 *   Erase partition filesystem or trigger low-level flash sector/block erase.
 ****************************************************************************/

static int littlefs_erase_cmd(int argc, FAR char *argv[])
{
  if (argc < 3)
    {
      printf("[LITTLEFS] No partition specified. Erasing/cleaning all partitions (HK1, HK2, Camera)...\n");
      format_one_partition("/dev/hk1", "/mnt/hk1");
      format_one_partition("/dev/hk2", "/mnt/hk2");
      format_one_partition("/dev/camera", "/mnt/camera");
      printf("\n[TIP] You can also target specific partitions:\n");
      printf("  littlefs erase hk1     - Erase/format HK1 partition\n");
      printf("  littlefs erase hk2     - Erase/format HK2 partition\n");
      printf("  littlefs erase camera  - Erase/format Camera partition\n");
      printf("  littlefs erase all     - Full 128MB chip erase\n\n");
      return 0;
    }

  if (strcmp(argv[2], "hk1") == 0)
    {
      return format_one_partition("/dev/hk1", "/mnt/hk1");
    }
  else if (strcmp(argv[2], "hk2") == 0)
    {
      return format_one_partition("/dev/hk2", "/mnt/hk2");
    }
  else if (strcmp(argv[2], "camera") == 0 || strcmp(argv[2], "cam") == 0)
    {
      return format_one_partition("/dev/camera", "/mnt/camera");
    }
  else
    {
      FAR struct spi_dev_s *spi = init_spi(SPI_PORT);
      if (spi == NULL)
        {
          printf("ERROR: Failed to initialize SPI%d for flash erase!\n", SPI_PORT);
          return -EIO;
        }

      if (strcmp(argv[2], "all") == 0 || strcmp(argv[2], "chip") == 0)
        {
          return erase_flash_all(spi);
        }
      else if (strcmp(argv[2], "die") == 0)
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_die(spi, addr);
        }
      else if (strcmp(argv[2], "64kb") == 0)
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_64kb(spi, addr);
        }
      else if (strcmp(argv[2], "32kb") == 0)
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_32kb(spi, addr);
        }
      else if (strcmp(argv[2], "4kb") == 0)
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_4kb(spi, addr);
        }
      else
        {
          printf("Unknown erase target: %s\n", argv[2]);
          return -EINVAL;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: littlefs_main
 *,
 * Description:
 *   NSH entry point.
 *
 *     littlefs                         -> run telemetry storage daemon
 *     littlefs read   [hk1|hk2]        -> dump stored telemetry records
 *     littlefs clean  [hk1|hk2|camera] -> delete files, keep format
 *     littlefs format [hk1|hk2|camera] -> force-reformat LittleFS partition
 *     littlefs erase  [hk1|hk2|camera|chip|die|4kb|32kb|64kb] -> flash erase
 *     littlefs size   [partition/info] -> show partition table & file sizes
 *     littlefs test                    -> run low-level SPI NOR self-test
 ****************************************************************************/

int littlefs_main(int argc, FAR char *argv[])
{
  if (argc > 1)
    {
      if (strcmp(argv[1], "read") == 0)
        {
          if (argc > 2 && strcmp(argv[2], "hk1") == 0)
            {
              return littlefs_read_hk1();
            }
          else if (argc > 2 && strcmp(argv[2], "hk2") == 0)
            {
              return littlefs_read_hk2();
            }
          else
            {
              littlefs_read_hk1();
              littlefs_read_hk2();
              return 0;
            }
        }
      else if (strcmp(argv[1], "clean") == 0 || strcmp(argv[1], "clear") == 0)
        {
          if (argc > 2 && strcmp(argv[2], "hk1") == 0)
            {
              return clean_one_partition("/dev/hk1", "/mnt/hk1");
            }
          else if (argc > 2 && strcmp(argv[2], "hk2") == 0)
            {
              return clean_one_partition("/dev/hk2", "/mnt/hk2");
            }
          else if (argc > 2 && (strcmp(argv[2], "camera") == 0 || strcmp(argv[2], "cam") == 0))
            {
              return clean_one_partition("/dev/camera", "/mnt/camera");
            }
          else
            {
              printf("[LITTLEFS] Cleaning all partitions (HK1, HK2, Camera)...\n");
              clean_one_partition("/dev/hk1", "/mnt/hk1");
              clean_one_partition("/dev/hk2", "/mnt/hk2");
              clean_one_partition("/dev/camera", "/mnt/camera");
              return 0;
            }
        }
      else if (strcmp(argv[1], "format") == 0)
        {
          if (argc > 2 && strcmp(argv[2], "hk1") == 0)
            {
              return format_one_partition("/dev/hk1", "/mnt/hk1");
            }
          else if (argc > 2 && strcmp(argv[2], "hk2") == 0)
            {
              return format_one_partition("/dev/hk2", "/mnt/hk2");
            }
          else if (argc > 2 && (strcmp(argv[2], "camera") == 0 || strcmp(argv[2], "cam") == 0))
            {
              return format_one_partition("/dev/camera", "/mnt/camera");
            }
          else
            {
              printf("[LITTLEFS] Formatting all partitions (HK1, HK2, Camera)...\n");
              format_one_partition("/dev/hk1", "/mnt/hk1");
              format_one_partition("/dev/hk2", "/mnt/hk2");
              format_one_partition("/dev/camera", "/mnt/camera");
              return 0;
            }
        }
      else if (strcmp(argv[1], "size") == 0 || strcmp(argv[1], "partition") == 0 ||
               strcmp(argv[1], "part") == 0 || strcmp(argv[1], "info") == 0)
        {
          return print_partition_sizes();
        }
      else if (strcmp(argv[1], "erase") == 0)
        {
          return littlefs_erase_cmd(argc, argv);
        }
      else if (strcmp(argv[1], "test") == 0)
        {
          return littlefs_hardware_test();
        }
      else if (strcmp(argv[1], "help") == 0 || strcmp(argv[1], "-h") == 0)
        {
          printf("\nUsage: littlefs [command] [target]\n");
          printf("Commands:\n");
          printf("  read   [hk1 | hk2]            - Dump stored telemetry records\n");
          printf("  clean  [hk1 | hk2 | camera]   - Delete files, keep format\n");
          printf("  format [hk1 | hk2 | camera]   - Force-reformat LittleFS partition\n");
          printf("  erase  [hk1 | hk2 | camera | 4kb | 32kb | 64kb | die | all] - Flash erase\n");
          printf("  size   [or partition/info]    - Display partition table & file sizes\n");
          printf("  test                          - Run low-level SPI NOR self-test\n");
          printf("  (no args)                     - Run telemetry storage daemon\n\n");
          return 0;
        }
    }

  return littlefs_telemetry_daemon();
}