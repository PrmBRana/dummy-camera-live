# Technical Report: MT25QL01G NOR Flash & LittleFS Implementation

This document details all the files modified, the exact bugs and architectural issues present in the original code, and how MT25QL01G NOR Flash operations (JEDEC ID read, partitioning, erase, write, and read) work in NuttX.

---

## 1. Summary of All Modified Files

| # | File Path | Purpose of Change |
|---|-----------|-------------------|
| 1 | boards/arm/stm32wl5/nucleo-wl55jc/include/board.h | Configured `GPIO_SPI1_NSS` (PA4) with push-pull output and 50MHz speed. |
| 2 | boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_boot.c | Fixed invalid include `<nuttx/mtd/m25px.h>`, added JEDEC ID query (`0x9F`), created 2x 64MB partitions, and called `stm32wl5_mt25q_initialize()` in `board_late_initialize()`. |
| 3 | boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_spi.c | Configured CS handling (`stm32wl5_spi1select`) and status (`stm32wl5_spi1status`) for `SPIDEV_FLASH(0)`. |
| 4 | boards/arm/stm32wl5/nucleo-wl55jc/src/Makefile | Changed `CSRCS-$(CONFIG_SPI_DRIVER) += stm32_spi.c` to `CSRCS-$(CONFIG_SPI) += stm32_spi.c` so SPI board logic actually compiles into `libboard.a`. |
| 5 | boards/arm/stm32wl5/nucleo-wl55jc/src/CMakeLists.txt | Fixed condition from `CONFIG_SPI_DRIVER` to `CONFIG_SPI` for parity with Makefile. |
| 6 | boards/arm/stm32wl5/nucleo-wl55jc/scripts/Make.defs | Added `POSTBUILD` hook to automatically calculate and display total Flash and SRAM usage at the end of `make`. |
| 7 | drivers/mtd/m25px.c | Added 4-byte address mode (`0xB7`) support and 4th address byte (`offset >> 24`) for 1Gb (128MB) MT25Q1G flash. |
| 8 | .config | Enabled `CONFIG_MTD_PARTITION=y` so `mtd_partition` is included in the build. |
| 9 | ../apps/examples/littlefs/littlefs_main_mount.c | Implemented JEDEC ID query, dual partition verification, autoformatted LittleFS mount (`/mnt/littlefs1` and `/mnt/littlefs2`), dummy burst data write, readback, data integrity verification, and UART hex/ASCII dump. |
| 10 | ../apps/examples/littlefs/CMakeLists.txt | Removed reference to nonexistent `ramdisk.c`. |

---

## 2. Where Were the Mistakes in the Original Code?

### Mistake 1: Fatal Include Error in `stm32_boot.c`
- **What was there**: `#include <nuttx/mtd/m25px.h>`
- **Why it failed**: The file `nuttx/mtd/m25px.h` does not exist in NuttX. All MTD declarations (including `m25p_initialize`) are in `<nuttx/mtd/mtd.h>`.
- **Fix**: Removed the `#include <nuttx/mtd/m25px.h>` line; `<nuttx/mtd/mtd.h>` was already present.

### Mistake 2: Missing `stm32_spi.c` Compilation in `libboard.a`
- **What was there**: In `boards/arm/stm32wl5/nucleo-wl55jc/src/Makefile`:
  ```makefile
  CSRCS-$(CONFIG_SPI_DRIVER) += stm32_spi.c
  ```
- **Why it failed**: `CONFIG_SPI_DRIVER` is the option for the character driver `/dev/spi*`, which was not enabled in your `.config`. As a result, `stm32_spi.c` was **never compiled**, causing linker errors:
  - `undefined reference to stm32wl5_spidev_initialize`
  - `undefined reference to stm32wl5_spi1select`
  - `undefined reference to stm32wl5_spi1status`
- **Fix**: Changed the condition to `CONFIG_SPI`, which is enabled whenever SPI peripheral support is turned on:
  ```makefile
  CSRCS-$(CONFIG_SPI) += stm32_spi.c
  ```

### Mistake 3: `stm32wl5_mt25q_initialize()` Was Never Called
- **What was there**: The function was defined as `static int stm32wl5_mt25q_initialize(void)` in `stm32_boot.c`, but it was never invoked anywhere in `board_late_initialize()`.
- **Why it failed**: At boot, the external flash hardware was never initialized.
- **Fix**: Added the invocation inside `board_late_initialize()`:
  ```c
  #if defined(CONFIG_MTD_M25P)
    ret = stm32wl5_mt25q_initialize();
  #endif
  ```

### Mistake 4: Missing `CONFIG_MTD_PARTITION` in `.config`
- **What was there**: `# CONFIG_MTD_PARTITION is not set`
- **Why it failed**: `mtd_partition()` in `drivers/mtd/mtd_partition.c` was excluded from compilation, causing an `undefined reference to mtd_partition` at link time.
- **Fix**: Enabled `CONFIG_MTD_PARTITION=y` in `.config`.

### Mistake 5 (CRITICAL BUG): 3-Byte Address Wrap-around in `drivers/mtd/m25px.c`
- **What was there**: The NuttX driver `m25px.c` had an entry for `M25P_MT25Q1G_CAPACITY (0x21)`, but all its command functions hardcoded **3-byte addressing**:
  ```c
  SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, offset & 0xff);
  ```
- **Why it failed**:
  - 3-byte addressing can only reach $2^{24} = 16\text{ MB}$ (128 Mbit).
  - MT25QL01G is **128 MB** (1 Gbit).
  - If you partition the flash into two 64 MB partitions, Partition 2 starts at offset 64 MB (`0x04000000`).
  - Because `offset >> 24` was discarded, `0x04000000` wrapped around to `0x000000`.
  - **Writing to Partition 2 would silently overwrite and corrupt Partition 1!**
- **Fix**:
  1. Added `uint8_t addrlen;` to `struct m25p_dev_s`.
  2. When `capacity == M25P_MT25Q1G_CAPACITY`, set `priv->addrlen = 4` and sent command `0xB7` (`ENTER_4BYTE_ADDR_MODE`) to the chip.
  3. In `m25p_sectorerase`, `m25p_pagewrite`, `m25p_bytewrite`, and `m25p_read`, sent the 4th address byte:
     ```c
     if (priv->addrlen == 4)
       {
         SPI_SEND(priv->dev, (offset >> 24) & 0xff);
       }
     ```

### Mistake 6: LittleFS Mount Failure on Fresh/Unformatted Flash
- **What was there**: In `littlefs_main_mount.c`:
  ```c
  ret = mount(LITTLEFS_MTD_PATH, LITTLEFS_MOUNTPOINT, "littlefs", 0, NULL);
  ```
- **Why it failed**: On a brand new or unformatted flash, LittleFS returns `-EFAULT` (`LFS_ERR_CORRUPT`). Without formatting options, `mount()` aborts.
- **Fix**: Used `"autoformat"` as the mount parameter:
  ```c
  ret = mount(dev, mnt, "littlefs", 0, "autoformat");
  ```
  NuttX's LittleFS driver (`fs/littlefs/lfs_vfs.c`) checks for `"autoformat"`. If the filesystem is not yet initialized, it automatically formats the partition and then mounts it.

---

## 3. How the Operations Work (Step-by-Step)

### A. JEDEC ID Readout
1. Asserts CS low (`stm32wl5_gpiowrite(GPIO_SPI1_NSS, false)`).
2. Sends command byte `0x9F` (RDID).
3. Clocks in 3 response bytes:
   - Byte 0: `0x20` (Micron Manufacturer ID).
   - Byte 1: `0xBA` (Memory Type: MT25QL 3V).
   - Byte 2: `0x21` (Capacity: 1 Gbit = 128 MB).
4. De-asserts CS high (`stm32wl5_gpiowrite(GPIO_SPI1_NSS, true)`).
5. Prints the decoded information to the UART console.

### B. Partitioning
- MT25QL01G has:
  - Sector size = 64 KB ($65,536\text{ bytes}$).
  - Total sectors = $\frac{128\text{ MB}}{64\text{ KB}} = 2048\text{ sectors}$.
- Partition 1: `mtd_partition(mtd, 0, 1024)`:
  - Covers sectors 0 to 1023 ($64\text{ MB}$).
  - Registered as character MTD `/dev/mt25q_p1`.
- Partition 2: `mtd_partition(mtd, 1024, 1024)`:
  - Covers sectors 1024 to 2047 ($64\text{ MB}$).
  - Registered as character MTD `/dev/mt25q_p2`.

### C. LittleFS Mount & Autoformat
- `mount("/dev/mt25q_p1", "/mnt/littlefs1", "littlefs", 0, "autoformat")`:
  - LittleFS reads the superblock on the partition.
  - If the flash is unformatted (blank `0xFF`), LittleFS detects corruption and executes `lfs_format()`.
  - Once formatted, it mounts the root directory at `/mnt/littlefs1`.
- Repeated for Partition 2 at `/mnt/littlefs2`.

### D. Dummy Burst Data Write
- Generates a 512-byte buffer with a readable header and 8-bit sequential test pattern (`0x00..0xFF`).
- Opens file `/mnt/littlefs1/burst_p1.dat` (`O_WRONLY | O_CREAT | O_TRUNC`).
- `write(fd, wbuf, 512)` transfers the data through LittleFS.
- LittleFS allocates physical blocks on Partition 1, issues `m25p_pagewrite` (command `0x02` with 4-byte address), and commits the transaction.
- `fsync(fd)` and `close(fd)` ensure all cache blocks are flushed to physical NOR flash.

### E. Dummy Burst Data Read & UART Print
- Opens file `/mnt/littlefs1/burst_p1.dat` (`O_RDONLY`).
- `read(fd, rbuf, 512)` reads back the data from the flash via `m25p_read` (command `0x03` with 4-byte address).
- Verifies `rbuf[i] == wbuf[i]` for all 512 bytes (100% data integrity check).
- Calls `print_hexdump()` to output the full 512 bytes to UART in standard hex + ASCII format:
  ```
  [UART DUMP] Data read back from /mnt/littlefs1/burst_p1.dat (512 bytes):
    [0000] 4D 54 32 35 51 4C 30 31 47 20 4C 69 74 74 6C 65  |MT25QL01G Little|
    [0010] 46 53 20 42 75 72 73 74 20 54 65 73 74 3A 20 50  |FS Burst Test: P|
    [0020] 61 74 68 3D 2F 6D 6E 74 2F 6C 69 74 74 6C 65 66  |ath=/mnt/littlef|
    ...
  ```
- Repeats the identical sequence on Partition 2 (`/mnt/littlefs2/burst_p2.dat`) to ensure both partitions operate independently without interference.

---

## 4. Automatic Build Memory Printout

Added `POSTBUILD` rule to `boards/arm/stm32wl5/nucleo-wl55jc/scripts/Make.defs`. Every time you run `make`, it displays:

```
==================== Memory Usage Summary ====================
  FLASH : 102932 B / 262144 B (100.5 KB / 256.0 KB) [39.27% used]
          Free Flash: 159212 B (155.5 KB)
  SRAM  :   7320 B /  32768 B (  7.1 KB /  32.0 KB) [22.34% used]
          Free SRAM :  25448 B ( 24.9 KB)
==============================================================
```
