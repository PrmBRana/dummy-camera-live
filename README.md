# NuttX STM32WL55 Dual-Core Test — M4 (LPUART) / M0+ (USART1)

A working dual-core firmware setup for the **NUCLEO-WL55JC** board (STM32WL55, Cortex-M4 + Cortex-M0+), built on **Apache NuttX RTOS**. The M4 core runs full NuttX with an IPCC test application; the M0+ core runs an independent bare-metal firmware image, flashed and linked separately. Inter-processor communication is established over the on-chip **IPCC** (Inter-Processor Communication Controller) peripheral, with each core also exposing its own UART for independent console/debug output.

**Status: dual-core boot, build, and flash flow verified working end-to-end on hardware.**

---

## Overview

The STM32WL55 is a dual-core SoC:

- **CPU1 — Cortex-M4**: runs Apache NuttX RTOS, hosts the NSH shell and the `ipcc_test` application.
- **CPU2 — Cortex-M0+**: runs a small bare-metal firmware, built independently of the NuttX build system, linked to its own flash/RAM regions, and flashed separately via OpenOCD.

The two cores communicate through the **IPCC** peripheral (hardware mailbox/interrupt mechanism), while each core also drives its own UART peripheral for human-readable debug output:

| Core | Role | RTOS | Debug UART |
|------|------|------|------------|
| CPU1 (Cortex-M4) | Main application core | NuttX | LPUART1 |
| CPU2 (Cortex-M0+) | Co-processor / IPCC responder | Bare-metal (no RTOS) | USART1 (PB6/PB7) |

This repository documents a from-scratch bring-up: NuttX `menuconfig`/build for CPU1, a hand-written linker script + startup assembly + Makefile for CPU2, and a verified OpenOCD flashing flow for the M0+ image at its dedicated flash offset.

---

## Repository Layout

```
nuttxspace/
├── nuttx/                                   # Apache NuttX RTOS source tree
│   ├── arch/arm/src/stm32wl5/               # STM32WL5 arch support
│   ├── boards/arm/stm32wl5/nucleo-wl55jc/   # Board support package (CPU1)
│   │   ├── configs/                         # Board defconfigs
│   │   ├── include/                         # Board headers
│   │   ├── scripts/                         # CPU1 linker scripts
│   │   ├── src/                             # Board bring-up source
│   │   └── CPU2/                            # Independent bare-metal M0+ project
│   │       ├── Makefile                     # Standalone build/flash for CPU2
│   │       ├── ipcc_m0plus_main.c           # M0+ application (USART1 + IPCC)
│   │       └── startup_cm0p.s               # Cortex-M0+ vector table & startup
│   └── ...
├── apps/
│   └── examples/
│       └── ipcc_test/
│           └── ipcc_test.c                  # CPU1 NSH app: talks to M0+ via /dev/ipcc0
└── README.md
```

---

## Hardware

- **Board:** ST NUCLEO-WL55JC
- **MCU:** STM32WL55JC (Cortex-M4 @ up to 48 MHz + Cortex-M0+ co-processor)
- **Debug probe:** Onboard ST-LINK V3 (confirmed via `lsusb`: `0483:374e STLINK-V3`)
- **Flash layout used in this project:**

| Region | Owner | Address | Size |
|--------|-------|---------|------|
| CPU1 application | Cortex-M4 / NuttX | `0x08000000` | up to `0x08020000` |
| CPU2 application | Cortex-M0+ / bare-metal | `0x08020000` | 128 KB |
| CPU2 RAM | Cortex-M0+ | `0x20000000` | 32 KB |

This split keeps the M0+ firmware entirely out of the NuttX-managed flash region, so each core can be built, flashed, and reset independently without touching the other's image.

---

## CPU1 — NuttX (Cortex-M4)

### Configuration & Build

```bash
cd nuttxspace/nuttx
make menuconfig     # configure board, drivers, and apps
make                # builds nuttx.elf / nuttx.bin
```

Build output confirms:
- Board symlinks resolved correctly (`arch/board`, `arch/chip`, `drivers/platform`, etc. all linked against `stm32wl5` / `nucleo-wl55jc`).
- Two applications registered into the NSH shell: `ipcc_test` and `nsh`/`sh`.
- Final link succeeds (`LD: nuttx`), producing `nuttx.bin`.

A harmless build-time warning appears regarding `CONFIG_BOARD_LOOPSPERMSEC` being unset — this only affects the precision of `up_udelay()`/busy-wait timing and does not block the build. It can be resolved later by calibrating with the `calib_udelay` NuttX app if microsecond-accurate delays are needed.

### IPCC Test Application

`apps/examples/ipcc_test/ipcc_test.c` is registered as an NSH command. It:

1. Opens `/dev/ipcc0` (IPCC channel 1, exposed as a character device).
2. Writes a test string (`"Hello from M4!"`) to the M0+ core.
3. Blocks on `read()` waiting for a reply from the M0+ core.
4. Prints the received bytes back out over the NSH console.

This validates the full path: **NuttX character driver → IPCC hardware mailbox → M0+ ISR → response → back to M4 NSH shell.**

Run it from the NSH prompt once flashed:

```
nsh> ipcc_test
IPCC Test starting...
Opened /dev/ipcc0 OK
Sent 14 bytes: "Hello from M4!"
Waiting for M0+ reply...
Received N bytes: "..."
IPCC Test done.
```

---

## CPU2 — Bare-Metal Firmware (Cortex-M0+)

The M0+ core does **not** run NuttX in this setup — it runs a minimal, hand-rolled bare-metal image built with its own Makefile, linker script, and startup assembly, completely decoupled from the NuttX build graph.

### Build System

Location: `boards/arm/stm32wl5/nucleo-wl55jc/CPU2/Makefile`

```bash
cd nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2
make            # compile + link -> ipcc_m0plus.bin
make size       # show .text/.data/.bss section sizes
make flash      # flash to 0x08020000 via OpenOCD
make clean      # remove build artifacts
```

Toolchain: `arm-none-eabi-gcc` targeting `-mcpu=cortex-m0plus -mthumb` (Thumb-1 only, no Thumb-2 extensions, matching the M0+ core's instruction set).

Confirmed working build output:

```
   text    data     bss     dec     hex filename
    648       0    1536    2184     888 ipcc_m0plus.elf
```

### Linker Script

The M0+ image is linked into its own dedicated flash and RAM windows, isolated from CPU1's NuttX image:

```
MEMORY {
  FLASH (rx)  : ORIGIN = 0x08020000, LENGTH = 128K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 32K
}
```

Standard Cortex-M section layout (`.isr_vector`, `.text`, `.data` with FLASH-to-RAM load mapping, `.bss`, and a heap/stack reservation block at the end of RAM).

### Startup Code (`startup_cm0p.s`)

A minimal Cortex-M0+ startup routine:

- Defines the full vector table (`g_vectors`), including all 61 external IRQ slots for the STM32WL55 M0+ vector layout.
- **`IRQ1` is wired to `IPCC_C2_RX_IRQHandler`** — the key vector for this project, firing when CPU1 writes a message into the IPCC mailbox.
- `Reset_Handler` performs the standard `.data` copy (FLASH→RAM) and `.bss` zero-fill before calling `main()`.
- All other handlers (`NMI`, `HardFault`, `SVC`, `PendSV`, `SysTick`, and every unused IRQ) are weakly aliased to `Default_Handler`, an infinite loop — safe defaults until/unless a real handler is implemented.

### M0+ Application (`ipcc_m0plus_main.c`)

The M0+ firmware brings up **USART1** directly via register-level access (no HAL, no RTOS):

- Enables `GPIOB` and `USART1` peripheral clocks via `RCC_AHB2ENR` / `RCC_APB2ENR`.
- Configures `PB6` (TX) / `PB7` (RX) into alternate-function mode, **AF7 (USART1)**.
- Sets the baud-rate register for **115200 baud**, computed against a **48 MHz** `SYSCLK` (`BRR = 417`) — this assumes CPU1 has already switched the shared system clock to 48 MHz via PLL before CPU2 starts transmitting, since both cores share the same clock domain on this part.
- Initializes `USART1_CR1` with `UE | TE | RE` (USART enable, transmit enable, receive enable).
- Main loop continuously transmits a heartbeat string over USART1, confirming the M0+ core is alive and running independently of CPU1.

This UART path is separate from the IPCC link — it's a simple, independent liveness/debug output for the M0+ core, useful for confirming the co-processor booted and is executing before even testing the IPCC channel.

---

## Flashing the M0+ Core

The M0+ image is flashed independently via OpenOCD, targeting its dedicated `0x08020000` flash offset:

```bash
openocd \
  -f interface/stlink.cfg \
  -f target/stm32wlx.cfg \
  -c "init" \
  -c "targets" \
  -c "reset halt" \
  -c "halt" \
  -c "flash probe 0" \
  -c "program ipcc_m0plus.bin 0x08020000 verify" \
  -c "reset run" \
  -c "exit"
```

This is wired up as `make flash` in the CPU2 Makefile.

### Bring-up notes from real hardware sessions

A few practical issues came up while getting this flow stable, in case they help future debugging:

- **Stale/zombie OpenOCD or GDB sessions** holding the ST-LINK can cause `make flash` to fail with `Error 1` and no further diagnostic output, since OpenOCD can't claim the probe. Fix: `pkill -9 openocd` and check for lingering `gdb-multiarch` processes before retrying.
- **Confirm the probe is enumerated** before debugging further: `lsusb | grep -i stm` should show the ST-LINK (`0483:374e STLINK-V3` in this case).
- **Sanity-check the connection** with a bare `openocd -c "init" -c "exit"` first — this should report the target core (`Cortex-M4 r0p1` is detected here, since OpenOCD attaches to CPU1/AP0 by default on this dual-core part), idcode, flash size, and RDP level without attempting to program anything.
- **Clock speed auto-negotiation:** OpenOCD requests 500 kHz SWD clock but the ST-LINK/target negotiates down to 200 kHz (`Unable to match requested speed 500 kHz, using 200 kHz`) — this is expected/benign and programming still completes successfully.
- Once the probe is free and connectivity is confirmed, `make flash` consistently completes with `** Programming Finished **` / `** Verify Started **` / `** Verified OK **`.

---

## Build & Flash — Full Sequence (as verified)

```bash
# 1. Build and flash CPU1 (NuttX / Cortex-M4)
cd nuttxspace/nuttx
make menuconfig
make
# (flash nuttx.bin to CPU1 via your normal NuttX flashing flow)

# 2. Build and flash CPU2 (bare-metal / Cortex-M0+)
cd boards/arm/stm32wl5/nucleo-wl55jc/CPU2
make clean
make
make flash
```

After both cores are flashed and the board is reset:

```
nsh> ipcc_test
```

on the CPU1 NSH console should successfully complete a round-trip exchange with the CPU2 IPCC handler, while CPU2's USART1 output independently prints its heartbeat message.

---

## Key Design Notes

- **CPU2 is intentionally NOT a NuttX target.** It's built and flashed entirely outside the NuttX build graph (own Makefile, own linker script, own startup file) so the M0+ side stays minimal, fast to iterate on, and easy to reason about at the register level.
- **Flash/RAM partitioning is static and address-based** — CPU1 owns `0x08000000` upward, CPU2 owns `0x08020000` upward, with no overlap, so `make flash` on CPU2 never touches CPU1's image and vice versa.
- **IPCC is the cross-core transport; UART is per-core debug output.** These are two independent channels — don't confuse `/dev/ipcc0` traffic on CPU1 with the raw USART1 heartbeat coming from CPU2.
- **Shared `SYSCLK` domain:** since both cores derive their peripheral clocks from the same PLL/SYSCLK on the STM32WL55, the M0+ UART baud-rate divisor must be computed against whatever clock CPU1 has already configured by the time CPU2 starts running — these two cores are not clock-independent.

---

## Toolchain

- `arm-none-eabi-gcc` / `arm-none-eabi-objcopy` / `arm-none-eabi-size`
- OpenOCD `0.12.0+dev` (with `interface/stlink.cfg` + `target/stm32wlx.cfg`)
- Apache NuttX RTOS (CPU1)

---

## License

CPU1 application code (`apps/examples/ipcc_test/ipcc_test.c`) is contributed under the **Apache License 2.0**, consistent with the upstream Apache NuttX project. See individual file headers for details.
