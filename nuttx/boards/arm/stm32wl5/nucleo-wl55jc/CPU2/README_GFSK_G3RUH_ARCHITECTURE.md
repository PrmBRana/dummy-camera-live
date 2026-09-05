# STM32WL55 GMSK AX.25 G3RUH & CW Morse Beacon Firmware Documentation

## 1. System Architecture Overview

The **STM32WL55JC** is a dual-core SoC consisting of:
1. **CPU1 (Cortex-M4):** Runs the main application / RTOS (NuttX OS).
2. **CPU2 (Cortex-M0+):** Dedicated standalone real-time Sub-GHz radio coprocessor handling all RF PHY, GMSK modulation, CW Morse keying, AX.25 framing, and G3RUH scrambling/descrambling.

This documentation details the **CPU2 (Cortex-M0+)** firmware located in:
`/home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2`

```
+----------------------------------------------------------------------------------------------------+
|                                    SATELLITE FIRMWARE (CPU2 CM0+)                                  |
|                                                                                                    |
|   +--------------------------+       +----------------------------+       +--------------------+   |
|   |  ipcc_m0plus_main.c      | ----> |      core/protocol.c       | ----> |  core/radio_app.c  |   |
|   |  - State Machine         |       |  - AX.25 UI Frame Builder  |       |  - Pure G3RUH GMSK |   |
|   |  - CW Morse Beacon       |       |  - CRC-16 / FCS (LSB)      |       |  - 9600 baud, BT0.5|   |
|   |    ("S2S2BEA" on boot)   |       |  - Bit Stuffing (5x 1s)    |       |  - Fdev = 2400 Hz  |   |
|   |  - 100x Telemetry Burst  |       |  - NRZ-I Encoding          |       |  - +22 dBm (RFO_HP)|   |
|   |  - 5s GMSK Housekeeping  |       |  - G3RUH Scrambling        |       |  - CW Carrier Tone |   |
|   |  - Command Dispatcher    |       +----------------------------+       +--------------------+   |
|   +--------------------------+                                                      |              |
|                                                                                     v              |
|                                                                           +--------------------+   |
|                                                                           | target/            |   |
|                                                                           | radio_board_if.c   |   |
|                                                                           | - Nucleo PC3/4/5   |   |
|                                                                           | - Custom PA8/PA1   |   |
|                                                                           +--------------------+   |
+-------------------------------------------------------------------------------------|--------------+
                                                                                      | (435.000 MHz)
                                                                                      v
+----------------------------------------------------------------------------------------------------+
|                                  GROUND STATION / SDR RECEIVERS                                    |
|                                                                                                    |
|   [CW Mode (Boot)]             SDR (CW / USB Mode @ 435 MHz) ---> Fldigi / CWGet / By Ear          |
|                                                                                                    |
|   [GMSK Mode (Post-Cmd)]       SDR (N-FM @ 435 MHz) ----------> VB-Cable ---> UZ7HO SoundModem     |
|                                (18-20 kHz Flat Audio)                         (FSK G3RUH 9600bd)   |
+----------------------------------------------------------------------------------------------------+
```

---

## 2. Operating Modes & Lifecycle

### Mode 1: Initial Continuous CW Morse Beacon (On Boot)
* **Signal:** Unmodulated RF carrier (Continuous Wave - CW) keyed on **`435.000 MHz`** at `+22 dBm`.
* **Morse Message:** `"S2S2BEA"` (`... ..--- ... ..--- -... . .-`) at standard ~15 WPM ($80\text{ ms}$ dot, $240\text{ ms}$ dash).
* **Listening Window:** After each Morse sequence, the satellite listens for $1.5\text{ seconds}$ on **`437.375 MHz`** for an uplink command.

### Mode Transition: Ground Command Received (`CMD_REQUEST_BURST` / `0x01`)
* When the Ground Station transmits a valid command frame:
  1. The satellite **permanently stops CW mode** (`s_cw_mode_active = false`).
  2. The satellite transmits a **100-packet high-speed GMSK telemetry burst**.
  3. The satellite enters the **Standard Normal Mode**.

### Mode 2: Standard Normal Mode (Post-Command)
* **Housekeeping Beacon:** Transmits a 9600 baud GMSK AX.25 beacon (`"NEPSAT BEACON"`) every **5 seconds** on `435.000 MHz`.
* **Continuous Listening:** Listens continuously on `437.375 MHz` between beacons.
* **On-Demand Burst:** Responds to any subsequent `0x01` ground command with another 100-packet GMSK burst.

---

## 3. Directory Structure & Key Files

### Primary Entry Points
* [ipcc_m0plus_main.c](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/ipcc_m0plus_main.c): **Main entry point for Satellite CPU2 firmware**. Implements CW Morse keying, command detection, 100-packet telemetry burst, and 5-second GMSK beacon timer.
* [gs_m0plus_main.c](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/satellite/gs_main.c): Standalone Ground Station testing firmware entry point.

### Protocol & Framing Subsystem (`core/`)
* [core/Inc/config.h](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/core/Inc/config.h): **Master Configuration Header**. Defines frequencies (`435.000 MHz` TX, `437.375 MHz` RX), Callsigns (`NEPSAT` / `GROUND`), Bitrate (`9600 bps`), Deviation (`2400 Hz` for GMSK $h=0.5$).
* [core/Src/protocol.c](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/core/Src/protocol.c) / [core/Inc/protocol.h](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/core/Inc/protocol.h): Complete AX.25 + G3RUH engine.
* [core/Src/ax25.c](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/core/Src/ax25.c): AX.25 UI frame builder, bit-shifter, and frame parser.
* [core/Src/crc16.c](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/core/Src/crc16.c): Standard CCITT-16 / X.25 reflected CRC calculation (Residue `0xF0B8`).
* [core/Src/g3ruh.c](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/core/Src/g3ruh.c): G3RUH scrambler/descrambler ($1 + x^{12} + x^{17}$).
* [core/Src/radio_app.c](file:///home/prem/Desktop/nuttxspace/nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2/core/Src/radio_app.c): Low-level SubGHz radio PHY controller.

---

## 4. Master Configuration Table (`core/Inc/config.h`)

| Parameter | Macro Name | Value | Description |
| :--- | :--- | :--- | :--- |
| **Transmit Frequency** | `DEFAULT_TX_FREQUENCY_HZ` | `435000000UL` | Satellite $\rightarrow$ Ground Station ($435.000\text{ MHz}$). |
| **Receive Frequency** | `DEFAULT_RX_FREQUENCY_HZ` | `437375000UL` | Ground Station $\rightarrow$ Satellite ($437.375\text{ MHz}$). |
| **Bitrate** | `RADIO_BIT_RATE_BPS` | `9600` | GMSK 9600 baud data rate. |
| **Frequency Deviation** | `RADIO_FDEV_HZ` | `2400` | $\pm 2.4\text{ kHz}$ ($4.8\text{ kHz}$ peak-to-peak shift, $h=0.5$). |
| **Gaussian Shaping** | `MOD_SHAPING_G_BT_05` | $BT = 0.5$ | Gaussian pulse filtering. |
| **Leading Flag Count** | `PROTOCOL_LEADING_FLAG_COUNT` | `64` | 64 flags ($53\text{ ms}$) for receiver PLL lock. |
| **Physical Packet Size**| `RADIO_FIXED_PACKET_LEN` | `200` | Fixed transfer size in bytes. |
| **Output Power** | `RADIO_TX_POWER_DBM` | `22` | $+22\text{ dBm}$ ($160\text{ mW}$) High-Power PA. |
| **Source Callsign** | `DEFAULT_SOURCE_CALLSIGN` | `"NEPSAT"` | Satellite Callsign (SSID 1). |
| **Dest Callsign** | `DEFAULT_DEST_CALLSIGN` | `"GROUND"` | Ground Station Callsign (SSID 0). |

---

## 5. Ground Station & SDR Decoding Guide

### Part A: Decoding the CW Morse Signal (`"S2S2BEA"`)

1. **SDR Software Configuration (SDR# / SDR Console / GQRX / SDR++):**
   * **Frequency:** `435.000 MHz`
   * **Demodulation Mode:** **`CW`** or **`USB` (Upper Sideband)**
   * **Filter Bandwidth:** Set to **`500 Hz` – `1 kHz`**
   * **CW Pitch (BFO offset):** `600 Hz` – `800 Hz` (produces clear audible tone)
2. **Decoding Software Options:**
   * **By Ear:** Standard amateur audio tone.
   * **Automated Decoders (Fldigi / CWGet / CW Skimmer / MRP40):**
     * Route audio to **VB-Audio Virtual Cable**.
     * In **Fldigi**: Select mode **`CW`** $\rightarrow$ Set AFC ON $\rightarrow$ Click on the 700 Hz audio waterfall trace $\rightarrow$ Decoded text `"S2S2BEA"` will print in the text window.

---

### Part B: Decoding GMSK 9600 AX.25 Telemetry Packets

1. **SDR Software Configuration:**
   * **Frequency:** `435.000 MHz`
   * **Demodulation Mode:** **`N-FM` (Narrowband FM)**
   * **Filter Bandwidth:** **`18 kHz` – `20 kHz`**
   * **Audio De-emphasis:** **OFF (Unchecked / Flat Audio)** *(Crucial for 9600 baud)*
   * **Audio High-Cut (Low-Pass):** **`12 kHz` or higher**
   * **Squelch:** **OFF**
   * **Audio Output:** **VB-Audio Virtual Cable**

2. **UZ7HO SoundModem Configuration:**
   * **Modem Type:** Select **`FSK G3RUH 9600bd`** on Channel A.
   * **Audio Input:** Select **CABLE Output (VB-Audio Virtual Cable)**.
   * **DCD Threshold:** Set slider to $\approx 30\% - 40\%$.
   * **Verification:** When packets are received, the **`DCD A`** indicator turns RED and decoded AX.25 frames (`NEPSAT-1 > GROUND-0: NEPSAT BEACON` or burst telemetry) will print in the log window.

---

## 6. Build and Flashing Instructions

```bash
# Build and flash Satellite firmware (CW + GMSK):
make flash_sat

# Build and flash Ground Station firmware:
make flash_gs
```
