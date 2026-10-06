############################################################################
# CPU2/Makefile
#
# STM32WL55 Standalone Ground Station (GS) Firmware (Cortex-M0+)
#
# Hardware:
#   - High-Power Sub-GHz Transceiver (RFO_HP up to +22 dBm)
#   - External Satellite PAs DISABLED (Safe for GS / Nucleo hardware)
#   - Frequencies: TX = 437.375 MHz (Uplink), RX = 435.000 MHz (Downlink)
#   - Serial Console: UART2 @ 115200 8N1 (LPUART1 / USART1)
#   - Standalone firmware running at 0x08032000 (56 KB)
#
############################################################################

MAIN_SRC = Main_code/Main_Ground_Station/gs_app.c \
           Main_code/Main_Ground_Station/cw_rx.c
TARGET   = gs_m0plus


# ==========================================================================
# TOOLCHAIN
# ==========================================================================

CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size


# ==========================================================================
# LINKER SCRIPT (SELF-CONTAINED INSIDE CPU2)
# ==========================================================================

LDSCRIPT = scripts/wl55jc_cm0p.ld


# ==========================================================================
# CPU
# ==========================================================================

CPU = -mcpu=cortex-m0plus -mthumb


# ==========================================================================
# COMMON INCLUDE PATHS (100% MATCHING SATELLITE_M0+)
# ==========================================================================

CFLAGS = $(CPU) \
         -Os \
         -Wall \
         -Wextra \
         -g \
         -fdata-sections \
         -ffunction-sections \
         -DUSE_HAL_DRIVER \
         -DSTM32WL55xx \
         -DCORE_CM0PLUS \
         -I. \
         -IMain_code/Main_Ground_Station \
         -IInc \
         -IMission \
         -Iprotocol \
         -Iprotocol/ax25 \
         -Iprotocol/g3ruh \
         -Icore \
         -Iradio_driver \
         -Ihal \
         -Ihal/Inc \
         -Isubghz \
         -Ibsp \
         -Itarget \
         -Iipc \
         -Iutilities \
         -Iuart \
         -Idrivers/CMSIS/Device/ST/STM32WLxx/Include \
         -Idrivers/CMSIS/Core/Include


# ==========================================================================
# ASSEMBLER
# ==========================================================================

ASFLAGS = $(CPU) -g


# ==========================================================================
# LINKER
# ==========================================================================

LDFLAGS = $(CPU) \
          -T $(LDSCRIPT) \
          -Wl,--gc-sections \
          -Wl,-Map=$(TARGET).map \
          --specs=nano.specs \
          --specs=nosys.specs \
          -nostartfiles


# ==========================================================================
# C SOURCE FILES (MATCHING SATELLITE_M0+ MODULAR ARCHITECTURE)
# ==========================================================================

C_SRCS = \
$(MAIN_SRC) \
uart/uart_debug.c \
ipc/Sring_buffer.c \
protocol/ax25/ax25.c \
protocol/ax25/crc16.c \
protocol/g3ruh/g3ruh.c \
Mission/protocol.c \
Mission/radio_app.c \
protocol/stm32wlxx_it.c \
protocol/stm32_seq.c \
protocol/stm32_lpm.c \
core/stm32_mem.c \
core/system_stm32wlxx.c \
core/timer_if.c \
core/rtc.c \
core/error_handler.c \
core/stm32_adv_trace.c \
core/stm32_tiny_vsnprintf.c \
core/stm32_trace_driver.c \
radio_driver/radio.c \
radio_driver/radio_driver.c \
radio_driver/radio_fw.c \
radio_driver/stm32_timer.c \
radio_driver/sys_app.c \
target/radio_board_if.c \
subghz/subghz.c \
hal/Src/stm32wlxx_hal.c \
hal/Src/stm32wlxx_hal_subghz.c \
hal/Src/stm32wlxx_hal_rtc.c \
hal/Src/stm32wlxx_hal_rtc_ex.c \
hal/Src/stm32wlxx_hal_rcc.c \
hal/Src/stm32wlxx_hal_rcc_ex.c \
hal/Src/stm32wlxx_hal_gpio.c \
hal/Src/stm32wlxx_hal_cortex.c \
hal/Src/stm32wlxx_hal_pwr.c \
hal/Src/stm32wlxx_hal_pwr_ex.c \
bsp/stm32wlxx_nucleo_radio.c


# ==========================================================================
# STARTUP
# ==========================================================================

S_SRCS = startup_cm0p.s


# ==========================================================================
# OBJECT FILES
# ==========================================================================

C_OBJS = $(C_SRCS:.c=.o)

S_OBJS = $(S_SRCS:.s=.o)

ALL_OBJS = $(C_OBJS) $(S_OBJS)


# ==========================================================================
# DEFAULT BUILD
# ==========================================================================

ifeq ($(MAKECMDGOALS),)
all: $(TARGET).bin
else ifeq ($(MAKECMDGOALS),all)
all: $(TARGET).bin
else
all: $(TARGET).bin
endif


# ==========================================================================
# LINK ELF
# ==========================================================================

$(TARGET).elf: $(ALL_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^


# ==========================================================================
# CREATE BIN
# ==========================================================================

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

	@echo ""
	@echo "========================================"
	@echo " M0+ BUILD SUCCESS: $(TARGET)"
	@echo "========================================"
	@echo " ELF : $(TARGET).elf"
	@echo " BIN : $(TARGET).bin"
	@echo " MAP : $(TARGET).map"
	@echo " Flash address: 0x08032000"
	@echo "========================================"

	@$(SIZE) $(TARGET).elf | awk 'NR==2 { \
		text=$$1; data=$$2; bss=$$3; \
		flash=text+data; ram=data+bss; \
		c2_flash=56*1024; c2_ram=28*1024; \
		f_used=flash; f_free=c2_flash-flash; f_pct=f_used*100/c2_flash; f_free_pct=100-f_pct; \
		r_used=ram; r_free=c2_ram-ram; r_pct=r_used*100/c2_ram; r_free_pct=100-r_pct; \
		printf "\n==================================== CPU2 (M0+) MEMORY USAGE SUMMARY ====================================\n"; \
		printf "  %-18s  %-19s  %-21s  %-20s  %7s  %7s\n", "Memory Region", "Total Capacity", "Used Space", "Free / Available", "Used %", "Free %"; \
		printf "  ------------------  -------------------  ---------------------  --------------------  -------  -------\n"; \
		printf "  %-18s  %5.1f KB (%6d B)  %5.1f KB (%6d B)  %5.1f KB (%6d B)   %5.1f%%   %5.1f%%\n", "CPU2 Flash (M0+)", c2_flash/1024, c2_flash, f_used/1024, f_used, f_free/1024, f_free, f_pct, f_free_pct; \
		printf "  %-18s  %5.1f KB (%6d B)  %5.1f KB (%6d B)  %5.1f KB (%6d B)   %5.1f%%   %5.1f%%\n", "CPU2 RAM (SRAM2)", c2_ram/1024, c2_ram, r_used/1024, r_used, r_free/1024, r_free, r_pct, r_free_pct; \
		printf "  %-18s  %5.1f KB (%6d B)  [  Reserved for M4 <-> M0+ Shared Ringbuffer at 0x20008000  ]\n", "Shared IPC SRAM2", 4.0, 4096; \
		printf "==========================================================================================================\n\n"; \
	}'


# ==========================================================================
# COMPILE C
# ==========================================================================

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@


# ==========================================================================
# ASSEMBLE
# ==========================================================================

%.o: %.s
	$(AS) $(ASFLAGS) -c $< -o $@


# ==========================================================================
# PROGRAMMER
# ==========================================================================

STM32_PROG ?= $(shell which STM32_Programmer_CLI 2>/dev/null || \
	ls $(HOME)/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI 2>/dev/null || \
	ls /home/*/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI 2>/dev/null | head -n 1 || \
	ls /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_*/tools/bin/STM32_Programmer_CLI 2>/dev/null | head -n 1 || \
	ls /opt/st/stm32cubeprogrammer_*/bin/STM32_Programmer_CLI 2>/dev/null | head -n 1 || \
	echo STM32_Programmer_CLI)

# ==========================================================================
# FLASH CPU2 (0x08032000 - 0x0803FFFF)
# ==========================================================================

flash flash_gs flash_GS: $(TARGET).bin
	@if [ -x "$$(which $(STM32_PROG) 2>/dev/null)" ] || [ -x "$(STM32_PROG)" ]; then \
	  echo "Flashing Ground Station $(TARGET).bin to 0x08032000 using STM32_Programmer_CLI (CPU1 preserved)..."; \
	  $(STM32_PROG) -c port=SWD mode=UR -w $(TARGET).bin 0x08032000 -v -hardRst 2>/dev/null || \
	  $(STM32_PROG) -c port=SWD mode=HOTPLUG -w $(TARGET).bin 0x08032000 -v -hardRst 2>/dev/null || \
	  $(STM32_PROG) -c port=SWD -w $(TARGET).bin 0x08032000 -v -hardRst || \
	  (echo "STM32_Programmer_CLI failed. Trying OpenOCD fallback..." && \
	  openocd -f interface/stlink.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program $(TARGET).bin 0x08032000 verify reset exit" 2>/dev/null || \
	  openocd -f interface/stlink-dap.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program $(TARGET).bin 0x08032000 verify reset exit"); \
	  echo "Booting Ground Station CPU2 (Cortex-M0+) via PWR_CR4_C2BOOT..."; \
	  $(STM32_PROG) -c port=SWD -w32 0x5800040C 0x00008000 2>/dev/null || true; \
	else \
	  echo "Flashing Ground Station $(TARGET).bin to 0x08032000 using OpenOCD (CPU1 preserved)..."; \
	  openocd -f interface/stlink.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program $(TARGET).bin 0x08032000 verify reset exit" 2>/dev/null || \
	  openocd -f interface/stlink-dap.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program $(TARGET).bin 0x08032000 verify reset exit"; \
	fi

gs GS: $(TARGET).bin

flash_sat flashsatM0 flash_SAT:
	$(MAKE) -C ../Satellite_M0+/Com_sat flash_sat

# ==========================================================================
# SIZE
# ==========================================================================

size: $(TARGET).elf
	@$(SIZE) $< | awk 'NR==2 { \
		text=$$1; data=$$2; bss=$$3; \
		flash=text+data; ram=data+bss; \
		c2_flash=56*1024; c2_ram=28*1024; \
		f_used=flash; f_free=c2_flash-flash; f_pct=f_used*100/c2_flash; f_free_pct=100-f_pct; \
		r_used=ram; r_free=c2_ram-ram; r_pct=r_used*100/c2_ram; r_free_pct=100-r_pct; \
		printf "\n==================================== CPU2 (M0+) MEMORY USAGE SUMMARY ====================================\n"; \
		printf "  %-18s  %-19s  %-21s  %-20s  %7s  %7s\n", "Memory Region", "Total Capacity", "Used Space", "Free / Available", "Used %", "Free %"; \
		printf "  ------------------  -------------------  ---------------------  --------------------  -------  -------\n"; \
		printf "  %-18s  %5.1f KB (%6d B)  %5.1f KB (%6d B)  %5.1f KB (%6d B)   %5.1f%%   %5.1f%%\n", "CPU2 Flash (M0+)", c2_flash/1024, c2_flash, f_used/1024, f_used, f_free/1024, f_free, f_pct, f_free_pct; \
		printf "  %-18s  %5.1f KB (%6d B)  %5.1f KB (%6d B)  %5.1f KB (%6d B)   %5.1f%%   %5.1f%%\n", "CPU2 RAM (SRAM2)", c2_ram/1024, c2_ram, r_used/1024, r_used, r_free/1024, r_free, r_pct, r_free_pct; \
		printf "  %-18s  %5.1f KB (%6d B)  [  Reserved for M4 <-> M0+ Shared Ringbuffer at 0x20008000  ]\n", "Shared IPC SRAM2", 4.0, 4096; \
		printf "==========================================================================================================\n\n"; \
	}'


# ==========================================================================
# CLEAN
# ==========================================================================

clean:
	rm -f $(ALL_OBJS) gs_main.o ipc/*.o core/*.o core/Src/*.o radio_driver/*.o subghz/*.o hal/*.o bsp/*.o target/*.o
	rm -f $(TARGET).elf $(TARGET).bin $(TARGET).map

.PHONY: all flash flash_gs flash_GS gs GS size clean