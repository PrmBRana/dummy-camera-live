############################################################################
# boards/arm/stm32wl5/nucleo-wl55jc/cpu2/Makefile
#
# STM32WL55 CPU2 / Cortex-M0+ standalone firmware
#
# CPU1 = Cortex-M4 / NuttX
# CPU2 = Cortex-M0+ / standalone firmware
#
# IPCC:
#
#   CH0 : M4 -> M0+
#   CH1 : M0+ -> M4
#
# Shared SRAM2:
#
#   M4 writes 8 x 80-byte packets
#   M0+ reads the packets
#
############################################################################

APP ?= satellite

ifeq ($(APP),gs)
  MAIN_SRC = gs_main.c
  TARGET   = gs_m0plus
else
  MAIN_SRC = sat_main.c
  TARGET   = ipcc_m0plus
endif



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
# COMMON INCLUDE PATHS (100% LOCAL TO CPU2)
# ==========================================================================

CFLAGS = $(CPU) \
         -O2 \
         -Wall \
         -Wextra \
         -g \
         -fdata-sections \
         -ffunction-sections \
         -DUSE_HAL_DRIVER \
         -DSTM32WL55xx \
         -DCORE_CM0PLUS \
         -I. \
         -IInc \
         -Icore \
         -Icore/Src \
         -Icore/Inc \
         -Iradio_driver \
         -Ihal \
         -Isubghz \
         -Ibsp \
         -Itarget \
         -Iipc \
         -Iutilities \
         -Idrivers/CMSIS/Device/ST/STM32WLxx/Include \
         -Idrivers/CMSIS/Include \
         -Idrivers/STM32WLxx_HAL_Driver/Inc


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
# C SOURCE FILES
# ==========================================================================

C_SRCS = \
$(MAIN_SRC) \
uart_debug.c \
ipc/Sring_buffer.c \
core/Src/ax25.c \
core/Src/crc16.c \
core/Src/g3ruh.c \
core/Src/protocol.c \
core/Src/radio_app.c \
core/Src/stm32wlxx_it.c \
core/stm32_seq.c \
core/stm32_lpm.c \
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
hal/stm32wlxx_hal.c \
hal/stm32wlxx_hal_subghz.c \
hal/stm32wlxx_hal_rtc.c \
hal/stm32wlxx_hal_rtc_ex.c \
hal/stm32wlxx_hal_rcc.c \
hal/stm32wlxx_hal_rcc_ex.c \
hal/stm32wlxx_hal_gpio.c \
hal/stm32wlxx_hal_cortex.c \
hal/stm32wlxx_hal_pwr.c \
hal/stm32wlxx_hal_pwr_ex.c \
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
all: sat gs
else ifeq ($(MAKECMDGOALS),all)
all: sat gs
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
	@echo " M0+ BUILD SUCCESS"
	@echo "========================================"
	@echo " ELF : $(TARGET).elf"
	@echo " BIN : $(TARGET).bin"
	@echo " MAP : $(TARGET).map"
	@echo " Flash address: 0x08032000"
	@echo "========================================"

	@$(SIZE) $(TARGET).elf


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

STM32_PROG ?= $(shell which STM32_Programmer_CLI 2>/dev/null || ls /home/prem/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI 2>/dev/null || echo STM32_Programmer_CLI)

# ==========================================================================
# FLASH CPU2
# ==========================================================================

flash: $(TARGET).bin
	@if [ -x "$$(which $(STM32_PROG) 2>/dev/null)" ] || [ -x "$(STM32_PROG)" ]; then \
	  echo "Flashing $(TARGET).bin to 0x08032000 using STM32_Programmer_CLI..."; \
	  $(STM32_PROG) -c port=SWD -w $(TARGET).bin 0x08032000 -v -hardRst; \
	else \
	  echo "Flashing $(TARGET).bin to 0x08032000 using OpenOCD..."; \
	  openocd \
	    -f interface/stlink.cfg \
	    -f target/stm32wlx.cfg \
	    -c "init" \
	    -c "targets" \
	    -c "reset halt" \
	    -c "halt" \
	    -c "flash probe 0" \
	    -c "program $(TARGET).bin 0x08032000 verify" \
	    -c "reset run" \
	    -c "exit"; \
	fi

sat:
	$(MAKE) APP=satellite ipcc_m0plus.bin

flash_sat flash_SAT:
	$(MAKE) APP=satellite flash

gs:
	$(MAKE) APP=gs gs_m0plus.bin

flash_gs flash_GS:
	$(MAKE) APP=gs flash


# ==========================================================================
# SIZE
# ==========================================================================

size: $(TARGET).elf
	$(SIZE) $<


# ==========================================================================
# CLEAN
# ==========================================================================

clean:
	rm -f $(ALL_OBJS) sat_main.o gs_main.o ipc/*.o core/*.o core/Src/*.o radio_driver/*.o subghz/*.o hal/*.o bsp/*.o target/*.o
	rm -f ipcc_m0plus.elf ipcc_m0plus.bin ipcc_m0plus.map
	rm -f gs_m0plus.elf gs_m0plus.bin gs_m0plus.map


.PHONY: all flash sat flash_sat flash_SAT gs flash_gs flash_GS size clean