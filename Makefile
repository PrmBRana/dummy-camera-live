CPU2_DIR   ?= $(shell if [ -d JC2/cpu2 ]; then echo JC2/cpu2; elif [ -d CPU2 ]; then echo CPU2; else echo nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2; fi)
STM32_PROG ?= $(shell which STM32_Programmer_CLI 2>/dev/null || \
	ls $(HOME)/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI 2>/dev/null || \
	ls /home/*/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI 2>/dev/null | head -n 1 || \
	ls /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_*/tools/bin/STM32_Programmer_CLI 2>/dev/null | head -n 1 || \
	ls /opt/st/stm32cubeprogrammer_*/bin/STM32_Programmer_CLI 2>/dev/null | head -n 1 || \
	echo STM32_Programmer_CLI)

all:
	$(MAKE) -C nuttx

# ==========================================================================
# FLASH CPU1 (CORTEX-M4 / NUTTX ONLY: 0x08000000 - 0x08031FFF)
# This writes ONLY to sectors 0-99 (200 KB max).
# CPU2 at 0x08032000 (sectors 100-127) is strictly preserved!
# ==========================================================================
flash flash_cpu1 flash_m4 flash_M4 flash_CPU1:
	@if [ ! -f nuttx/nuttx.bin ]; then \
	  echo "nuttx/nuttx.bin not found. Building NuttX..."; \
	  $(MAKE) -C nuttx; \
	fi
	@if [ -x "$$(which $(STM32_PROG) 2>/dev/null)" ] || [ -x "$(STM32_PROG)" ]; then \
	  echo "Flashing CPU1 (NuttX) to 0x08000000 using STM32_Programmer_CLI (CPU2 preserved)..."; \
	  $(STM32_PROG) -c port=SWD mode=UR -w nuttx/nuttx.bin 0x08000000 -v -hardRst 2>/dev/null || \
	  $(STM32_PROG) -c port=SWD mode=HOTPLUG -w nuttx/nuttx.bin 0x08000000 -v -hardRst 2>/dev/null || \
	  $(STM32_PROG) -c port=SWD -w nuttx/nuttx.bin 0x08000000 -v -hardRst || \
	  (echo "STM32_Programmer_CLI failed. Trying OpenOCD fallback..." && \
	  openocd -f interface/stlink.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program nuttx/nuttx.bin 0x08000000 verify reset exit" 2>/dev/null || \
	  openocd -f interface/stlink-dap.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program nuttx/nuttx.bin 0x08000000 verify reset exit"); \
	else \
	  echo "Flashing CPU1 (NuttX) to 0x08000000 using OpenOCD (CPU2 preserved)..."; \
	  openocd -f interface/stlink.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program nuttx/nuttx.bin 0x08000000 verify reset exit" 2>/dev/null || \
	  openocd -f interface/stlink-dap.cfg -f target/stm32wlx.cfg -c "reset_config none" -c "program nuttx/nuttx.bin 0x08000000 verify reset exit"; \
	fi

# ==========================================================================
# FLASH CPU2 (CORTEX-M0+ RADIO FIRMWARE ONLY: 0x08032000 - 0x0803FFFF)
# This writes ONLY to sectors 100-127 (56 KB max).
# CPU1 at 0x08000000 (sectors 0-99) is strictly preserved!
# ==========================================================================
flash_sat flash_SAT flash_cpu2_sat flash_m0 flash_cpu2:
	$(MAKE) -C $(CPU2_DIR) flash_sat

flash_gs flash_GS flash_cpu2_gs:
	$(MAKE) -C $(CPU2_DIR) flash_gs

build_m0 build_cpu2:
	$(MAKE) -C $(CPU2_DIR) all

# Flash both CPU1 and CPU2 sequentially (without mass erase)
flash_all_sat: flash_cpu1 flash_sat
flash_all_gs:  flash_cpu1 flash_gs

clean_m0 clean_cpu2:
	$(MAKE) -C $(CPU2_DIR) clean

clean_all: clean_m0
	$(MAKE) -C nuttx clean

.PHONY: all flash flash_cpu1 flash_m4 flash_M4 flash_CPU1 flash_gs flash_GS flash_cpu2_gs flash_sat flash_SAT flash_cpu2_sat flash_m0 flash_cpu2 build_m0 build_cpu2 flash_all_sat flash_all_gs clean_m0 clean_cpu2 clean_all

