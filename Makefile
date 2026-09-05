CPU2_DIR ?= $(shell if [ -d CPU2 ]; then echo CPU2; else echo nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2; fi)

all:
	$(MAKE) -C nuttx

flash_gs flash_GS:
	$(MAKE) -C $(CPU2_DIR) flash_GS

flash_sat flash_SAT:
	$(MAKE) -C $(CPU2_DIR) flash_sat

clean_m0:
	$(MAKE) -C $(CPU2_DIR) clean

.PHONY: all flash_gs flash_GS flash_sat flash_SAT clean_m0

