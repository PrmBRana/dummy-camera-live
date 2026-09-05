all:
	$(MAKE) -C nuttx

flash_gs flash_GS:
	$(MAKE) -C nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2 flash_GS

flash_sat flash_SAT:
	$(MAKE) -C nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2 flash_sat

clean_m0:
	$(MAKE) -C nuttx/boards/arm/stm32wl5/nucleo-wl55jc/CPU2 clean

.PHONY: all flash_gs flash_GS flash_sat flash_SAT clean_m0
