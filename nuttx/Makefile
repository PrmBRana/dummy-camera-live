############################################################################
# Makefile
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# Check if the system has been configured

ifeq ($(wildcard .config),)
.DEFAULT default:
	@echo "NuttX has not been configured!"
	@echo "To configure the project:"
	@echo "  tools/configure.sh <config>"
	@echo "For a list of available configurations:"
	@echo "  tools/configure.sh -L"
else
include .config

# Include the correct Makefile for the selected architecture.

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
include tools/Win.mk
else
include tools/Unix.mk
endif

CPU2_DIR ?= $(shell if [ -d ../JC2/cpu2 ]; then echo ../JC2/cpu2; elif [ -d ../CPU2 ]; then echo ../CPU2; elif [ -d boards/arm/stm32wl5/nucleo-wl55jc/CPU2 ]; then echo boards/arm/stm32wl5/nucleo-wl55jc/CPU2; else echo CPU2; fi)

# Flash Cortex-M4 (NuttX) only (0x08000000)
flash_cpu1 flash_m4 flash_M4 flash_CPU1: flash

# Flash Cortex-M0+ from JC2/cpu2 (0x08032000)
flash_sat flash_SAT flash_cpu2_sat flash_m0 flash_cpu2:
	$(MAKE) -C $(CPU2_DIR) flash_sat

flash_gs flash_GS flash_cpu2_gs:
	$(MAKE) -C $(CPU2_DIR) flash_gs

build_m0 build_cpu2:
	$(MAKE) -C $(CPU2_DIR) all

clean_m0 clean_cpu2:
	$(MAKE) -C $(CPU2_DIR) clean

# Flash both sequentially (M4 first, then M0+, no mass erase)
flash_all_sat: flash_cpu1 flash_sat
flash_all_gs:  flash_cpu1 flash_gs

.PHONY: flash_cpu1 flash_m4 flash_M4 flash_CPU1 flash_gs flash_GS flash_cpu2_gs flash_sat flash_SAT flash_cpu2_sat flash_m0 flash_cpu2 build_m0 build_cpu2 clean_m0 clean_cpu2 flash_all_sat flash_all_gs

endif

