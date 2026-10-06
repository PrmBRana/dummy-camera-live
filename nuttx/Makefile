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

SAT_M0_DIR ?= ../JC1/cpu2

# Flash Satellite Cortex-M4 (NuttX) only (0x08000000)
flash_cpu1 flash_m4 flash_M4 flash_CPU1: flash

# Flash JC1 Satellite Cortex-M0+ from JC1/cpu2 (0x08032000)
flash_sat flash_SAT flash_cpu2_sat flashsatM0 flash_satM0 flash_m0 flash_M0 flash_cpu2 flash_CPU2:
	$(MAKE) -C $(SAT_M0_DIR) flash

build_sat_m0:
	$(MAKE) -C $(SAT_M0_DIR) all

clean_sat_m0:
	$(MAKE) -C $(SAT_M0_DIR) clean

# Flash both Satellite cores sequentially (M4 first at 0x08000000, then M0+ at 0x08032000)
flash_all_sat: flash_cpu1 flash_sat

.PHONY: flash_cpu1 flash_m4 flash_M4 flash_CPU1 flash_sat flash_SAT flash_cpu2_sat flashsatM0 flash_satM0 flash_m0 flash_M0 flash_cpu2 flash_CPU2 build_sat_m0 clean_sat_m0 flash_all_sat

endif

