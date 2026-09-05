/****************************************************************************
 * include/nuttx/analog/ads7953.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_ANALOG_ADS7953_H
#define __INCLUDE_NUTTX_ANALOG_ADS7953_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADS7953_NCHANNELS             16

/* Range Selection:
 *   ADS7953_RANGE_1X: 0 to Vref (typically 2.5V)
 *   ADS7953_RANGE_2X: 0 to 2*Vref (typically 5.0V)
 */

#define ADS7953_RANGE_1X              0
#define ADS7953_RANGE_2X              1

/* IOCTL Commands */

#ifndef AN_ADS7953_FIRST
#  define AN_ADS7953_FIRST            (AN_ADS7046_FIRST + AN_ADS7046_NCMDS)
#  define AN_ADS7953_NCMDS            2
#endif

#define ANIOC_ADS7953_SET_RANGE       _ANIOC(AN_ADS7953_FIRST + 0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ads7953_register
 *
 * Description:
 *   Register the ADS7953 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/adc0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             ADS7953
 *   devno   - The SPI bus chip select device ID (e.g. SPIDEV_USER(0))
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads7953_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     uint32_t devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_ADS7953_H */
