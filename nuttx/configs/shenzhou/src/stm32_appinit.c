/****************************************************************************
 * config/shenzhou/src/stm32_appinit.c
 *
 *   Copyright (C) 2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>

#include "stm32.h"
#include "shenzhou.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#define HAVE_MMCSD    1
#define HAVE_USBDEV   1
#define HAVE_USBHOST  1
#define HAVE_W25      1

/* Configuration ************************************************************/
/* SPI1 connects to the SD CARD (and to the SPI FLASH) */

#define STM32_MMCSDSPIPORTNO   1  /* SPI1 */
#define STM32_MMCSDSLOTNO      0  /* Only one slot */

#ifndef CONFIG_STM32_SPI1
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_MMCSD
#endif

/* Default MMC/SD minor number */

#ifdef HAVE_MMCSD
#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif

/* Default MMC/SD SLOT number */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != STM32_MMCSDSLOTNO
#    error "Only one MMC/SD slot:  Slot 0"
#  endif

/* Verify configured SPI port number */

#  if defined(CONFIG_NSH_MMCSDSPIPORTNO) && CONFIG_NSH_MMCSDSPIPORTNO != STM32_MMCSDSPIPORTNO
#    error "Only one MMC/SD port:  SPI1"
#  endif
#endif

/* Can't support the W25 device if it SPI1 or W25 support is not enabled */

#if !defined(CONFIG_STM32_SPI1) || !defined(CONFIG_MTD_W25)
#  undef HAVE_W25
#endif

/* Can't support W25 features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_W25
#endif

/* Default W25 minor number */

#if defined(HAVE_W25) && !defined(CONFIG_NSH_W25MINOR)
#  define CONFIG_NSH_W25MINOR 0
#endif

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device is USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#if defined(HAVE_MMCSD) || defined(HAVE_USBHOST) || defined(HAVE_W25)
  int ret;
#endif

  /* Initialize and register the W25 FLASH file system. */

#ifdef HAVE_W25
  ret = stm32_w25initialize(CONFIG_NSH_W25MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize W25 minor %d: %d\n",
              CONFIG_NSH_W25MINOR, ret);
      return ret;
    }
#endif

  /* Initialize the SPI-based MMC/SD slot */

#ifdef HAVE_MMCSD
  ret = stm32_sdinitialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot %d: %d\n",
              STM32_MMCSDSLOTNO, ret);
      return ret;
    }
#endif

  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

#ifdef HAVE_USBHOST
  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
