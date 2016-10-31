############################################################################
# apps/examples/bluenrg_sensor/Makefile
#
#   Copyright (C) 2008, 2010-2013 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# Directories
BLUENRG_DIR = $(APPDIR)/examples/bluenrg_sensor
BLUENRG_SRC = $(BLUENRG_DIR)/src
BLUENRG_INC = $(BLUENRG_DIR)/include

MID_HCI = $(BLUENRG_DIR)/middlewares/st/stm32_bluenrg/simplebluenrg_hci/hci
MID_HCI_CTL = $(MID_HCI)/controller
MID_INTF = $(BLUENRG_DIR)/middlewares/st/stm32_bluenrg/interface
MID_UTIL = $(BLUENRG_DIR)/middlewares/st/stm32_bluenrg/simplebluenrg_hci/utils

MID_HCI_INC = $(BLUENRG_DIR)/middlewares/st/stm32_bluenrg/simplebluenrg_hci/includes
MID_INTF_INC = $(MID_INTF)

# Bluenrg sensor demo! built-in application info
CONFIG_EXAMPLES_BLUENRG_SENSOR_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_BLUENRG_SENSOR_STACKSIZE ?= 2048

APPNAME = bluenrg_sensor
PRIORITY = $(CONFIG_EXAMPLES_BLUENRG_SENSOR_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_BLUENRG_SENSOR_STACKSIZE)

#CPPFLAGS += $(BLUENRG_MIDDLEWARE)/includes
#CFLAGS += $(BLUENRG_MIDDLEWARE)/includes
#CXXFLAGS += $(BLUENRG_MIDDLEWARE)/includes
CFLAGS += -I$(BLUENRG_INC)
#CFLAGS += -I$(ARCH_INC)/drivers/stm32f4xx_hal_driver/inc
#CFLAGS += -I$(ARCH_INC)/drivers/bsp/stm32f4xx-nucleo
#CFLAGS += -I arch/board/drivers/bsp/x-nucleo-idb04a1
#CFLAGS += -I$(ARCH_INC)/drivers/cmsis/device/st/stm32f4xx/include
#CFLAGS += -I$(ARCH_INC)/drivers/cmsis/include
#CFLAGS += -I$(MID_HCI_INC)
#CFLAGS += -I$(MID_INTF_INC)

#vpath %.h ../middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes
VPATH = middlewares

# Bluenrg sensor demo! Files 
ASRCS = 
CSRCS = src/bluenrg_interface.c src/clock.c src/cube_hal_f4.c src/stm32xx_it.c src/sensor_service.c src/stm32f4xx_hal_msp.c 
#$(MID_HCI)/hci.c $(MID_HCI_CTL)/bluenrg_gap_aci.c $(MID_HCI_CTL)/bluenrg_gatt_aci.c $(MID_HCI_CTL)/bluenrg_hal_aci.c $(MID_HCI_CTL)/bluenrg_l2cap_aci.c $(MID_HCI_CTL)/bluenrg_utils_small.c $(MID_INTF)/bluenrg_itf_template.c $(MID_UTIL)/gp_timer.c $(MID_UTIL)/ble_list.c $(MID_UTIL)/osal.c
MAINSRC = src/bluenrg_sensor_main.c

CONFIG_EXAMPLES_BLUENRG_SENSOR_PROGNAME ?= bluenrg_sensor$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_BLUENRG_SENSOR_PROGNAME)

include $(APPDIR)/Application.mk
