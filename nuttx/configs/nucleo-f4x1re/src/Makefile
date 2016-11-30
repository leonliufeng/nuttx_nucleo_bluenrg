############################################################################
# configs/nucleo-f4x1re/src/Makefile
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = stm32_boot.c stm32_spi.c stm32_bnrg.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  stm32_buttons.c
endif

ifeq ($(CONFIG_WL_CC3000),y)
CSRCS += stm32_wireless.c
ifeq ($(CONFIG_CC3000_PROBES),)
CSRCS += stm32_io.c
endif
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_adc.c
ifeq ($(CONFIG_AJOYSTICK),y)
CSRCS += stm32_ajoystick.c
endif
endif

ifeq ($(CONFIG_QENCODER),y)
CSRCS += stm32_qencoder.c
endif

ifeq ($(CONFIG_NSH_LIBRARY),y)
CSRCS += stm32_appinit.c
endif

CUR_DIR := ${shell pwd | sed -e 's/ /\\ /g'}

NRG_HAL = $(CUR_DIR)/drivers/stm32f4xx_hal_driver/src
NRG_CMSIS = $(CUR_DIR)/drivers/cmsis/device/st/stm32f4xx/source/templates
NRG_BSP_4XX = $(CUR_DIR)/drivers/bsp/stm32f4xx-nucleo
NRG_BSP_IDB4A1 = $(CUR_DIR)/drivers/bsp/x-nucleo-idb04a1

MID_HCI = $(CUR_DIR)/middlewares/st/stm32_bluenrg/simplebluenrg_hci/hci
MID_HCI_CTL = $(MID_HCI)/controller
MID_INTF = $(CUR_DIR)/middlewares/st/stm32_bluenrg/interface
MID_UTIL = $(CUR_DIR)/middlewares/st/stm32_bluenrg/simplebluenrg_hci/utils


CSRCS += $(NRG_BSP_4XX)/stm32f4xx_nucleo.c $(NRG_BSP_IDB4A1)/stm32_bluenrg_ble.c $(NRG_CMSIS)/system_stm32f4xx.c $(NRG_HAL)/stm32f4xx_hal.c  $(NRG_HAL)/stm32f4xx_hal_adc.c $(NRG_HAL)/stm32f4xx_hal_adc_ex.c $(NRG_HAL)/stm32f4xx_hal_cortex.c $(NRG_HAL)/stm32f4xx_hal_dma.c $(NRG_HAL)/stm32f4xx_hal_flash.c $(NRG_HAL)/stm32f4xx_hal_flash_ex.c $(NRG_HAL)/stm32f4xx_hal_gpio.c $(NRG_HAL)/stm32f4xx_hal_pwr.c $(NRG_HAL)/stm32f4xx_hal_pwr_ex.c $(NRG_HAL)/stm32f4xx_hal_rcc.c $(NRG_HAL)/stm32f4xx_hal_rcc_ex.c $(NRG_HAL)/stm32f4xx_hal_msp.c $(NRG_HAL)/stm32f4xx_hal_spi.c $(NRG_HAL)/stm32f4xx_hal_uart.c $(MID_HCI)/hci.c $(MID_HCI_CTL)/bluenrg_gap_aci.c $(MID_HCI_CTL)/bluenrg_gatt_aci.c $(MID_HCI_CTL)/bluenrg_hal_aci.c $(MID_HCI_CTL)/bluenrg_l2cap_aci.c $(MID_HCI_CTL)/bluenrg_utils_small.c $(MID_INTF)/bluenrg_itf_template.c $(MID_UTIL)/gp_timer.c $(MID_UTIL)/ble_list.c $(MID_UTIL)/osal.c


CFLAGS += -I ../includes/drivers/stm32f4xx_hal_driver/inc
CFLAGS += -I ../includes/drivers/bsp/stm32f4xx-nucleo
CFLAGS += -I ../includes/drivers/bsp/x-nucleo-idb04a1
CFLAGS += -I ../includes/drivers/cmsis/device/st/stm32f4xx/include
CFLAGS += -I ../includes/drivers/cmsis/include
CFLAGS += -I ../includes/middlewares/st/stm32_bluenrg/interface
CFLAGS += -I ../includes/middlewares/st/stm32_bluenrg/simplebluenrg_hci


include $(TOPDIR)/configs/Board.mk
