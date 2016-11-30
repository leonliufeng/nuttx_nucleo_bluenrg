/*******************************************************************************
 *  configs/nucleo-f4x1re/src/stm32_bnrg.c
 *
 *  copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Leon Liu <liujianji@yuewutech.com>
 *             
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
 ******************************************************************************/

/************************************************************************************
 * Included Files
 *************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
//#include <nuttx/wireless/wireless.h>
//#include <nuttx/wireless/cc3000.h>
//#include <nuttx/wireless/cc3000/include/cc3000_upif.h>

#include "stm32.h"
#include "stm32_it.h"
#include "nucleo-f4x1re.h"

#include <arch/board/drivers/stm32f4xx_hal_driver/inc/stm32f4xx_hal.h>
#include <arch/board/drivers/bsp/stm32f4xx-nucleo/stm32f4xx_nucleo.h>
#include <arch/board/drivers/stm32f4xx_hal_driver/inc/stm32f4xx_hal_conf.h>
//#include <arch/board/drivers/stm32f4xx_hal_driver/inc/stm32f4xx_hal_gpio.h>
#include <arch/board/drivers/bsp/stm32f4xx-nucleo/stm32f4xx_nucleo_bluenrg.h>
/****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

volatile uint32_t ms_counter = 0;
volatile uint8_t button_event = 0;


/****************************************************************************
 * Private Types
 ****************************************************************************/




/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/





/****************************************************************************
 *  Private Functions
 *****************************************************************************/
int BNRG_SPI_EXTI_IRQHandler_Slim(int irq, FAR void *context)
{
   // printf("%s.\n", __FUNCTION__);
    BNRG_SPI_EXTI_IRQHandler();
    return 0;
}

void BNRG_SPI_EXTI_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BNRG_SPI_EXTI_PIN);
}


/**
 *   * @brief  This function handles the Push Button interrupt request.
 *     * @param  None
 *       * @retval None
 *         */
void PUSH_BUTTON_EXTI_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
            
//    button_event = 1;
}



/* IRQ/GPIO access callbacks.  These operations all hidden behind
 *
 * callbacks to isolate the CC3000 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the CC3000 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

struct cc3000_config_s
{
    uint32_t spi_mode;
};
struct stm32_config_s
{
    struct cc3000_config_s dev;
    xcpt_t handler;
};
static int bnrg_attach_irq(FAR struct cc3000_config_s *state, xcpt_t handler)
{
    FAR struct stm32_config_s *priv = (FAR struct stm32_config_s *)state;

    /* Just save the handler for use when the interrupt is enabled */

    priv->handler = handler;
    return OK;
}

void bnrg_enable_irq(/*FAR struct cc3000_config_s *state, */bool enable)
{
    //FAR struct stm32_config_s *priv = (FAR struct stm32_config_s *)state;

    /* The caller should not attempt to enable interrupts if the handler
     * has not yet been 'attached'
     */

    //DEBUGASSERT(priv->handler || !enable);
    /* Attach and enable, or detach and disable */

    iinfo("enable:%d\n", enable);
    if (enable)
    {
            
       (void)stm32_gpiosetevent(GPIO_SPI1_BNRG_INT, true, false, false, BNRG_SPI_EXTI_IRQHandler_Slim);
    }
    else
    {
       (void)stm32_gpiosetevent(GPIO_SPI1_BNRG_INT, false, false, false, NULL);
    }
}

static void bnrg_enable_power(FAR struct cc3000_config_s *state, bool enable)
{
    iinfo("enable:%d\n", enable);

    /* Active high enable */

    //stm32_gpiowrite(GPIO_WIFI_EN, enable);
}

static void bnrg_select(FAR struct cc3000_config_s *state, bool enable)
{
    iinfo("enable:%d\n", enable);

    /* Active high enable */

    stm32_gpiowrite(GPIO_SPI1_BNRG_CS, enable);
}

static void bnrg_clear_irq(FAR struct cc3000_config_s *state)
{
    /* Does nothing */
}

static bool bnrg_read_irq(FAR struct cc3000_config_s *state)
{
    /* Active low*/

    return  stm32_gpioread(GPIO_SPI1_BNRG_INT) ? false : true;
}

