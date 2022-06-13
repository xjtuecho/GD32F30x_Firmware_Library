/*!
    \file    main.c
    \brief   keyboard polling mode

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "gd32f303c_eval.h"

volatile uint32_t sysTickTimer = 0;

void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

void SysTick_Handler(void)
{
    sysTickTimer++;
}

//  delays number of tick Systicks (happens every 1 ms)
void Delay(uint32_t dlyTicks)
{
    uint32_t curTicks;

    curTicks = sysTickTimer;
    while ((sysTickTimer - curTicks) < dlyTicks) {
        __NOP();
    }
}

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);

    /* PA0 = KEY1 */
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ,  GPIO_PIN_0);
    /* PA1 = KEY2 */
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ,  GPIO_PIN_1);

    /* PA8  = LED3 */
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,  GPIO_PIN_8);
    gpio_bit_reset(GPIOA, GPIO_PIN_8);

    /* PA15 = LED4 */
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,  GPIO_PIN_15);
    gpio_bit_reset(GPIOA, GPIO_PIN_15);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    /* PC13  = LED */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP,   GPIO_OSPEED_2MHZ,  GPIO_PIN_13);
    gpio_bit_set(GPIOC, GPIO_PIN_13);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    systick_config();
    gpio_config();

    while(1){
        /* check whether the button is pressed */
        if(RESET == gpio_input_bit_get(GPIOA, GPIO_PIN_0)){
            Delay(100);

            /* check whether the button is pressed */
            if(RESET == gpio_input_bit_get(GPIOA, GPIO_PIN_0)){
                if(RESET == gpio_input_bit_get(GPIOC, GPIO_PIN_13)){
                    gpio_bit_reset(GPIOA, GPIO_PIN_8);
                    gpio_bit_reset(GPIOA, GPIO_PIN_15);
                    gpio_bit_set(GPIOC, GPIO_PIN_13);
                } else {
                    gpio_bit_set(GPIOA, GPIO_PIN_8);
                    gpio_bit_set(GPIOA, GPIO_PIN_15);
                    gpio_bit_reset(GPIOC, GPIO_PIN_13);
                }
            }
        }
    }
}
