/*!
    \file    main.c
    \brief   the example of EXTI which generates an interrupt request and toggle the LED

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
    /* PC13  = LED2 */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP,   GPIO_OSPEED_2MHZ,  GPIO_PIN_13);
    gpio_bit_set(GPIOC, GPIO_PIN_13);
}

void exti_config(void)
{
    nvic_irq_enable(EXTI1_IRQn, 2U, 0U);
    /* connect key EXTI line to key GPIO pin */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_1);
    /* configure key EXTI line */
    exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_1);
}

void EXTI1_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_1)) {
        gpio_bit_write(GPIOC, GPIO_PIN_13, (bit_status)(1-gpio_input_bit_get(GPIOC, GPIO_PIN_13)));
        gpio_bit_write(GPIOA, GPIO_PIN_8, (bit_status)(1-gpio_input_bit_get(GPIOA, GPIO_PIN_8)));
        gpio_bit_write(GPIOA, GPIO_PIN_15, (bit_status)(1-gpio_input_bit_get(GPIOA, GPIO_PIN_15)));
        exti_interrupt_flag_clear(EXTI_1);
    }
}

int main(void)
{
    gpio_config();
    exti_config();
    while (1);
}
