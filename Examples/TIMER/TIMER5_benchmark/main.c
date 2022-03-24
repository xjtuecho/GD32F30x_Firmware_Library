/*!
    \file    main.c
    \brief   transfer data from FLASH to RAM

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
#include <stdio.h>

extern void dhrystone (void);

int main(void)
{
    timer_parameter_struct timer_init_struct = {0};
    volatile uint32_t us = 0;

    /* config debug serial port */
    gd_eval_com_init(EVAL_COM1);
    printf(" GD32F303 TIMER5 benchmark demo.\r\n");

    /* config RCU */
    rcu_periph_clock_enable(RCU_TIMER5);

    /* config PA8 */
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX, GPIO_PIN_8);

    /* config TIMER5 */
    timer_deinit(TIMER5);
    timer_init_struct.prescaler = rcu_clock_freq_get(CK_APB1)*2/1000000UL - 1;
    timer_init_struct.alignedmode = TIMER_COUNTER_EDGE;
    timer_init_struct.counterdirection = TIMER_COUNTER_UP;
    timer_init_struct.period = 0xFFFF;
    timer_init_struct.clockdivision = TIMER_CKDIV_DIV1;
    timer_init_struct.repetitioncounter = 0;
    timer_init(TIMER5, &timer_init_struct);
    /* enable TIMER5 */
    timer_enable(TIMER5);

    printf(" Delay 10ms using TIMER5...\r\n");
    timer_counter_value_config(TIMER5, 0);
    gpio_bit_set(GPIOA, GPIO_PIN_8);
    while(timer_counter_read(TIMER5) < 10000);
    gpio_bit_reset(GPIOA, GPIO_PIN_8);

    printf(" Start Dhrystone 2.1 benchmark...\r\n");
    timer_counter_value_config(TIMER5, 0);
    gpio_bit_set(GPIOA, GPIO_PIN_8);
    dhrystone();
    us = timer_counter_read(TIMER5);
    gpio_bit_reset(GPIOA, GPIO_PIN_8);
    printf(" Stopped, elasped time = %dus...\r\n", us);

    while(1){
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
