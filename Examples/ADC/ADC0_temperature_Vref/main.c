/*!
    \file    main.c
    \brief   ADC channel of temperature and Vref

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
#include <stdio.h>
#include "gd32f303c_eval.h"

uint16_t adc_raw[2];
uint16_t temperature;
uint16_t vref_value;

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

void rcu_config(void)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
}

void adc_config(void)
{
    /* ADC SCAN function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2);


    /* ADC temperature sensor channel config */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_16, ADC_SAMPLETIME_239POINT5);
    /* ADC internal reference voltage channel config */
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5);

    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);

    /* ADC temperature and Vrefint enable */
    adc_tempsensor_vrefint_enable();


    /* enable ADC interface */
    adc_enable(ADC0);
    Delay(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);
    /* configure systick */
    systick_config();
    /* system clocks configuration */
    rcu_config();
    /* ADC configuration */
    adc_config();

    while(1){
        /* ADC software trigger enable */
        adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
        /* delay a time in milliseconds */
        Delay(2000);
        adc_raw[0] = ADC_IDATA0(ADC0);
        adc_raw[1] = ADC_IDATA1(ADC0);
        /* value convert */
        temperature = (1450 - (adc_raw[0]*3300L>>12)) * 10 / 41 + 250;
        vref_value = adc_raw[1]*3300L>>12;

        /* value print */
        printf(" the temperature data is %d.%d oC\r\n", temperature/10, temperature%10);
        printf(" the reference voltage data is %dmV \r\n", vref_value);
        printf(" \r\n");
    }

}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
