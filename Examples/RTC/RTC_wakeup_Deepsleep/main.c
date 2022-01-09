/*!
    \file    main.c
    \brief   deepsleep wakeup through exti interrupt

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

void rtc_configuration(void)
{
    rcu_periph_clock_enable(RCU_BKPI);
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
    bkp_deinit();
    rcu_osci_on(RCU_LXTAL);
    rcu_osci_stab_wait(RCU_LXTAL);
    rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();
    rtc_lwoff_wait();
    rtc_interrupt_enable(RTC_INT_ALARM);
    rtc_lwoff_wait();
    /* set RTC prescaler: set RTC period to 1s */
    rtc_prescaler_set(32768-1);
    rtc_lwoff_wait();
}

void setup_rtc_alarm(void)
{
    uint32_t cnt = rtc_counter_get();
    printf("RTC_CNT=%us ", cnt);
	cnt += 10;
    rtc_lwoff_wait();
    rtc_alarm_config(cnt - 1);
    rtc_lwoff_wait();
    printf("Set RTC_ALARM=%us\r\n", cnt);
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);

    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

    rtc_configuration();

    exti_interrupt_flag_clear(EXTI_17);
    exti_init(EXTI_17, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    nvic_irq_enable(RTC_Alarm_IRQn, 1, 0);

    printf("RTC wakeup DeepSleep Demo...\r\n");

    while(1){
        setup_rtc_alarm();
        printf("Goto DeepSleep...\r\n ");
        pmu_to_deepsleepmode(PMU_LDO_NORMAL, WFI_CMD);
        rtc_register_sync_wait();
        printf("Wakeup from DeepSleep, RTC_CNT=%us\r\n\r\n", rtc_counter_get());
    }
}

void RTC_Alarm_IRQHandler(void)
{
    if(rtc_flag_get(RTC_FLAG_ALARM) != RESET)
    {
        rtc_lwoff_wait();
        rtc_flag_clear(RTC_FLAG_ALARM);
        rtc_lwoff_wait();
    }

    if(pmu_flag_get(PMU_FLAG_WAKEUP) != RESET)
    {
        pmu_flag_clear(PMU_FLAG_RESET_WAKEUP);
    }

    exti_interrupt_flag_clear(EXTI_17);

    SystemInit();
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));

    return ch;
}
