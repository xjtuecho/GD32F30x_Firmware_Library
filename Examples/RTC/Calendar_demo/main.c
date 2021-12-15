/*!
    \file    main.c
    \brief   RTC calendar 

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

/* enter the second interruption,set the second interrupt flag to 1 */
__IO uint32_t timedisplay;

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
    rtc_interrupt_enable(RTC_INT_SECOND);
    rtc_lwoff_wait();
    /* set RTC prescaler: set RTC period to 1s */
    rtc_prescaler_set(32767);
    rtc_lwoff_wait();
}

/*!
    \brief      get numeric values from the hyperterminal
    \param[in]  value: input value from the hyperterminal
    \param[out] none
    \retval     input value in BCD mode
*/
uint8_t usart_scanf(uint32_t value)
{
    uint32_t index = 0;
    uint32_t tmp[2] = {0, 0};

    while (index < 2){
        /* loop until RBNE = 1 */
        while (usart_flag_get(USART0, USART_FLAG_RBNE) == RESET);
        tmp[index++] = (usart_data_receive(USART0));

        if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39)){
            printf("\n\rPlease enter valid number between 0 and 9\n");
            index--;
        }
    }
    /* calculate the Corresponding value */
    index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
    /* check */
    if (index > value){
        printf("\n\rPlease enter valid number between 0 and %d\n", value);
        return 0xFF;
    }
    return index;
}

/*!
    \brief      return the time entered by user, using Hyperterminal
    \param[in]  none
    \param[out] none
    \retval     current time of RTC counter value
*/
uint32_t time_regulate(void)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    printf("\r\n==============Time Settings=====================================");
    printf("\r\n  Please Set Hours");

    while (tmp_hh == 0xFF){
        tmp_hh = usart_scanf(23);
    }
    printf(":  %d", tmp_hh);
    printf("\r\n  Please Set Minutes");
    while (tmp_mm == 0xFF){
        tmp_mm = usart_scanf(59);
    }
    printf(":  %d", tmp_mm);
    printf("\r\n  Please Set Seconds");
    while (tmp_ss == 0xFF){
        tmp_ss = usart_scanf(59);
    }
    printf(":  %d", tmp_ss);

    /* return the value  store in RTC counter register */
    return((tmp_hh*3600 + tmp_mm*60 + tmp_ss));
}

void time_display(uint32_t timevar)
{
    printf(" Time: %0.2d:%0.2d:%0.2d\r\n",
         timevar / 3600,
         (timevar % 3600) / 60, 
         (timevar % 3600) % 60);
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);

    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(RTC_IRQn,1,0);
	
    printf( "\r\n This is a RTC demo...... \r\n" );

    if (bkp_read_data(BKP_DATA_0) != 0xA5A5){
        /* backup data register value is not correct or not yet programmed
        (when the first time the program is executed) */
        printf("\r\nThis is a RTC demo!\r\n");
        printf("\r\n\n RTC not yet configured....");

        rtc_configuration();

        printf("\r\n RTC configured....");

        rtc_lwoff_wait();
        rtc_counter_set(time_regulate());
        rtc_lwoff_wait();

        bkp_write_data(BKP_DATA_0, 0xA5A5);
    }else{
        /* check if the power on reset flag is set */
        if (rcu_flag_get(RCU_FLAG_PORRST) != RESET){
            printf("\r\n\n Power On Reset occurred....");
        }else if (rcu_flag_get(RCU_FLAG_SWRST) != RESET){
            /* check if the pin reset flag is set */
            printf("\r\n\n External Reset occurred....");
        }

        rcu_periph_clock_enable(RCU_PMU);
        pmu_backup_write_enable();

        printf("\r\n No need to configure RTC....");
        rtc_register_sync_wait();

        rtc_interrupt_enable(RTC_INT_SECOND);
        rtc_lwoff_wait();
    }

#ifdef RTCCLOCKOUTPUT_ENABLE
    /* enable PMU and BKPI clocks */
    rcu_periph_clock_enable(RCU_BKPI);
    rcu_periph_clock_enable(RCU_PMU);
    /* allow access to BKP domain */
    pmu_backup_write_enable();

    /* disable the tamper pin */
    bkp_tamper_detection_disable();

    /* enable RTC clock output on tamper Pin */
    bkp_rtc_calibration_output_enable();
#endif

    rcu_all_reset_flag_clear();
    printf("\n\r");
    while (1){
        /* if 1s has paased, display current time*/
        if (timedisplay == 1){
            time_display(rtc_counter_get());
            timedisplay = 0;
        }
    }
}

void RTC_IRQHandler(void)
{
    if (rtc_flag_get(RTC_FLAG_SECOND) != RESET){
        rtc_flag_clear(RTC_FLAG_SECOND);
        timedisplay = 1;
        rtc_lwoff_wait();
        /* reset RTC counter when time is 23:59:59 */
        if (rtc_counter_get() == 24*60*60){
            rtc_counter_set(0x0);
            rtc_lwoff_wait();
        }
    }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));

    return ch;
}
