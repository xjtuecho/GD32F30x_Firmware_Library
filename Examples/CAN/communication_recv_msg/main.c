/*!
    \file    main.c
    \brief   communication_Loopback in normal mode

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

can_receive_message_struct  receive_message;
volatile FlagStatus receive_flag = RESET;

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_AF);

    /* PD0 = CAN0_RX, PD1 = CAN0_TX*/
    gpio_init(GPIOD, GPIO_MODE_IPU,   GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_pin_remap_config(GPIO_CAN_FULL_REMAP, ENABLE);
}

void can_config(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    can_deinit(CAN0);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.no_auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    /* configure baudrate to 1Mbps */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_4TQ;
    can_parameter.prescaler = 6;
    can_init(CAN0, &can_parameter);

    /* enable CAN receive FIFO1 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INT_RFNE1);

    /* initialize CAN0 filter number */
    can_filter.filter_number = 0;

    /* initialize filter */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}

void nvic_config(void)
{
    nvic_irq_enable(CAN0_RX1_IRQn, 0, 0);
}

void CAN0_RX1_IRQHandler(void)
{
    if(can_receive_message_length_get(CAN0, CAN_FIFO1) > 0) {
        can_message_receive(CAN0, CAN_FIFO1, &receive_message);
        receive_flag = SET;
    }
}

void can_print_rxmsg(can_receive_message_struct *msg)
{
    uint8_t i = 0;
    if(CAN_FT_DATA == msg->rx_ft) {
        printf((CAN_FF_STANDARD == msg->rx_ff)
            ? " DATA MSG ID=0x%04X LEN=%d DATA="
            : " DATA MSG ID=0x%08X LEN=%d DATA=",
            (CAN_FF_STANDARD == msg->rx_ff) ? msg->rx_sfid : msg->rx_efid,
            msg->rx_dlen);
        for(i=0; i<msg->rx_dlen; i++)
            printf("%02X ", msg->rx_data[i]);
    } else {
        printf((CAN_FF_STANDARD == msg->rx_ff)
            ? " REMOTE MSG ID=0x%04X"
            : " REMOTE MSG ID=0x%08X",
            (CAN_FF_STANDARD == msg->rx_ff) ? msg->rx_sfid : msg->rx_efid);
    }
    printf("\r\n");
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);
    rcu_periph_clock_enable(RCU_CAN0);
    gpio_config();
    nvic_config();
    can_config();
    printf("CAN0 receive message demo.\r\n");
    while(1) {
        if(SET == receive_flag) {
            can_print_rxmsg(&receive_message);
            receive_flag = RESET;
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
