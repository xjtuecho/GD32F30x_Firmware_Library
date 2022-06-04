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

can_trasnmit_message_struct transmit_message;

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /* PA0 = KEY1 */
    gpio_init(GPIOA, GPIO_MODE_IPU,   GPIO_OSPEED_2MHZ,  GPIO_PIN_0);

    /* PB8 = CAN0_RX, PB9 = CAN0_TX*/
    gpio_init(GPIOB, GPIO_MODE_IPU,   GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);
}

void can_config(void)
{
    can_parameter_struct can_parameter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
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
}

void can_init_txmsg(can_trasnmit_message_struct *msg)
{
    uint8_t i = 0;
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
    msg->tx_sfid = 0x123;
    msg->tx_efid = 0;
    msg->tx_ft = CAN_FT_DATA;
    msg->tx_ff = CAN_FF_STANDARD;
    msg->tx_dlen = 8;
    for(i=0; i<8; i++) {
        msg->tx_data[i] = i;
    }
}

void delay(void)
{
    volatile uint16_t nTime = 0x0000;
    for(nTime = 0; nTime < 0xFFFF; nTime++){
    }
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);
    rcu_periph_clock_enable(RCU_CAN0);
    gpio_config();
    can_config();

    /* initialize transmit message */
    can_init_txmsg(&transmit_message);
    printf("please press the KEY1 to transmit message!\r\n");
    while(1) {
        /* waiting for the Tamper key pressed */
        while(RESET == gpio_input_bit_get(GPIOA, GPIO_PIN_0)) {
            printf("transmit data: %x\r\n", transmit_message.tx_data[0]);
            /* transmit message */
            can_message_transmit(CAN0, &transmit_message);
            delay();
            /* waiting for KEY1 up */
            while(RESET == gpio_input_bit_get(GPIOA, GPIO_PIN_0));
            transmit_message.tx_data[0]++;
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
