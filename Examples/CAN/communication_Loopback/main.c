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

volatile ErrStatus test_flag_polling;
volatile ErrStatus test_flag_interrupt;

void can_loopback_init(void)
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
    can_parameter.working_mode = CAN_LOOPBACK_MODE;

    /* configure baudrate to 125kbps */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_4TQ;
    can_parameter.prescaler = 48;
    can_init(CAN0, &can_parameter);

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

ErrStatus can_loopback_polling(void)
{
    can_trasnmit_message_struct transmit_message;
    can_receive_message_struct  receive_message;
    uint32_t timeout = 0xFFFF;
    uint8_t transmit_mailbox = 0;

    /* initialize CAN */
    can_loopback_init();

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
    transmit_message.tx_sfid = 0x11;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 2;
    transmit_message.tx_data[0] = 0xAB;
    transmit_message.tx_data[1] = 0xCD;

    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &receive_message);

    /* transmit message */
    transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
    /* waiting for transmit completed */
    while(CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)
        && 0 != timeout){
        timeout--;
    }
    timeout = 0xFFFF;
    /* waiting for receive completed */
    while(can_receive_message_length_get(CAN0, CAN_FIFO1) < 1
        && 0 != timeout){
        timeout--;
    }

    /* initialize receive message*/
    receive_message.rx_sfid = 0x00;
    receive_message.rx_ff = 0;
    receive_message.rx_dlen = 0;
    receive_message.rx_data[0] = 0x00;
    receive_message.rx_data[1] = 0x00;
    can_message_receive(CAN0, CAN_FIFO1, &receive_message);

    /* check the receive message */
    if(0x11 == receive_message.rx_sfid
        && CAN_FF_STANDARD == receive_message.rx_ff
        && 2 == receive_message.rx_dlen
        && 0xAB == receive_message.rx_data[0]
        && 0xCD == receive_message.rx_data[1]){
        return SUCCESS;
    }else{
        return ERROR;
    }
}

ErrStatus can_loopback_interrupt(void)
{
    can_trasnmit_message_struct transmit_message;
    uint32_t timeout = 0x0000FFFF;

    /* initialize CAN and filter */
    can_loopback_init();

    /* enable CAN receive FIFO1 not empty interrupt  */
    can_interrupt_enable(CAN0, CAN_INT_RFNE1);

    /* initialize transmit message */
    transmit_message.tx_sfid = 0;
    transmit_message.tx_efid = 0x1234;
    transmit_message.tx_ff = CAN_FF_EXTENDED;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_dlen = 2;
    transmit_message.tx_data[0] = 0xDE;
    transmit_message.tx_data[1] = 0xCA;
    /* transmit a message */
    can_message_transmit(CAN0, &transmit_message);

    /* waiting for receive completed */
    while(SUCCESS != test_flag_interrupt && 0 != timeout){
        timeout--;
    }
    if(0 == timeout){
        test_flag_interrupt = ERROR;
    }

    /* disable CAN receive FIFO1 not empty interrupt  */
    can_interrupt_disable(CAN0, CAN_INTEN_RFNEIE1);

    return test_flag_interrupt;
}

/*!
    \brief      this function handles CAN0 RX1 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN0_RX1_IRQHandler(void)
{
    can_receive_message_struct receive_message;
    /* initialize receive message */
    receive_message.rx_sfid = 0x00;
    receive_message.rx_efid = 0x00;
    receive_message.rx_ff = 0;
    receive_message.rx_dlen = 0;
    receive_message.rx_fi = 0;
    receive_message.rx_data[0] = 0x00;
    receive_message.rx_data[1] = 0x00;

    /* check the receive message */
    can_message_receive(CAN0, CAN_FIFO1, &receive_message);

    if(0x1234 == receive_message.rx_efid
        && CAN_FF_EXTENDED == receive_message.rx_ff
        && 2 == receive_message.rx_dlen
        && 0xDE == receive_message.rx_data[0]
        && 0xCA == receive_message.rx_data[1]){
        test_flag_interrupt = SUCCESS;
    }else{
        test_flag_interrupt = ERROR;
    }
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);
    rcu_periph_clock_enable(RCU_CAN0);
    nvic_irq_enable(CAN0_RX1_IRQn, 0, 0);

    /* loopback of polling */
    test_flag_polling = can_loopback_polling();
    if(SUCCESS == test_flag_polling){
        printf("loopback of polling test is success.\r\n");
    }else{
        printf("loopback of polling test is failed.\r\n");
    }

    /* loopback of interrupt */
    test_flag_interrupt = can_loopback_interrupt();
    if(SUCCESS == test_flag_interrupt){
        printf("loopback of interrupt test is success.\r\n");
    }else{
        printf("loopback of interrupt test is failed.\r\n");
    }
    while (1);
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
