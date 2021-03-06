/*!
    \file    main.c
    \brief   transfer data from RAM to RAM

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
#include <string.h>
#include <stdio.h>

#define DATANUM                  16

__IO ErrStatus trans_flag1 = ERROR;
__IO ErrStatus trans_flag2 = ERROR;
__IO ErrStatus trans_flag3 = ERROR;
__IO ErrStatus trans_flag4 = ERROR;
uint8_t src_addr[DATANUM]= {
    0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
    0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10
};
uint8_t dest_addr1[DATANUM];
uint8_t dest_addr2[DATANUM];
uint8_t dest_addr3[DATANUM];
uint8_t dest_addr4[DATANUM];

ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length)
{
    while (length--){
        if (*src++ != *dst++){
            return ERROR;
        }
    }
    return SUCCESS;
}

int main(void)
{
    int i = 0;
    dma_parameter_struct dma_init_struct;

    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA0);

    /* config debug serial port */
    gd_eval_com_init(EVAL_COM1);
    printf("GD32F303 DMA RAM to RAM transfer demo.\r\n");

    memset(dest_addr1, 0, DATANUM);
    memset(dest_addr2, 0, DATANUM);
    memset(dest_addr3, 0, DATANUM);
    memset(dest_addr4, 0, DATANUM);

    /* initialize DMA channel1 */
    dma_deinit(DMA0, DMA_CH1);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)dest_addr1;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = DATANUM;
    dma_init_struct.periph_addr = (uint32_t)src_addr;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH1, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH1);
    dma_memory_to_memory_enable(DMA0, DMA_CH1);

    /* initialize DMA channel2 */
    dma_deinit(DMA0, DMA_CH2);
    dma_init_struct.memory_addr = (uint32_t)dest_addr2;
    dma_init(DMA0, DMA_CH2, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH2);
    dma_memory_to_memory_enable(DMA0, DMA_CH2);

    /* initialize DMA channel3 */
    dma_deinit(DMA0, DMA_CH3);
    dma_init_struct.memory_addr = (uint32_t)dest_addr3;
    dma_init(DMA0, DMA_CH3, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);
    dma_memory_to_memory_enable(DMA0, DMA_CH3);

    /* initialize DMA channel4 */
    dma_deinit(DMA0, DMA_CH4);
    dma_init_struct.memory_addr = (uint32_t)dest_addr4;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH4);
    dma_memory_to_memory_enable(DMA0, DMA_CH4);

    /* enable DMA channel1~channel4 */
    dma_channel_enable(DMA0, DMA_CH1);
    dma_channel_enable(DMA0, DMA_CH2);
    dma_channel_enable(DMA0, DMA_CH3);
    dma_channel_enable(DMA0, DMA_CH4);

    /* wait for DMA transfer complete */
    for(i = 0; i < 200; i++);

    /* compare the data of src_addr with data of dest_addr */
    trans_flag1 = memory_compare(src_addr, dest_addr1, DATANUM);
    trans_flag2 = memory_compare(src_addr, dest_addr2, DATANUM);
    trans_flag3 = memory_compare(src_addr, dest_addr3, DATANUM);
    trans_flag4 = memory_compare(src_addr, dest_addr4, DATANUM);

    /* print DMA transfer result */
    printf("DMA_CH1 RAM to RAM transfer %s\r\n",
        (SUCCESS == trans_flag1) ? "passed": "failed");
    printf("DMA_CH2 RAM to RAM transfer %s\r\n",
        (SUCCESS == trans_flag2) ? "passed": "failed");
    printf("DMA_CH3 RAM to RAM transfer %s\r\n",
        (SUCCESS == trans_flag3) ? "passed": "failed");
    printf("DMA_CH4 RAM to RAM transfer %s\r\n",
        (SUCCESS == trans_flag4) ? "passed": "failed");

    while (1);
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
