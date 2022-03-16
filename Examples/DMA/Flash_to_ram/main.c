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
#include <string.h>
#include "gd32f303c_eval.h"
#include <stdio.h>

#define TRANSFER_NUM                     0x400                     /* Configuration value in bytes */
uint8_t g_destbuf[TRANSFER_NUM];
__IO uint32_t g_dmacomplete_flag = 0;

ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint32_t length)
{
    while (length--){
        if (*src++ != *dst++)
            return ERROR;
    }
    return SUCCESS;
}

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA0);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    nvic_irq_enable(DMA0_Channel0_IRQn, 0, 0);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    dma_parameter_struct dma_init_struct;

    /* config debug serial port */
    gd_eval_com_init(EVAL_COM1);
    printf("GD32F303 DMA FLASH to RAM transfer demo.\r\n");

    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA0);
    nvic_irq_enable(DMA0_Channel0_IRQn, 0, 0);

    memset(g_destbuf ,0 ,TRANSFER_NUM);
    
    /* DMA channel0 initialize */
    dma_deinit(DMA0, DMA_CH0);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)g_destbuf;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = TRANSFER_NUM;
    dma_init_struct.periph_addr = (uint32_t)FLASH_BASE;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_init_struct);
    /* DMA channel0 mode configuration */
    dma_circulation_disable(DMA0, DMA_CH0);
    dma_memory_to_memory_enable(DMA0, DMA_CH0);
    /* DMA channel0 interrupt configuration */
    dma_interrupt_enable(DMA0, DMA_CH0, DMA_INT_FTF);
    /* enable DMA transfer */
    dma_channel_enable(DMA0, DMA_CH0);

    /* wait DMA interrupt */
    while(0 == g_dmacomplete_flag);
    
    /* compare destdata with transdata */
    if(memory_compare((uint8_t*)FLASH_BASE, g_destbuf, TRANSFER_NUM))
        printf("FLASH->RAM dma transfer passed\r\n");
    else
        printf("FLASH->RAM dma transfer failed\r\n");

    while(1);
}

void DMA0_Channel0_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_FTF)){
        g_dmacomplete_flag = 1;
        dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_G);
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
