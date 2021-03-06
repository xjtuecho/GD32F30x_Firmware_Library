/*!
    \file    main.c
    \brief   master transmitter slave receiver through DMA

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

#define I2C0_SLAVE_ADDRESS7    0x82
#define I2C1_SLAVE_ADDRESS7    0x72

uint8_t i2c_transmitter[16];
uint8_t i2c_receiver[16];
__IO ErrStatus state = ERROR;

ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length)
{
    while(length--){
        if(*src++ != *dst++){
            return ERROR;
        }
    }
    return SUCCESS;
}

void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_I2C1);
    rcu_periph_clock_enable(RCU_DMA0);
}

void gpio_config(void)
{
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    /* connect PB10 to I2C1_SCL */
    /* connect PB11 to I2C1_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11);
}

void i2c_config(void)
{
    /* configure I2C0 clock */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* configure I2C0 address */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE,
                         I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    /* configure I2C1 clock */
    i2c_clock_config(I2C1, 100000, I2C_DTCY_2);
    /* configure I2C1 address */
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE,
                         I2C_ADDFORMAT_7BITS, I2C1_SLAVE_ADDRESS7);
    /* enable I2C1 */
    i2c_enable(I2C1);
    /* enable acknowledge */
    i2c_ack_config(I2C1, I2C_ACK_ENABLE);
}

void dma_config(void)
{
    dma_parameter_struct dma_init_struct;

    /* initialize DMA channel4 */
    dma_deinit(DMA0, DMA_CH4);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)i2c_receiver;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 16;
    dma_init_struct.periph_addr = 0x40005810;    // I2C1_DATA_ADDRESS
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);

    /* initialize DMA channel5 */
    dma_deinit(DMA0, DMA_CH5);
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)i2c_transmitter;
    dma_init_struct.periph_addr = 0x40005410;    // I2C0_DATA_ADDRESS
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH5, &dma_init_struct);
}

int main(void)
{
    int i;
    gd_eval_com_init(EVAL_COM1);

    rcu_config();
    gpio_config();
    i2c_config();
    dma_config();

    for(i=0;i<16;i++){
        i2c_transmitter[i]=i+0x80;
        i2c_receiver[i] = 0x00;
    }

    printf("I2C Master_transmitter&slave_receiver demo\r\n");

    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus*/
    i2c_master_addressing(I2C0, I2C1_SLAVE_ADDRESS7, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    while(!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);

    /* enable I2C1 DMA */
    i2c_dma_enable(I2C1, I2C_DMA_ON);
    /* enable I2C0 DMA */
    i2c_dma_enable(I2C0, I2C_DMA_ON);
    /* enable DMA0 channel4 */
    dma_channel_enable(DMA0, DMA_CH4);
    /* enable DMA0 channel5 */
    dma_channel_enable(DMA0, DMA_CH5);

    /* DMA0 channel4 full transfer finish flag */
    while(!dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF));
    /* DMA0 channel5 full transfer finish flag */
    while(!dma_flag_get(DMA0, DMA_CH5, DMA_FLAG_FTF));

    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0)&0x0200);
    while(!i2c_flag_get(I2C1, I2C_FLAG_STPDET));
    /* clear the STPDET bit */
    i2c_enable(I2C1);
    state = memory_compare(i2c_transmitter, i2c_receiver, 16);
    if(SUCCESS == state){
        printf("I2C Master_transmitter&slave_receiver passed!\r\n");
    }else{
        printf("I2C Master_transmitter&slave_receiver failed!\r\n");
    }
    while(1);
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
