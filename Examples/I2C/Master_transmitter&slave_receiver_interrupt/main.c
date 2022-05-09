/*!
    \file    main.c
    \brief   master transmitter slave receiver interrupt

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

volatile uint8_t*  i2c_txbuffer = i2c_transmitter;
volatile uint8_t*  i2c_rxbuffer = i2c_receiver;
volatile uint16_t  I2C_nBytes = 16;
volatile ErrStatus status = ERROR;
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
}

void gpio_config(void)
{
    /* connect PB8 to I2C0_SCL */
    /* connect PB9 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    /* remap I2C0 to PB8 and PB9 */
    gpio_pin_remap_config(GPIO_I2C0_REMAP, ENABLE);
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
    /* enable the I2C0 interrupt */
    i2c_interrupt_enable(I2C0, I2C_INT_ERR);
    i2c_interrupt_enable(I2C0, I2C_INT_BUF);
    i2c_interrupt_enable(I2C0, I2C_INT_EV);

    /* configure I2C1 clock */
    i2c_clock_config(I2C1, 100000, I2C_DTCY_2);
    /* configure I2C1 address */
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE,
                         I2C_ADDFORMAT_7BITS, I2C1_SLAVE_ADDRESS7);
    /* enable I2C1 */
    i2c_enable(I2C1);
    /* enable acknowledge */
    i2c_ack_config(I2C1, I2C_ACK_ENABLE);
    /* enable the I2C1 interrupt */
    i2c_interrupt_enable(I2C1, I2C_INT_ERR);
    i2c_interrupt_enable(I2C1, I2C_INT_BUF);
    i2c_interrupt_enable(I2C1, I2C_INT_EV);
}

void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(I2C0_EV_IRQn, 0, 3);
    nvic_irq_enable(I2C0_ER_IRQn, 0, 2);
    nvic_irq_enable(I2C1_EV_IRQn, 0, 4);
    nvic_irq_enable(I2C1_ER_IRQn, 0, 1);
}

int main(void)
{
    int i;
    gd_eval_com_init(EVAL_COM1);

    rcu_config();
    gpio_config();
    i2c_config();
    nvic_config();

    for(i=0;i<16;i++){
        i2c_transmitter[i]=i+0x80;
        i2c_receiver[i] = 0x00;
    }

    printf("I2C Master_transmitter&slave_receiver demo\r\n");

    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);

    while((I2C_nBytes>0));
    while(SUCCESS != status);
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

void I2C0_EV_IRQHandler(void)
{
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SBSEND)){
        /* send slave address */
        i2c_master_addressing(I2C0, I2C1_SLAVE_ADDRESS7, I2C_TRANSMITTER);
    }else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_ADDSEND)){
        /*clear ADDSEND bit */
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_TBE)){
        if(I2C_nBytes>0){
            /* the master sends a data byte */
            i2c_data_transmit(I2C0, *i2c_txbuffer++);
            I2C_nBytes--;
        }else{
            /* the master sends a stop condition to I2C bus */
            i2c_stop_on_bus(I2C0);
            /* disable the I2C0 interrupt */
            i2c_interrupt_disable(I2C0, I2C_INT_ERR);
            i2c_interrupt_disable(I2C0, I2C_INT_BUF);
            i2c_interrupt_disable(I2C0, I2C_INT_EV);
        }
    }
}

void I2C0_ER_IRQHandler(void)
{
    /* no acknowledge received */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_AERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_AERR);
    }

    /* SMBus alert */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBALT)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBALT);
    }

    /* bus timeout in SMBus mode */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBTO)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBTO);
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_OUERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_OUERR);
    }

    /* arbitration lost */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_LOSTARB)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_LOSTARB);
    }

    /* bus error */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_BERR);
    }

    /* CRC value doesn't match */
    if(i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_PECERR)){
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_PECERR);
    }
    /* disable the error interrupt */
    i2c_interrupt_disable(I2C0, I2C_INT_ERR);
    i2c_interrupt_disable(I2C0, I2C_INT_BUF);
    i2c_interrupt_disable(I2C0, I2C_INT_EV);
}

void I2C1_EV_IRQHandler(void)
{
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_ADDSEND)){
        /* clear the ADDSEND bit */
        i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_RBNE)){
        /* if reception data register is not empty ,I2C1 will read a data from I2C_DATA */
        *i2c_rxbuffer++ = i2c_data_receive(I2C1);
    }else if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_STPDET)){
        status = SUCCESS;
        /* clear the STPDET bit */
        i2c_enable(I2C1);
        /* disable I2C1 interrupt */
        i2c_interrupt_disable(I2C1, I2C_INT_ERR);
        i2c_interrupt_disable(I2C1, I2C_INT_BUF);
        i2c_interrupt_disable(I2C1, I2C_INT_EV);
    }
}

void I2C1_ER_IRQHandler(void)
{
    /* no acknowledge received */
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_AERR)){
        i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_AERR);
    }

    /* SMBus alert */
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_SMBALT)){
        i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_SMBALT);
    }

    /* bus timeout in SMBus mode */
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_SMBTO)){
        i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_SMBTO);
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_OUERR)){
       i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_OUERR);
    }

    /* arbitration lost */
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_LOSTARB)){
        i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_LOSTARB);
    }

    /* bus error */
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_BERR)){
        i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_BERR);
    }

    /* CRC value doesn't match */
    if(i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_PECERR)){
        i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_PECERR);
    }
    /* disable the error interrupt */
    i2c_interrupt_disable(I2C1, I2C_INT_ERR);
    i2c_interrupt_disable(I2C1, I2C_INT_BUF);
    i2c_interrupt_disable(I2C1, I2C_INT_EV);
}

