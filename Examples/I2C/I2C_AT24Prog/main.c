/*!
    \file    main.c
    \brief   USART printf

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
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#define I2C0_SLAVE_ADDRESS7     0x50

#define TX_LEN  128
#define RX_LEN  128
#define CMD_LEN  128

uint8_t syscall_buf[256];

volatile uint8_t isCmdOk = 0;
char txBuf[TX_LEN], rxBuf[RX_LEN], cmdBuf[CMD_LEN];
uint16_t txPtr = 0, rxPtr = 0, cmdPtr = 0;

struct syscall_item {
    const char *name;		/* the name of system call */
    const char *desc;		/* description of system call */
    void (*func)(void);		/* the function address of system call */
};

char *syscall_cmd = NULL;
char *syscall_param1 = NULL;
char *syscall_param2 = NULL;

extern const struct syscall_item syscall_table[];
extern const uint8_t syscall_num;

void usart_recv_char(char dat)
{
    txPtr = 0;
    if(isprint(dat)){
        txBuf[txPtr++] = dat;
        if(cmdPtr < CMD_LEN)
            cmdBuf[cmdPtr++] = tolower(dat);
    } else if(0x08 == dat) {
        /* backspace */
        txBuf[txPtr++] = 0x08;
        txBuf[txPtr++] = 0x20;
        txBuf[txPtr++] = 0x08;
        if(cmdPtr > 0)
            cmdPtr--;
    } else if(0x0D == dat) {
        /* return */
        txBuf[txPtr++] = 0x0D;
        txBuf[txPtr++] = 0x0A;
        if(cmdPtr < CMD_LEN) {
            cmdBuf[cmdPtr++] = 0x00;
            cmdPtr = 0;
            isCmdOk = 1;
        } else {
            cmdBuf[CMD_LEN-1] = 0x00;
            cmdPtr = 0;
        }
    } else {
        /* omit other chars */
    }
}

uint8_t EE24_Write(uint16_t addr, uint8_t *dat, uint8_t len)
{
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus*/
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);

    /* send a data byte */
    i2c_data_transmit(I2C0, addr&0xFF);
    /* wait until the transmission data register is empty*/
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));

    while(len>0)
    {
        /* send a data byte */
        i2c_data_transmit(I2C0, addr&0xFF);
        /* wait until the transmission data register is empty*/
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
        len--;
    }

    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0)&0x0200);

    return 0;
}

uint8_t EE24_Read(uint16_t addr, uint8_t *dat, uint8_t len)
{
    uint8_t i=0;

    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus*/
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);

    /* send a data byte */
    i2c_data_transmit(I2C0, addr&0xFF);
    /* wait until the transmission data register is empty*/
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus*/
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);

    for(i=0;i<len-1;i++)
    {
        /* wait until the RBNE bit is set */
        while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
        /* read a data from I2C_DATA */
        dat[i] = i2c_data_receive(I2C0);
      //I2C_Ack(EE24_I2CCH);		// MCU Ack
    }

    /* wait until the RBNE bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    /* read a data from I2C_DATA */
    dat[i] = i2c_data_receive(I2C0);
  //I2C_Nack(EE24_I2CCH);

    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0)&0x0200);

    return 0;
}

void PrintByteBuffer(const uint8_t *buf, uint32_t start, uint32_t len)
{
    uint16_t i = 0, j = 0;

    while(i<len) {
        printf("0x%08X ", start+i);
        for(j=0; j<16; j++)
            printf("%02X ", buf[i+j]);
        for(j=0; j<16; j++)
            printf("%c", (buf[i+j] >= 0x20 && buf[i+j]<=0x7E) ? buf[i+j] : '.');
        printf("\r\n");
        i += 16;
    }
}

void tskSyscall(void)
{
    uint8_t i = 0;
    if(isCmdOk){
        syscall_cmd = strtok(cmdBuf, " ");
        if(syscall_cmd){
            syscall_param1 = strtok(NULL, " ");
            if(syscall_param1)
                syscall_param2 = strtok(NULL, " ");
        }
        for(i=0;i<syscall_num;i++)
            if(strstr(syscall_cmd, syscall_table[i].name))
                break;
        if(i<syscall_num && syscall_table[i].func)
            (*syscall_table[i].func)();
        syscall_cmd = NULL;
        syscall_param1 = NULL;
        syscall_param2 = NULL;
        isCmdOk = 0;
    }
}

void syscall_test(void)
{
    if(syscall_cmd){
        printf("cmd:%s\r\n", syscall_cmd);
        if(syscall_param1){
            printf("param1:%s\r\n", syscall_param1);
            if(syscall_param2)
                printf("param2:%s\r\n", syscall_param2);
        }
    }
}

void syscall_help(void)
{
    uint8_t i = 0;
    for(i=0; i<syscall_num; i++)
        if(syscall_table[i].func)
            printf(" %s -> %s", syscall_table[i].name, syscall_table[i].desc);
}

void syscall_eerd(void)
{
    uint32_t startAddr = 0;
    uint16_t memlen = 0;

    if(syscall_param1) {
        startAddr = strtoul(syscall_param1, NULL, 16);
        if(syscall_param2)
            memlen = strtoul(syscall_param2, NULL, 16);
        memlen =  (memlen < 8 || memlen > 0x100) ? 0x100 : memlen;
        EE24_Read(startAddr, syscall_buf, memlen);
        PrintByteBuffer(syscall_buf, startAddr, memlen);
    }
}

void syscall_eewr(void)
{
    uint32_t startAddr = 0;
    uint16_t memlen = 0;

    if(syscall_param1 && syscall_param2) {
        startAddr = strtoul(syscall_param1, NULL, 16);
        syscall_buf[0] = strtoul(syscall_param2, NULL, 16);
        EE24_Write(startAddr, syscall_buf, 1);
        printf(" write 0x%02X to", syscall_buf[0]);
        printf(" flash address 0x%08X ...\r\n", startAddr);
    }
}

void syscall_version(void)
{
    printf(" Hello GD32, this is a simple command line demo.\r\n");
}

void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
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
}

void gpio_config(void)
{
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    nvic_irq_enable(USART0_IRQn, 0, 0);
    rcu_config();
    gpio_config();
    i2c_config();
    syscall_version();
    while(1){
        tskSyscall();
    }
}

void USART0_IRQHandler(void)
{
    volatile uint16_t i = 0;
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
        i = usart_data_receive(USART0);
        usart_recv_char(i);
        for(i=0; i<txPtr; i++){
            usart_data_transmit(USART0, txBuf[i]);
            while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
        }
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}

const struct syscall_item syscall_table[] = {
    {"eerd",    "eerd [addr] Read EEPROM.\r\n", syscall_eerd},
    {"eewr",    "write [addr] [data] Write EEPROM.\r\n", syscall_eewr},
    {"test",    "test command.\r\n", syscall_test},
    {"help",    "help Info.\r\n", syscall_help},
    {"version", "display version Info.\r\n", syscall_version},
};
const uint8_t syscall_num = sizeof(syscall_table)/sizeof(syscall_table[0]);
