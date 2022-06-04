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

#define LOBYTE(w)          (w&0xFF)
#define HIBYTE(w)          ((w>>8)&0xFF)

#define GD25_CS_HIGH()     gpio_bit_set(GPIOB, GPIO_PIN_12)
#define GD25_CS_LOW()      gpio_bit_reset(GPIOB, GPIO_PIN_12)

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

static uint32_t SPI_TimeoutCnt = 0;
uint16_t SPI_SwapByte(uint8_t byte)
{
    volatile uint16_t timeout = 0;
    uint16_t recv = 0;

    // wait for TBE = 1
    timeout = 0xFFFF;
    while(spi_i2s_flag_get(SPI1, SPI_FLAG_TBE) == RESET){
        timeout--;
        if(0 == timeout) {
            SPI_TimeoutCnt++;
            break;
        }
    }
    // Send byte through the SPI peripheral
    spi_i2s_data_transmit(SPI1, byte);
    // wait for RBNE = 1
    timeout = 0xFFFF;
    while(spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE) == RESET) {
        timeout--;
        if(0 == timeout) {
            SPI_TimeoutCnt++;
            break;
        }
    }
    // Return the byte read from the SPI bus
    recv = spi_i2s_data_receive(SPI1);
    // wait for TRANS = 0
    timeout = 0xFFFF;
    while (spi_i2s_flag_get(SPI1, SPI_FLAG_TRANS) == SET) {
        timeout--;
        if(0 == timeout) {
            SPI_TimeoutCnt++;
            break;
        }
    }
    return recv;
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

void GD25_WriteEnable(void)
{
    GD25_CS_LOW();
    SPI_SwapByte(0x06);
    GD25_CS_HIGH();
}

void GD25_WriteDisable(void)
{
    GD25_CS_LOW();
    SPI_SwapByte(0x04);
    GD25_CS_HIGH();
}

uint8_t GD25_ReadSR1(void)
{
    uint8_t byte = 0;

    GD25_CS_LOW();
    SPI_SwapByte(0x05);
    byte = SPI_SwapByte(0xFF);
    GD25_CS_HIGH();

    return byte;
}

void GD25_WaitBusy(void)
{
    while((GD25_ReadSR1() & 0x01) == 0x01);
}

void GD25_WritePage(uint8_t *buf, uint32_t addr, uint16_t len)
{
    uint16_t i = 0;
    uint8_t cmd[4];

    cmd[i] = 0x02;
    cmd[1] = LOBYTE(addr>>16);
    cmd[2] = LOBYTE(addr>>8);
    cmd[3] = LOBYTE(addr>>0);
    GD25_WriteEnable();
    GD25_WaitBusy();
    GD25_CS_LOW();
    for(i=0;i<4;i++)
        SPI_SwapByte(cmd[i]);
    for(i=0;i<len;i++)
        SPI_SwapByte(buf[i]);
    GD25_CS_HIGH();
    GD25_WriteDisable();
    GD25_WaitBusy();
}

void GD25_Erase4KB(uint32_t addr)
{
    uint16_t i = 0;
    uint8_t cmd[4];

    cmd[i] = 0x20;
    cmd[1] = LOBYTE(addr>>16);
    cmd[2] = LOBYTE(addr>>8);
    cmd[3] = LOBYTE(addr>>0);
    GD25_WriteEnable();
    GD25_WaitBusy();
    GD25_CS_LOW();
    for(i=0;i<4;i++)
        SPI_SwapByte(cmd[i]);
    GD25_CS_HIGH();
    GD25_WriteDisable();
    GD25_WaitBusy();
}

void GD25_Read(uint8_t *buf, uint32_t addr, uint16_t len)
{
    uint16_t i = 0;
    uint8_t cmd[4];

    cmd[i] = 0x03;
    cmd[1] = LOBYTE(addr>>16);
    cmd[2] = LOBYTE(addr>>8);
    cmd[3] = LOBYTE(addr>>0);
    GD25_CS_LOW();
    for(i=0;i<4;i++)
        SPI_SwapByte(cmd[i]);
    for(i=0;i<len;i++)
        buf[i] = SPI_SwapByte(0xFF);
    GD25_CS_HIGH();
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

void syscall_read(void)
{
    uint32_t startAddr = 0;
    uint16_t memlen = 0;

    if(syscall_param1) {
        startAddr = strtoul(syscall_param1, NULL, 16);
        if(syscall_param2)
            memlen = strtoul(syscall_param2, NULL, 16);
        memlen =  (memlen < 8 || memlen > 0x100) ? 0x100 : memlen;
        GD25_Read(syscall_buf, startAddr, memlen);
        PrintByteBuffer(syscall_buf, startAddr, memlen);
    }
}

void syscall_write(void)
{
    uint32_t startAddr = 0;
    uint16_t memlen = 0;

    if(syscall_param1 && syscall_param2) {
        startAddr = strtoul(syscall_param1, NULL, 16);
        syscall_buf[0] = strtoul(syscall_param2, NULL, 16);
        GD25_WritePage(syscall_buf, startAddr, 1);
        printf(" write 0x%02X to", syscall_buf[0]);
        printf(" flash address 0x%08X ...\r\n", startAddr);
    }
}

void syscall_erase(void)
{
    uint32_t startAddr = 0;
    if(syscall_param1) {
        startAddr = strtoul(syscall_param1, NULL, 16);
        GD25_Erase4KB(startAddr);
        printf(" erase 4kB sector 0x%08X\r\n", startAddr);
    }
}

void syscall_version(void)
{
    printf(" Hello GD32, this is a simple command line demo.\r\n");
}

void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_AF);
}

void spi_config(void)
{
    spi_parameter_struct spi_init_struct;

    /* SPI1 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;   // 120M/8=15M
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1);
}

void gpio_config(void)
{
    /* SPI1 GPIO config:NSS/PB12, SCK/PB13, MISO/PB14, MOSI/PB15 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_15);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    /* PB12 as NSS */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    GD25_CS_HIGH();
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    nvic_irq_enable(USART0_IRQn, 0, 0);
    rcu_config();
    gpio_config();
    spi_config();
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
    {"read",    "read [addr] Read SPI Flash.\r\n", syscall_read},
    {"write",   "write [addr] [data] Write SPI Flash.\r\n", syscall_write},
    {"erase",   "erase [addr] Erase SPI Flash.\r\n", syscall_erase},
    {"test",    "test command.\r\n", syscall_test},
    {"help",    "help Info.\r\n", syscall_help},
    {"version", "display version Info.\r\n", syscall_version},
};
const uint8_t syscall_num = sizeof(syscall_table)/sizeof(syscall_table[0]);
