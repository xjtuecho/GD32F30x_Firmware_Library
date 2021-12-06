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

#define TX_LEN  128
#define RX_LEN  128
#define CMD_LEN  128

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

void syscall_version(void)
{
    printf(" Hello GD32, this is a simple command line demo.\r\n");
}

int main(void)
{
    gd_eval_com_init(EVAL_COM1);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    nvic_irq_enable(USART0_IRQn, 0, 0);
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
	{"test",    "test command.\r\n", syscall_test},
	{"help",    "help Info.\r\n", syscall_help},
	{"version", "display version Info.\r\n", syscall_version},
};
const uint8_t syscall_num = sizeof(syscall_table)/sizeof(syscall_table[0]);
