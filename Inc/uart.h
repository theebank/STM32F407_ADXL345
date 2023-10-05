#include <stdio.h>
#include "stm32f4xx.h"
#include <stdint.h>
/*
 * uart.h
 *
 *  Created on: Oct 4, 2023
 *      Author: TKuma
 */

#ifndef UART_H_
#define UART_H_

void uart2_tx_init();
void uart2_write(int ch);
int __io_putchar(int ch);

void uart2_rxtx_init();
char uart2_read();


#endif /* UART_H_ */
