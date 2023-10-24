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

#define SR_RXNE				(1U<<5)
#define SR_TXE				(1U<<7)

void uart2_tx_init();
void uart2_write(int ch);
int __io_putchar(int ch);

void uart2_rxtx_init();
char uart2_read();

void uart2_rx_interrupt_init();
void uart2_tx_interrupt_init();

void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len);

#define HISR_TCIF6		(1U<<21)
#define HIFCR_CTCIF6	(1U<<21)

#endif /* UART_H_ */
