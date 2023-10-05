#include "uart.h"
/*
 * uart.c
 *
 *  Created on: Oct 4, 2023
 *      Author: TKuma
 */

#define GPIOAEN				(0x1UL<< 0)
#define USART2EN			(0x1UL<<17)

#define sysfreq				16000000
#define APB1_CLK			sysfreq

#define UART_BR				115200

#define CR1_TE				(1U<<3)
#define CR1_RE				(1U<<2)
#define CR1_EN				(1U<<13)
#define SR_TXE				(1U<<7)
#define SR_RXNE				(1U<<5)

static uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
void uart2_tx_init();
void uart2_write(int ch);
int __io_putchar(int ch);

static uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate){
	return (PeriphClk + (BaudRate/2U))/BaudRate;
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate){
	USARTx->BRR = compute_uart_div(PeriphClk,BaudRate);
}

void uart2_rxtx_init(){
	//configure uart gpio pin
//	enable clock access to gpioa
	RCC->AHB1ENR |= GPIOAEN;
//	set pa2 mode to alternate func mode
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5);
//	set pa2 alternate func type to uart tx af07
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

//	set pa3 mode to alternate func mode
	GPIOA->MODER &=~(1U<<6);
	GPIOA->MODER |= (1U<<7);

//	set pa3 alternate func type to uart tx af07
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &=~ (1U<<15);


//	confugre uart module
//	enable clcock access to uart2
	RCC->APB1ENR |= USART2EN;
//	configure uart baudrate
	uart_set_baudrate(USART2,APB1_CLK, UART_BR);
//	configure transfer direction
	USART2->CR1 |= CR1_TE;
	USART2->CR1 |= CR1_RE;

	//enable uart module
	USART2->CR1 |= CR1_EN;
}

char uart2_read(){
	//make sure receive data reg is not empty
	while(!(USART2->SR & SR_RXNE)){}
//	return data
	return USART2->DR;
}

void uart2_tx_init(){
	//configure uart gpio pin
//	enable clock access to gpioa
	RCC->AHB1ENR |= GPIOAEN;
//	set pa2 mode to alternate func mode
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5);
//	set pa2 alternate func type to uart tx af07
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

//	confugre uart module
//	enable clcock access to uart2
	RCC->APB1ENR |= USART2EN;
//	configure uart baudrate
	uart_set_baudrate(USART2,APB1_CLK, UART_BR);
//	configure transfer direction
	USART2->CR1 |= CR1_TE;

	//enable uart module
	USART2->CR1 |= CR1_EN;
}

void uart2_write(int ch){
	//make sure transmit data reg is empty
	while(!(USART2->SR & SR_TXE)){}
//	write to transmit data reg


	USART2->DR = (ch & 0xFF);

}


int __io_putchar(int ch){
	uart2_write(ch);
	return ch;
}

