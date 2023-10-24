#include "uart.h"
/*
 * uart.c
 *
 *  Created on: Oct 4, 2023
 *      Author: TKuma
 */

#define GPIOAEN				(0x1UL<< 0)
#define USART2EN			(0x1UL<<17)
	#define UART2EN			(1U<<17)

#define sysfreq				16000000
#define APB1_CLK			sysfreq

#define UART_BR				115200

#define SYS_FREQ		16000000

#define UART_BAUDRATE		115200


#define CR1_UE			(1U<<13)
#define CR1_TE				(1U<<3)
#define CR1_RE				(1U<<2)
#define CR1_EN				(1U<<13)
#define SR_TXE				(1U<<7)
#define SR_RXNE				(1U<<5)

#define CR1_RXNEIE			(1U<<5)
#define CR1_TXEIE			(1U<<7)

#define DMA1EN				(1U<<21)
#define DMA_CR_EN				     (1U<<0)

#define CHSEL4				(1U<<27)

#define DMA_CR_MINC				(1U<<10)
#define DMA_DIR_MEM_TO_PERIPH		(1U<<6)
#define DMA_CR_TCIE				(1U<<4)
#define UART_CR3_DMAT				 (1U<<7)
#define DMA_MEM_INC					(1U<<10)

static uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
void uart2_rx_interrupt_init();
void uart2_tx_interrupt_init();
void uart2_tx_init();
void uart2_write(int ch);
int __io_putchar(int ch);
void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len);

static uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate){
	return (PeriphClk + (BaudRate/2U))/BaudRate;
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate){
	USARTx->BRR = compute_uart_div(PeriphClk,BaudRate);
}

void uart2_rx_interrupt_init(){
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
//	enable receiver for rxne interrupt
	USART2->CR1 |= CR1_RXNEIE;
// 	enable uart2 interrupt in nvic
	NVIC_EnableIRQ(USART2_IRQn);
	//enable uart module
	USART2->CR1 |= CR1_EN;
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

void uart2_tx_init(void)
{
	/****************Configure uart gpio pin***************/
	/*Enable clock access to gpioa */
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA2 mode to alternate function mode*/
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	/*Set PA2 alternate function type to UART_TX (AF07)*/
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);


	/****************Configure uart module ***************/
	/*Enable clock access to uart2 */
	RCC->APB1ENR |= UART2EN;

	/*Configure baudrate*/
	uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);

	/*Configure the transfer direction*/
	 USART2->CR1 =  CR1_TE;

	/*Enable uart module*/
	 USART2->CR1 |= CR1_UE;


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

void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len)
{
	/*Enable clock access to DMA*/
	RCC->AHB1ENR |=DMA1EN;

	/*Disable DMA1 Stream6*/
	DMA1_Stream6->CR &=~DMA_CR_EN;

    /*Wait until DMA1 Stream 6 is disabled */
    while(DMA1_Stream6->CR & DMA_CR_EN){}

	/*Clear all interrupt flags of Stream6*/

	DMA1->HIFCR |=(1U<<16);
	DMA1->HIFCR |=(1U<<18);
	DMA1->HIFCR |=(1U<<19);
	DMA1->HIFCR |=(1U<<20);
	DMA1->HIFCR |=(1U<<21);


	/*Set the destination buffer*/
	DMA1_Stream6->PAR = dst;

	/*Set the source buffer*/
	DMA1_Stream6->M0AR = src;

	/*Set length*/
	DMA1_Stream6->NDTR = len;

	/*Select Stream6 CH4*/
	DMA1_Stream6->CR = CHSEL4;

	/*Enable memory increment*/
	DMA1_Stream6->CR |= DMA_MEM_INC;

	/*Configure transfer direction*/
	DMA1_Stream6->CR |= DMA_DIR_MEM_TO_PERIPH;

	/*Enable DMA transfer complete interrupt*/
	DMA1_Stream6->CR |=DMA_CR_TCIE;

	/*Enable direct mode and disable FIFO*/
	DMA1_Stream6->FCR = 0;

	/*Enable DMA1 Stream6*/
	DMA1_Stream6->CR |=DMA_CR_EN;

	/*Enable UART2 transmitter DMA*/
	USART2->CR3 |=UART_CR3_DMAT;

	/*DMA Interrupt enable in NVIC*/
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);


}


