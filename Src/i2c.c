/*
 * i2c.c
 *
 *  Created on: Oct 18, 2023
 *      Author: TKuma
 */
#include "stm32f4xx.h"
#include "i2c.h"


#define GPIOBEN			(1U<<1)
#define I2C1EN			(1U<<21)

#define I2C_100KHz				80 //0b 0101 0000 = 80
#define SD_MODE_MAX_RISE_TIME	17

#define CR1_PE				(1U<<0)
#define SR2_BUSY			(1U<<1)
#define CR1_START			(1U<<8)
#define CR1_STOP			(1U<<9)
#define SR1_SB				(1U<<0)
#define SR1_ADDR			(1U<<1)
#define SR1_TxE				(1U<<7)
#define SR1_RxNE			(1U<<6)
#define CR1_ACK				(1U<<10)
#define SR1_BTF				(1U<<2)

//pinout
//pb8 --- SCL
//pb9 ---SDA

void I2C1_init(void){
//	enable clock access to GPIOB
	RCC->AHB1ENR |= GPIOBEN;
//	set pb8 & pb9 mode to alt func
	GPIOB->MODER &=~ (1U<<16);
	GPIOB->MODER |= (1U<<17);

	GPIOB->MODER &=~ (1U<<18);
	GPIOB->MODER |= (1U<<19);


//	set pb8 and pb9 output type to open drain
	GPIOB->OTYPER |= (1U<<8);
	GPIOB->OTYPER |= (1U<<9);
//	enable pullup for pb8 and pb9
	GPIOB->PUPDR |= (1U<<16);
	GPIOB->PUPDR &=~ (1U<<17);

	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &=~ (1U<<19);

//	set alt func mode for pb8 and pb9
	GPIOB->AFR[1] &=~ (1U<<0);
	GPIOB->AFR[1] &=~ (1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &=~ (1U<<3);

	GPIOB->AFR[1] &=~ (1U<<4);
	GPIOB->AFR[1] &=~ (1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &=~ (1U<<7);

//	enable clock access to i2c1
	RCC->APB1ENR |= I2C1EN;

//	enter rest mode
	I2C1->CR1 |= (1U<<15);
//  come out of reset mode
	I2C1->CR1 &=~ (1U<<15);

//	set periph clock freq
	I2C1->CR2 |= (1U<<4);//16 MHz

//	set i2c to standard mode, 100khz clock
	I2C1->CCR = I2C_100KHz;

//	set rise time
	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

//	enable i2c1 module
	I2C1->CR1 |= CR1_PE;
}
void I2C1_byteRead(char saddr, char maddr, char* data){

	volatile int tmp;
//	wait until bus not busy
	while(I2C1->SR2 & SR2_BUSY){}

//	generate start condition
//	set start bit to 1
	I2C1->CR1 |= CR1_START;

//	wait until start flag
	while(!(I2C1->SR1 & SR1_SB)){}

//	transmit slave address + write
	I2C1->DR = saddr <<1;

//	wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}

//	clear addr flag
	tmp = I2C1->SR2;

//	send mem addr
	I2C1->DR = maddr;

//	wait until transmitter empty
	while(!(I2C1->SR1 & SR1_TxE)){}

//	generate restart
	I2C1->CR1 |= CR1_START;

//	wait until start flag
	while(!(I2C1->SR1 & SR1_SB)){}

//	transmite slave addr + read
	I2C1->DR = saddr << 1 | 1;

//	wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}

//	disable ack
	I2C1->CR1 &=~ CR1_ACK;

//	clear addr flag
	tmp = I2C1->SR2;

//	generate stop after data received
	I2C1->CR1 |= CR1_STOP;

//	wait until rxne flag is set
	while(!(I2C1->SR1 & SR1_RxNE)){}

//	read data from dr
	*data++ = I2C1->DR;
}

void I2C1_burstRead(char saddr, char maddr, int n, char* data){
	volatile int tmp;

//	wait until bus not busy
	while(I2C1->SR2 & SR2_BUSY){}

//	generate start condition
//	set start bit to 1
	I2C1->CR1 |= CR1_START;

//	wait until start flag
	while(!(I2C1->SR1 & SR1_SB)){}

//	transmit slave addr + write
	I2C1->DR = saddr <<1;

//	wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}

//	clear addr flag
	tmp = I2C1->SR2;

//	wait until transmitter empty
	while(!(I2C1->SR1 & SR1_TxE)){}

//	send memory address
	I2C1->DR = maddr;

//	wait until transmitter empty
	while(!(I2C1->SR1 & SR1_TxE)){}

//	generate restart
	I2C1->CR1 |= CR1_START;

//	wait until start flag
	while(!(I2C1->SR1 & SR1_SB)){}

//	transmite slave addr + read
	I2C1->DR = saddr << 1 | 1;

//	wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}


//	clear addr flag
	tmp = I2C1->SR2;

//	enable ack
	I2C1->CR1 |= CR1_ACK;

	while(n > 0U){
//		if one byte
		if(n==1U){
			//	disable ack
			I2C1->CR1 &=~ CR1_ACK;

			//	generate stop condition
			I2C1->CR1 |= CR1_STOP;

			//	wait until rxne flag is set
			while(!(I2C1->SR1 & SR1_RxNE)){}

			//	read data from dr
			*data++ = I2C1->DR;

			break;
		}else{
			//	wait until rxne flag is set
			while(!(I2C1->SR1 & SR1_RxNE)){}


			//	read data from dr
			(*data++) = I2C1->DR;

			n--;
		}
	}

}
void I2C1_burstWrite( char saddr, char maddr, int n, char* data){
	volatile int tmp;

	//	wait until bus not busy
	while(I2C1->SR2 & SR2_BUSY){}

	//	generate start condition
	//	set start bit to 1
	I2C1->CR1 |= CR1_START;

	//	wait until start flag
	while(!(I2C1->SR1 & SR1_SB)){}

	//	transmit slave addr + write
	I2C1->DR = saddr <<1;

	//	wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}

	//	clear addr flag
	tmp = I2C1->SR2;

	//	wait until transmitter empty
	while(!(I2C1->SR1 & SR1_TxE)){}

	//	send memory address
	I2C1->DR = maddr;

	for (int i =0;i<n;i++){
		//	wait until transmitter empty
		while(!(I2C1->SR1 & SR1_TxE)){}

		// transmit mem address
		I2C1->DR = *data++;

	}
	// wait until transfer finished
	while(!(I2C1->SR1 & SR1_BTF)){}


	//	generate stop after data received
	I2C1->CR1 |= CR1_STOP;


}

