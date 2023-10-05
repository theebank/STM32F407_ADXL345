#include "uart.h"

#define GPIOAEN				(0x1UL<< 0)
#define GPIODEN				(0x1UL<< 3)

#define GPIODEN				(0x1UL<< 3)

#define PIN0				(1U<<0)
#define USER_BTN_PIN		PIN0

#define PIN12				(1U<<12)
#define LED_GREEN			PIN12

#define PIN13				(1U<<13)
#define LED_ORANGE			PIN13

#define PIN14				(1U<<14)
#define LED_RED				PIN14

#define PIN15				(1U<<15)
#define LED_BLUE			PIN15

void LightsOn(){
	GPIOD->BSRR |= LED_GREEN;
	GPIOD->BSRR |= LED_ORANGE;
	GPIOD->BSRR |= LED_RED;
	GPIOD->BSRR |= LED_BLUE;
}
void LightsOff(){
	GPIOD->BSRR |= (1U<<28);
	GPIOD->BSRR |= (1U<<29);
	GPIOD->BSRR |= (1U<<30);
	GPIOD->BSRR |= (1U<<31);
}

char key;

int main(void){

	RCC->AHB1ENR |= GPIOAEN;
	RCC->AHB1ENR |= GPIODEN;


	GPIOD->MODER |= (0x1UL<<24U);
	GPIOD->MODER &=~(1U<<25);

	GPIOD->MODER |= (0x1UL<<26U);
	GPIOD->MODER &=~(1U<<27);

	GPIOD->MODER |= (0x1UL<<28U);
	GPIOD->MODER &=~(1U<<29);

	GPIOD->MODER |= (0x1UL<<30U);
	GPIOD->MODER &=~(1U<<31);

	uart2_tx_init();

	while(1){
		key = uart2_read();
		if(key=='1'){
			GPIOD->ODR &=~LED_GREEN;
		}else{
			GPIOD->ODR |= LED_GREEN;
		}

	}
}
