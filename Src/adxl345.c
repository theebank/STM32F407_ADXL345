/*
 * adxl345.c
 *
 *  Created on: Oct 22, 2023
 *      Author: TKuma
 */
#include "adxl345.h"

char data;
uint8_t data_rec[6];

void adxl_read_address(uint8_t reg){
	I2C1_byteRead(DEVICE_ADDR, reg, &data);
}

void adxl_write(uint8_t reg, char value){
	char data[1];
	data[0] =value;
	I2C1_burstWrite( DEVICE_ADDR, reg, 1, data);
}
void adxl_read_values(uint8_t reg){
	I2C1_burstRead(DEVICE_ADDR, reg, 6, (char *)data_rec);
}
void adxl_init(void){
//	enable i2c module
	I2C1_init();
//	read the devid, should return 0xe5
	adxl_read_address(DEVID_R);
//	set data format range to +/- 4g
	adxl_write(DATA_FORMAT_R, FOUR_G);
//	reset all bits
	adxl_write(POWER_CTL_R, RESET);
//	set power control measure bit
	adxl_write(POWER_CTL_R, SET_MEASURE_B);

}
void adxl_init_spi(void){
	/*Enable SPI gpio*/
	spi_gpio_init();

	/*Config SPI*/
	spi1_config();

	/*Set data format range to +-4g*/
	adxl_write_spi (DATA_FORMAT_R, FOUR_G);

	/*Reset all bits*/
	adxl_write_spi (POWER_CTL_R, RESET);

	/*Configure power control measure bit*/
	adxl_write_spi (POWER_CTL_R, SET_MEASURE_B);
}

void adxl_write_spi(uint8_t address, uint8_t value){
	  uint8_t data[2];

	  /*Enable multi-byte, place address into buffer*/
	  data[0] = address|MULTI_BYTE_EN;

	  /*Place data into buffer*/
	  data[1] = value;

	  /*Pull cs line low to enable slave*/
	  cs_enable();

	  /*Transmit data and address*/
	  spi1_transmit(data, 2);

	  /*Pull cs line high to disable slave*/
	  cs_disable();
}
void adxl_read_spi(uint8_t address, uint8_t *rxdata){
	  /*Set read operation*/
	  address |= READ_OPERATION;

	  /*Enable multi-byte*/
	  address |= MULTI_BYTE_EN;

	  /*Pull cs line low to enable slave*/
	  cs_enable();

    /*Send address*/
	  spi1_transmit(&address,1);

	  /*Read 6 bytes */
	  spi1_receive(rxdata,6);

	  /*Pull cs line high to disable slave*/
	  cs_disable();
}
