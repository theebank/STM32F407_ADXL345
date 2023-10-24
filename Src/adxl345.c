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
