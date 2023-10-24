# STM32F407 device driver used to interface ADXL345 Accelerometer

## Behaviour
Gather reading from the ADXL345 Accelerometer using the SPI or I2C protocols

## Design
Designed using the I2C and SPI protocols

## Hardware Required
- STM32F407 Discovery Board
- ADXL345 Accelerometer
- Female-Female Jumper Wires

## Hardware Setup
#### Via I2C
Connect ADXL345 Accelerometer to the STM32F407 Discovery board using the following connections:
- GND to GND
- VCC to 5V
- SDA to PB9
- SCL to PB8

#### Via SPI
Connect ADXL345 Accelerometer to the STM32F407 Discovery board using the following connections:
- GND to GND
- VCC to 5V
- CS to PA9(SS)
- SDO to PA6(MISO)
- SDA to PA7(MOSI)
- SCL to PA5(CLK) 
