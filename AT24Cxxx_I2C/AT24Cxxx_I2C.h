/*
 * 									AT24Cxxx_I2C.h
 *
 * This file contains all the AT24Cxxx related APIs supported by the driver.
 *
 */

#ifndef INC_AT24CXXX_I2C_H_
#define INC_AT24CXXX_I2C_H_

/* MCU Specific Header file */
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* EEPROM I2C Address */
#define EEPROM_x_ADDRESS	(0x50 << 1)



/* EEPROM APIs*/

// Byte Read and Byte Write
void EEPROM_Writebyte(I2C_HandleTypeDef *pI2CHandle, uint16_t address, uint8_t data);
uint8_t EEPROM_Readbyte(I2C_HandleTypeDef *pI2CHandle, uint16_t address);

// Buffer Read and Buffer Write
void EEPROM_Write(I2C_HandleTypeDef *pI2CHandle, uint16_t address, uint8_t *pdata, uint16_t length);
void EEPROM_Read(I2C_HandleTypeDef *pI2CHandle, uint16_t address, uint8_t *pdata, uint16_t length);

#endif /* INC_AT24CXXX_I2C_H_ */
