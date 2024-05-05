/*
 * 									AT24Cxxx_I2C.c
 *
 *  This file contains AT24Cxxx driver API implementations.
 *
 */

#include "AT24Cxxx_I2C.h"

/* ------------------------------------------------------------------------------------------------------
 * Name		:	EEPROM_Writebyte
 * Description	:	Writes 1 byte of data to a given memory address
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Memory address (where to write)
 * Parameter 3	:	Data to write
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void EEPROM_Writebyte(I2C_HandleTypeDef *pI2CHandle, uint16_t address, uint8_t data)
{
	;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	EEPROM_Readbyte
 * Description	:	Reads 1 byte of data from a given memory address.
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Memory address (from where to read)
 * Return Type	:	uint8_t
 * Note		:	Returns data, read from provided memory address
 * ------------------------------------------------------------------------------------------------------ */
uint8_t EEPROM_Readbyte(I2C_HandleTypeDef *pI2CHandle, uint16_t address)
{
	return 0;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	EEPROM_Write
 * Description	:	Writes data to a given memory address.
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Memory address (where to write)
 * Parameter 3	:	Pointer to data buffer
 * Parameter 4	: 	Length of data (bytes to write)
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void EEPROM_Write(I2C_HandleTypeDef *pI2CHandle, uint16_t address, uint8_t *pdata, uint16_t length)
{
	;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	EEPROM_Read
 * Description	:	Reads data from a given memory address.
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Memory address (where to read)
 * Parameter 3	:	Pointer to data buffer (to store read data)
 * Parameter 4	: 	Length of data (bytes to read)
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void EEPROM_Read(I2C_HandleTypeDef *pI2CHandle, uint16_t address, uint8_t *pdata, uint16_t length)
{
	;
}
