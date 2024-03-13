/*
 * 									LIS3MDL_I2C.c
 *
 *  This file contains LIS3MDL driver API implementations for I2C communication.
 *
 */

#include "LIS3MDL_I2C.h"


/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_DefaultInit
 * Description	:	Initialize LISM3MDL in pre-defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	In this initialization:
 * 	- Temperature sensor is enabled
 * 	- X, Y, and Z axis Operating mode is set for Ultra-high performance
 * 	- Output data rate is configured for 80 Hz
 * 	- FAST_ODR is disabled
 * 	- Self-test is disabled
 * 	- Full-scale selection is configured for ±4 Gauss
 * 	- System operating mode is configured for continuous-conversion
 * 	- Data LSB is at lower address
 * 	- Fast read is disabled
 * ------------------------------------------------------------------------------------------------------ */
void LIS3MDL_DefaultInit(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data = 0;
	/*- Register 1: default value 0b00010000 -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Enable temperature sensor, setting bit[7]  -*/
	data |= (1<<7);
	/*- Ultra-high performance operation mode for X and Y axis, setting bits[6:5]  -*/
	data |= (3<<5);
	/*- Configure output data rate, setting bits[4:2]  -*/
	data |= (7<<2);
	/*- Disable fast output data rate and self test, clearing bits[1:0]  -*/
	data &= ~(0x3);
	/*- Configuring Register 1 with above settings -*/
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Register 2: default value 0b00000000 -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_2, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Clearing entire register  -*/
	data &= (uint8_t)~(0xFF);
	/*- Bits[7,4,1,0] must be 0 -*/
	/*- Full-scale is configured for ±4 Gauss, clearing bits[6:5] -*/
	/*- Reboot memory: disabled, clearing bit[3] -*/
	/*- Soft Reset: disabled, clearing bit[2] -*/
	/*- Configuring Register 2 with above settings -*/
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_2, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Register 3: default value 0b00000011 -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_3, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Clearing entire register  -*/
	data &= (uint8_t)~(0xFF);
	/*- Bits[7,6,4,3] must be 0 -*/
	/*- Configure for low-power mode, clearing bit[5] -*/
	/*- SIM set to default, clearing bit[2] -*/
	/*- Configure system operating mode as continuous conversion, clearing bit[1:0] -*/
	/*- Configuring Register 3 with above settings -*/
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_3, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Register 4: default value 0b00000000 -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_4, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Clearing entire register  -*/
	data &= (uint8_t)~(0xFF);
	/*- Bits[7,6,5,4,0] must be 0 -*/
	/*- Ultra-high performance operation mode for X axis, setting bits[3:2]  -*/
	data |= (3<<2);
	/*- Configure data LSB at lower address, clearing bit[1]  -*/
	/*- Configuring Register 4 with above settings -*/
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_4, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Register 5: default value 0b00000000 -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_5, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Clearing entire register  -*/
	data &= (uint8_t)~(0xFF);
	/*- Bits[5:0] must be 0 -*/
	/*- Disable fast read, clearing bit[7] -*/
	/*- Disable: block data updated for magnetic data, clearing bit[6] -*/
	/*- Configuring Register 5 with above settings -*/
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_5, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

}
/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_UserInit
 * Description	:	Initialize LISM3MDL with user defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:
 * Parameter 3	:
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void LIS3MDL_UserInit(I2C_HandleTypeDef *pI2CHandle)
{

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_DeInit
 * Description	:	De-initialize LISM3MDL with user defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	Reset all the Control Register to their default state
 * ------------------------------------------------------------------------------------------------------ */
void LIS3MDL_DeInit(I2C_HandleTypeDef *pI2CHandle)
{
	/*- Perform Soft reset: register 2, bit[2] -*/
	uint8_t data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_2, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (1<<2);
	HAL_I2C_Mem_Write(&hi2c1,(0x1C << 1), 0x21, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getMAG_X
 * Description	:	Returns magnetic field data in X axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t LIS3MDL_getMAG_X(I2C_HandleTypeDef *pI2CHandle)
{
	int8_t Out_L = 0;
	int8_t Out_H = 0;
	/*- Read register OUT_X_L -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_X_L, I2C_MEMADD_SIZE_8BIT, &Out_L, 1, HAL_MAX_DELAY);
	/*- Read register OUT_X_H -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_X_H, I2C_MEMADD_SIZE_8BIT, &Out_H, 1, HAL_MAX_DELAY);

	return ((int16_t)(Out_H << 8) | Out_L);
}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getMAG_Y
 * Description	:	Returns magnetic field data in Y axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t LIS3MDL_getMAG_Y(I2C_HandleTypeDef *pI2CHandle)
{
	int8_t Out_L = 0;
	int8_t Out_H = 0;
	/*- Read register OUT_Y_L -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_Y_L, I2C_MEMADD_SIZE_8BIT, &Out_L, 1, HAL_MAX_DELAY);
	/*- Read register OUT_Y_H -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_Y_H, I2C_MEMADD_SIZE_8BIT, &Out_H, 1, HAL_MAX_DELAY);

	return ((int16_t)(Out_H << 8) | Out_L);
}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getMAG_Z
 * Description	:	Returns magnetic field data in Z axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t LIS3MDL_getMAG_Z(I2C_HandleTypeDef *pI2CHandle)
{
	int8_t Out_L = 0;
	int8_t Out_H = 0;
	/*- Read register OUT_Z_L -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_Z_L, I2C_MEMADD_SIZE_8BIT, &Out_L, 1, HAL_MAX_DELAY);
	/*- Read register OUT_Z_H -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_Z_H, I2C_MEMADD_SIZE_8BIT, &Out_H, 1, HAL_MAX_DELAY);

	return ((int16_t)(Out_H << 8) | Out_L);
}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getTemperature_C
 * Description	:	Returns temperature sensor's output
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t LIS3MDL_getTemperature_C(I2C_HandleTypeDef *pI2CHandle)
{
	int8_t Out_L = 0;
	int8_t Out_H = 0;
	int16_t Temp_c = 0;
	/*- Read register OUT_Y_L -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, TEMP_OUT_L, I2C_MEMADD_SIZE_8BIT, &Out_L, 1, HAL_MAX_DELAY);
	/*- Read register OUT_Y_H -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, &Out_H, 1, HAL_MAX_DELAY);

	// configure offset and scaling
	Temp_c = (((int16_t)(Out_H << 8) | Out_L) / 256) + 25;
	return Temp_c;

}
