/*
 * 									LIS3MDL_I2C.c
 *
 *  This file contains LIS3MDL driver API implementations for I2C communication.
 *
 */

#include "LIS3MDL_I2C.h"



/*- Private Variables -*/
static float magSensitivity = 6842.0f; /* Default Sensitivity: 6842 LSB/Gauss | for ±4 Gauss */



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
	/*- Reset LIS3MDL -*/
	LIS3MDL_DeInit(pI2CHandle);

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
	/*- Ultra-high performance operation mode for Z axis, setting bits[3:2]  -*/
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

	/*- Default Sensitivity -*/
	magSensitivity = 6842.0f;

}
/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_UserInit
 * Description	:	Initialize LISM3MDL with user defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Temperature Sensor Enable or Disable
 * Parameter 3	:	Output Data Rate
 * Parameter 4	:	X, Y and Z axis Operating Mode
 * Parameter 5	:	Full-Scale Selection
 * Parameter 6	:	Measurement Modes
 * Return Type	:	none (void)
 * Note		: Possible arguments: MAG_TEMP_x, MAG_ODR_x, MAG_OM_x, MAG_FS_x, MAG_M_x
 * ------------------------------------------------------------------------------------------------------ */
void LIS3MDL_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t temp_Enable, int8_t ODR, int8_t xyzOM, int8_t FScale, int8_t measurementMode)
{
	/*- Reset LIS3MDL -*/
	LIS3MDL_DeInit(pI2CHandle);

	/*- temporary variables to store registers contents  -*/
	uint8_t reg1Config = 0;
	uint8_t reg2Config = 0;
	uint8_t reg3Config = 0;
	uint8_t reg4Config = 0;
	uint8_t reg5Config = 0;

	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_1, I2C_MEMADD_SIZE_8BIT, &reg1Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_2, I2C_MEMADD_SIZE_8BIT, &reg2Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_3, I2C_MEMADD_SIZE_8BIT, &reg3Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_4, I2C_MEMADD_SIZE_8BIT, &reg4Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, CTRL_REG_5, I2C_MEMADD_SIZE_8BIT, &reg5Config, 1, HAL_MAX_DELAY);

	reg1Config &= (uint8_t)~(0xFF);
	reg2Config &= (uint8_t)~(0xFF);
	reg3Config &= (uint8_t)~(0xFF);
	reg4Config &= (uint8_t)~(0xFF);
	reg5Config &= (uint8_t)~(0xFF);

	/*- Temperature sensor Configuration  -*/
	if (temp_Enable)
	{
		/*- Enable temperature sensor, setting bit[7]  -*/
		reg1Config |= (1<<7);
	}

	/*- ODR Configuration  -*/
	if (ODR == MAG_ODR_0_625HZ)
	{
		reg1Config &= ~(7<<2);
	}
	else if (ODR == MAG_ODR_1_25HZ)
	{
		reg1Config &= ~(7<<2);
		reg1Config |= (1<<2);
	}
	else if (ODR == MAG_ODR_2_5HZ)
	{
		reg1Config &= ~(7<<2);
	    reg1Config |= (1<<3);
	}
	else if (ODR == MAG_ODR_5HZ)
	{
		reg1Config &= ~(7<<2);
	    reg1Config |= (3<<2);
	}
	else if (ODR == MAG_ODR_10HZ)
	{
		reg1Config &= ~(7<<2);
	    reg1Config |= (1<<4);
	}
	else if (ODR == MAG_ODR_20HZ)
	{
		reg1Config &= ~(7<<2);
	    reg1Config |= (5<<2);
	}
	else if (ODR == MAG_ODR_40HZ)
	{
		reg1Config &= ~(7<<2);
	    reg1Config |= (3<<3);
	}
	else
	{
		/*- ODR 80Hz  -*/
		reg1Config |= (7<<2);
	}

	/*- X,Y and Z Axis Operational Mode Configuration  -*/
	if (xyzOM == MAG_OM_LOW_POWER)
	{
		/*- X and Y Operational Mode -*/
		reg1Config &= ~(3<<5);
		/*- Z Operational Mode -*/
		reg4Config &= ~(3<<2);
	}
	else if (xyzOM == MAG_OM_MEDIUM_POWER)
	{
		/*- X and Y Operational Mode -*/
		reg1Config &= ~(3<<5);
		reg1Config |= (1<<5);
		/*- Z Operational Mode -*/
		reg4Config &= ~(3<<2);
		reg4Config |= (1<<2);

	}
	else if (xyzOM == MAG_OM_HIGH_POWER)
	{
		/*- X and Y Operational Mode -*/
		reg1Config &= ~(3<<5);
		reg1Config |= (1<<6);
		/*- Z Operational Mode -*/
		reg4Config &= ~(3<<2);
		reg4Config |= (1<<3);

	}
	else
	{
		/*- Operational Mode is Ultra-high Performance Mode  -*/
		/*- X and Y Operational Mode -*/
		reg1Config |= (3<<5);
		/*- Z Operational Mode -*/
		reg4Config |= (3<<2);
	}

	/*- Full-Scale Configuration  -*/
	if (FScale == MAG_FS_4G)
	{
		reg2Config &= ~(3<<5);

		/*- Sensitivity -*/
		magSensitivity = 6842.0f;
	}
	else if (FScale == MAG_FS_8G)
	{
		reg2Config &= ~(3<<5);
		reg2Config |= (1<<5);

		/*- Sensitivity -*/
		magSensitivity = 3421.0f;
	}
	else if (FScale == MAG_FS_12G)
	{
		reg2Config &= ~(3<<5);
		reg2Config |= (1<<6);

		/*- Sensitivity -*/
		magSensitivity = 2281.0f;
	}
	else
	{
		/*- FS: ±16 Gauss -*/
		reg2Config |= (3<<5);

		/*- Sensitivity -*/
		magSensitivity = 1711.0f;
	}

	/*- Measurement Mode Configuration  -*/
	if (measurementMode == MAG_M_CONTINUOUS_CONV)
	{
		reg3Config &= ~(3<<0);
	}
	else if (measurementMode == MAG_M_SINGLE_CONV)
	{
		reg3Config &= ~(3<<0);
		reg3Config |= (1<<0);
	}
	else
	{
		/*- Power Down Mode -*/
		reg3Config |= (3<<0);
	}

	/*- Configure registers with user settings  -*/

	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_1, I2C_MEMADD_SIZE_8BIT, &reg1Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_2, I2C_MEMADD_SIZE_8BIT, &reg2Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_3, I2C_MEMADD_SIZE_8BIT, &reg3Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_4, I2C_MEMADD_SIZE_8BIT, &reg4Config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(pI2CHandle, MAG_ADDRESS, CTRL_REG_5, I2C_MEMADD_SIZE_8BIT, &reg5Config, 1, HAL_MAX_DELAY);

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_DeInit
 * Description	:	De-initialize/ Reset LISM3MDL
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
	HAL_I2C_Mem_Write(pI2CHandle,(0x1C << 1), 0x21, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getMAG_X
 * Description	:	Returns magnetic field data in X axis in Gauss
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float LIS3MDL_getMAG_X(I2C_HandleTypeDef *pI2CHandle)
{

	uint16_t magX = 0;
	uint8_t data[2];
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_X_L, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	magX = (int16_t)(data[1] << 8) | (data[0]);

	return ((float)magX / magSensitivity);

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getMAG_Y
 * Description	:	Returns magnetic field data in Y axis in Gauss
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float LIS3MDL_getMAG_Y(I2C_HandleTypeDef *pI2CHandle)
{

	uint16_t magY = 0;
	uint8_t data[2];
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_Y_L, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	magY = (int16_t)(data[1] << 8) | (data[0]);

	return ((float)magY / magSensitivity);

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getMAG_Z
 * Description	:	Returns magnetic field data in Z axis in Gauss
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float LIS3MDL_getMAG_Z(I2C_HandleTypeDef *pI2CHandle)
{

	uint16_t magZ = 0;
	uint8_t data[2];
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, OUT_Z_L, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	magZ = (int16_t)(data[1] << 8) | (data[0]);

	return ((float)magZ / magSensitivity);

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	LIS3MDL_getTemperature_C
 * Description	:	Returns temperature sensor's output
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float LIS3MDL_getTemperature_C(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_L = 0;
	uint8_t Out_H = 0;
	int16_t tempRaw = 0;
	/*- Read register OUT_Y_L -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, TEMP_OUT_L, I2C_MEMADD_SIZE_8BIT, &Out_L, 1, HAL_MAX_DELAY);
	/*- Read register OUT_Y_H -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, &Out_H, 1, HAL_MAX_DELAY);

	// configure offset and scaling and return
	tempRaw =  ((int16_t)(Out_H << 8) | Out_L);
	return ((float)tempRaw / 256.0f) + 25;

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	XYZ_dataReady
 * Description	:	Returns 1 if new data is available for X, Y and Z axis.
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	uint8_t 1 or 0
 * Note		:	returns 1 if new set of data is available.
 * ------------------------------------------------------------------------------------------------------ */
uint8_t LIS3MDL_XYZ_DRDY(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data = 0;
	/*- Read Status Register -*/
	HAL_I2C_Mem_Read(pI2CHandle, MAG_ADDRESS, STATUS_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Check for ZYXDA bit -*/
	if ((data>>3) & 0x01)
	{
		/*- A new set of data is available  -*/
		return 1;
	}
	/*- A new set of data is not available yet  -*/
	return 0;
}
