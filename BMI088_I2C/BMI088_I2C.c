/*
 * 									BMI088_I2C.c
 *
 *  This file contains LIS3MDL driver API implementations for I2C communication.
 *
 */

#include "BMI088_I2C.h"

/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Acc_UserInit
 * Description	:	Initialize BMI088-Accelerometer with user defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:
 * Parameter 3	:
 * Parameter 4	:
 * Return Type	:	none (void)
 * Note		: Possible arguments:
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Acc_UserInit(I2C_HandleTypeDef *pI2CHandle)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Acc_DefaultInit
 * Description	:	Initialize BMI088-Accelerometer in pre-defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	In this initialization:
 * 	-	Accelerometer range is ±6g
 * 	-	Accelerometer ODR is 100Hz
 * 	-	Accelerometer bandwidth of low pass filter is normal
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Acc_DefaultInit(I2C_HandleTypeDef *pI2CHandle)
{
	 uint8_t data = 0;
	/*- Power Configuration: Active Mode -*/
	 HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_PWR_CONF, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	 data &= (uint8_t)~(0xFF);
	 HAL_I2C_Mem_Write(pI2CHandle,ACC_ADDRESS, ACC_PWR_CONF, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);


	/*- Power Control: Accelerometer ON -*/
	 data = 0;
	 HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	 data |= 0x04;
	 HAL_I2C_Mem_Write(pI2CHandle, ACC_ADDRESS, ACC_PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	 /*- Accelerometer Configuration -*/
	 data = 0;
	 HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_CONF, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	 data |= (1<<7); // 7th bit MUST ALWAYS BE 1 [just to be sure, but Default is already 1]
	 /*- 1. Accelerometer Bandwidth: Normal (default) -*/
	 /*- 2. Accelerometer ODR: 100Hz (default) -*/
	 HAL_I2C_Mem_Write(pI2CHandle, ACC_ADDRESS, ACC_CONF, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	 /*- Accelerometer Range -*/
	 /*- 1. Accelerometer Range: ±6g (default) -*/
}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Gyro_UserInit
 * Description	:	Initialize BMI088-Gyroscope with user defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:
 * Parameter 3	:
 * Parameter 4	:
 * Return Type	:	none (void)
 * Note		: Possible arguments:
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Gyro_UserInit(I2C_HandleTypeDef *pI2CHandle)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Gyro_DefaultInit
 * Description	:	Initialize BMI088-Gyroscope in pre-defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	In this initialization:
 * 	-
 * 	-
 * 	-
 * 	-
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Gyro_DefaultInit(I2C_HandleTypeDef *pI2CHandle)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Acc_DeInit
 * Description	:	De-initialize BMI088-Accelerometer
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	Reset all accelerometer configurations
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Acc_DeInit(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data = 0;
	/*- Soft Reset: Accelerometer -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (0xB6);
	HAL_I2C_Mem_Write(pI2CHandle, ACC_ADDRESS, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Gyro_DeInit
 * Description	:	De-initialize BMI088-Gyroscope
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	Reset all gyroscope configurations
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Gyro_DeInit(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data = 0;
	/*- Soft Reset: Gyroscope -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, GYRO_SOFTRESET, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (0xB6);
	HAL_I2C_Mem_Write(pI2CHandle, GYRO_ADDRESS, GYRO_SOFTRESET, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(50);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getAcc_X
 * Description	:	Returns Acceleration in X axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t BMI_getAcc_X(I2C_HandleTypeDef *pI2CHandle)
{
	return 0;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getAcc_Y
 * Description	:	Returns Acceleration in Y axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t BMI_getAcc_Y(I2C_HandleTypeDef *pI2CHandle)
{
	return 0;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getAcc_Z
 * Description	:	Returns Acceleration in Z axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t BMI_getAcc_Z(I2C_HandleTypeDef *pI2CHandle)
{
	return 0;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getGyro_X
 * Description	:	Returns angular velocity in X axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t BMI_getGyro_X(I2C_HandleTypeDef *pI2CHandle)
{
	return 0;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getGyro_Y
 * Description	:	Returns angular velocity in Y axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t BMI_getGyro_Y(I2C_HandleTypeDef *pI2CHandle)
{
	return 0;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getGyro_Z
 * Description	:	Returns angular velocity in Z axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	int16_t (16 bit signed value)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
int16_t BMI_getGyro_Z(I2C_HandleTypeDef *pI2CHandle)
{
	return 0;
}

