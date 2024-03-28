/*
 * 									BMI088_I2C.c
 *
 *  This file contains BMI088 driver API implementations for I2C communication.
 *
 */

#include "BMI088_I2C.h"

/*- Private Variables -*/
static uint8_t accRange = 6; /* Default Accelerometer Range: ±6g */
static uint16_t gyroRange = 2000; /* Default Gyroscope Range: ±2000°/s and Resolution 16.384 LSB/°/s */
/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Acc_UserInit
 * Description	:	Initialize BMI088-Accelerometer with user defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Filter Settings: Bandwidth of the Low Pass Filter
 * Parameter 3	:	Output Data Rate
 * Parameter 4	:	Measurement Range
 * Return Type	:	none (void)
 * Note		: Possible arguments: ACC_BWP_x, ACC_ODR_x, ACC_RANGE_x
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Acc_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t acc_bwp, uint8_t acc_odr, uint8_t acc_range)
{
	/*- Reset to default settings -*/
	BMI_Acc_DeInit(pI2CHandle);

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

	 /*- Accelerometer Configuration: ACC_CONF -*/
	 data = 0;
	 HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_CONF, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	 data |= (1<<7); // 7th bit MUST ALWAYS BE 1 [just to be sure, but Default is already 1]

	 /*- 1. Accelerometer Bandwidth -*/
	 if (acc_bwp == ACC_BWP_4FOLD)
	 {
		 /*- OSR4: 4-fold Over-sampling -*/
		   data &= ~(7<<4);
	 }
	 else if (acc_bwp == ACC_BWP_2FOLD)
	 {
		 /*- OSR2: 2-fold Over-sampling -*/
		   data &= ~(3<<5);
		   data |= (1<<4);
	 }
	 else
	 {
		 /*- Filter Settings: Normal [Default] -*/
		   data &= ~(7<<4);
		   data |= (1<<5);
	 }

	 /*- 2. Accelerometer ODR -*/
	 if (acc_odr == ACC_ODR_12_5HZ)
	 {
		 /*- ODR: 12.5Hz -*/
		   data &= ~(0xF<<0);
		   data |= (0x05);
	 }
	 else if (acc_odr == ACC_ODR_25HZ)
	 {
		 /*- ODR: 25Hz -*/
		 data &= ~(0xF<<0);
		 data |= (0x06);
	 }
	 else if (acc_odr == ACC_ODR_50HZ)
	 {
		 /*- ODR: 50Hz -*/
		 data &= ~(0xF<<0);
		 data |= (0x07);

	 }
	 else if (acc_odr == ACC_ODR_200HZ)
	 {
		 /*- ODR: 200Hz -*/
		 data &= ~(0xF<<0);
		 data |= (0x09);
	 }
	 else if (acc_odr == ACC_ODR_400HZ)
	 {
		 /*- ODR: 400Hz -*/
		 data &= ~(0xF<<0);
		 data |= (0x0A);
	 }
	 else if (acc_odr == ACC_ODR_800HZ)
	 {
		 /*- ODR: 800Hz -*/
		 data &= ~(0xF<<0);
		 data |= (0x0B);
	 }
	 else if (acc_odr == ACC_ODR_1600HZ)
	 {
		 /*- ODR: 1600Hz -*/
		 data &= ~(0xF<<0);
		 data |= (0x0C);
	 }
	 else
	 {
		 /*- ODR: 100Hz [Default] -*/
		 data &= ~(0xF<<0);
		 data |= (0x08);
	 }
	 // Configure ACC_CONF with above settings
	 HAL_I2C_Mem_Write(pI2CHandle, ACC_ADDRESS, ACC_CONF, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);


	 /*- Accelerometer Range -*/
	 data = 0;
	 HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_RANGE, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	 if (acc_range == ACC_RANGE_3G)
	 {
		 /*- Range: ±3g -*/
		 data &= ~(3<<0);
		 accRange = 3;

	 }
	 else if (acc_range == ACC_RANGE_12G)
	 {
		 /*- Range: ±12g -*/
		 data &= ~(3<<0);
		 data |= (0x02) ;
		 accRange = 12;
	 }
	 else if (acc_range == ACC_RANGE_24G)
	 {
		 /*- Range: ±24g -*/
		 data &= ~(3<<0);
		 data |= (0x03) ;
		 accRange = 24;
	 }
	 else
	 {
		 /*- Range: ±6g [Default] -*/
		 data &= ~(3<<0);
		 data |= (0x01) ;
		 accRange = 6;
	 }

	 // Configure ACC_RANGE with above settings
	 HAL_I2C_Mem_Write(pI2CHandle, ACC_ADDRESS, ACC_RANGE, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

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
	/*- Reset to default settings -*/
	BMI_Acc_DeInit(pI2CHandle);

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
	 accRange = 6;
}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Gyro_UserInit
 * Description	:	Initialize BMI088-Gyroscope with user defined configuration (in normal mode)
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Output Data Rate and Filter Bandwidth
 * Parameter 3	:	Angular Rate Range and Resolution
 * Return Type	:	none (void)
 * Note		: Possible arguments: GYRO_ODRx_BWx, GYRO_FS_x
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Gyro_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t gyro_odr_bw, uint8_t gyro_range)
{
	/*- Reset to default settings -*/
	BMI_Gyro_DeInit(pI2CHandle);

	uint8_t data = 0;

	/*- Gyroscope Range -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, GYRO_RANGE, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	data |= gyro_range;

	HAL_I2C_Mem_Write(pI2CHandle, ACC_ADDRESS, GYRO_RANGE, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Gyroscope Bandwidth -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, GYRO_BANDWIDTH, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	data |= gyro_odr_bw;

	HAL_I2C_Mem_Write(pI2CHandle, ACC_ADDRESS, GYRO_BANDWIDTH, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Setting private variable gyroRange for data calculations -*/
	if (gyro_range == GYRO_FS_1000)
	{
		gyroRange = 1000;
	}
	else if (gyro_range == GYRO_FS_500)
	{
		gyroRange = 500;
	}
	else if (gyro_range == GYRO_FS_250)
	{
		gyroRange = 250;
	}
	else if (gyro_range == GYRO_FS_125)
	{
		gyroRange = 125;
	}
	else
	{
		gyroRange = 2000;
	}
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_Gyro_DefaultInit
 * Description	:	Initialize BMI088-Gyroscope in pre-defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	In this initialization:
 * 	-	Range:	Full Scale = ±2000 °/s and Resolution = 16.384 LSB/°/s
 * 	-	Bandwidth:	ODR = 2000Hz and Filter Bandwidth = 532Hz
 * 	-	Power Mode	: Normal Mode
 * ------------------------------------------------------------------------------------------------------ */
void BMI_Gyro_DefaultInit(I2C_HandleTypeDef *pI2CHandle)
{
	/*- Reset to default settings -*/
	BMI_Gyro_DeInit(pI2CHandle);

	/*- In default configuration
	 *
	 * -> Gyroscope Range		: Full Scale = ±2000 °/s and Resolution 16.384 LSB/°/s
	 * -> Gyroscope Bandwidth	: ODR = 2000Hz and Filter Bandwidth = 532Hz
	 * -> Gyroscope Power Mode	: Normal Mode
	 *
	 *  -*/
	gyroRange = 2000;
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
 * Return Type	:	float
 * Note		: Output: Acceleration in m/s^2
 * ------------------------------------------------------------------------------------------------------ */
float BMI_getAcc_X(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_MSB = 0;
	uint8_t Out_LSB = 0;
	int16_t accX = 0;

	/*- Read register ACC_X_LSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, &Out_LSB, 1, HAL_MAX_DELAY);
	/*- Read register ACC_X_MSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_X_MSB, I2C_MEMADD_SIZE_8BIT, &Out_MSB, 1, HAL_MAX_DELAY);

	accX = ((int16_t)(Out_MSB << 8) | Out_LSB);
	return ((float)accX / 32768.0f * accRange * 9.81);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getAcc_Y
 * Description	:	Returns Acceleration in Y axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:	Output: Acceleration in m/s^2
 * ------------------------------------------------------------------------------------------------------ */
float BMI_getAcc_Y(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_MSB = 0;
	uint8_t Out_LSB = 0;
	int16_t accY = 0;
	/*- Read register ACC_Y_LSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_Y_LSB, I2C_MEMADD_SIZE_8BIT, &Out_LSB, 1, HAL_MAX_DELAY);
	/*- Read register ACC_Y_MSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_Y_MSB, I2C_MEMADD_SIZE_8BIT, &Out_MSB, 1, HAL_MAX_DELAY);

	accY = ((int16_t)(Out_MSB << 8) | Out_LSB);
	return ((float)accY / 32768.0f * accRange * 9.81);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getAcc_Z
 * Description	:	Returns Acceleration in Z axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:	Output: Acceleration in m/s^2
 * ------------------------------------------------------------------------------------------------------ */
float BMI_getAcc_Z(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_MSB = 0;
	uint8_t Out_LSB = 0;
	int16_t accZ = 0;

	/*- Read register ACC_Z_LSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_Z_LSB, I2C_MEMADD_SIZE_8BIT, &Out_LSB, 1, HAL_MAX_DELAY);
	/*- Read register ACC_Z_MSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, ACC_Z_MSB, I2C_MEMADD_SIZE_8BIT, &Out_MSB, 1, HAL_MAX_DELAY);

	accZ = ((int16_t)(Out_MSB << 8) | Out_LSB);
	return ((float)accZ / 32768.0f * accRange * 9.81);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getGyro_X
 * Description	:	Returns angular velocity in X axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float BMI_getGyro_X(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_MSB = 0;
	uint8_t Out_LSB = 0;
	int16_t rateX = 0;

	/*- Read register RATE_X_LSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, RATE_X_LSB, I2C_MEMADD_SIZE_8BIT, &Out_LSB, 1, HAL_MAX_DELAY);
	/*- Read register RATE_X_MSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, RATE_X_MSB, I2C_MEMADD_SIZE_8BIT, &Out_MSB, 1, HAL_MAX_DELAY);

	rateX = ((int16_t)(Out_MSB << 8) | Out_LSB);
	return ((float) rateX / 32768.0f * gyroRange);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getGyro_Y
 * Description	:	Returns angular velocity in Y axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:	Output: Angular velocity (degree per second °/s)
 * ------------------------------------------------------------------------------------------------------ */
float BMI_getGyro_Y(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_MSB = 0;
	uint8_t Out_LSB = 0;
	int16_t rateY = 0;

	/*- Read register RATE_Y_LSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, RATE_Y_LSB, I2C_MEMADD_SIZE_8BIT, &Out_LSB, 1, HAL_MAX_DELAY);
	/*- Read register RATE_Y_MSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, RATE_Y_MSB, I2C_MEMADD_SIZE_8BIT, &Out_MSB, 1, HAL_MAX_DELAY);

	rateY = ((int16_t)(Out_MSB << 8) | Out_LSB);
	return ((float) rateY / 32768.0f * gyroRange);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getGyro_Z
 * Description	:	Returns angular velocity in Z axis
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:	Output: Angular velocity (degree per second °/s)
 * ------------------------------------------------------------------------------------------------------ */
float BMI_getGyro_Z(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_MSB = 0;
	uint8_t Out_LSB = 0;
	int16_t rateZ = 0;

	/*- Read register RATE_Z_LSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, RATE_Z_LSB, I2C_MEMADD_SIZE_8BIT, &Out_LSB, 1, HAL_MAX_DELAY);
	/*- Read register RATE_Z_MSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, GYRO_ADDRESS, RATE_Z_MSB, I2C_MEMADD_SIZE_8BIT, &Out_MSB, 1, HAL_MAX_DELAY);

	rateZ = ((int16_t)(Out_MSB << 8) | Out_LSB);
	return ((float) rateZ / 32768.0f * gyroRange);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMI_getTemperature_C
 * Description	:	Returns temperature sensor's output
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:	Accelerometer MUST be enabled to get temperature sensor's output
 * ------------------------------------------------------------------------------------------------------ */
float BMI_getTemperature_C(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t Out_MSB = 0;
	uint8_t Out_LSB = 0;
	uint16_t Temp_uint11 = 0;
	int16_t Temp_int11 = 0;

	/*- Read register TEMP_LSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, TEMP_LSB, I2C_MEMADD_SIZE_8BIT, &Out_LSB, 1, HAL_MAX_DELAY);
	/*- Read register TEMP_MSB -*/
	HAL_I2C_Mem_Read(pI2CHandle, ACC_ADDRESS, TEMP_MSB, I2C_MEMADD_SIZE_8BIT, &Out_MSB, 1, HAL_MAX_DELAY);

	/*- Calculation and logic from Data sheet -*/
	Temp_uint11 = (Out_MSB * 8) + (Out_LSB / 32);
	if (Temp_uint11 > 1023)
	{
		Temp_int11 = Temp_uint11 - 2048;
	}
	else
	{
		Temp_int11 = Temp_uint11;
	}

	return ((float)Temp_int11 * 0.125f + 23.0f);
}
