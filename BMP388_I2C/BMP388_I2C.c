/*
 * 									BMP388_I2C.c
 *
 *  This file contains BMP388 driver API implementations for I2C communication.
 *
 */

#include "BMP388_I2C.h"

/*- Private Helper Functions -*/
static void get_compensationCoeff(I2C_HandleTypeDef *pI2CHandle, compensationCoeff_t *pcompensationCoeff);
static int64_t compensateTemperature(int32_t rawTemp, compensationCoeff_t *pcompensationCoeff);
static uint64_t compensatePressure(int32_t rawPress, compensationCoeff_t *pcompensationCoeff);
static void enable_drdy(I2C_HandleTypeDef *pI2CHandle);

compensationCoeff_t compCoeff;

/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_DefaultInit
 * Description	:	Initialize BMP388 with  pre-defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	In this initialization:
 * 				- Both Temperature and Pressure sensors are enabled
 * 				- Over-sampling for pressure measurement is x4
 * 				- Over-sampling for temperature measurement is x1 (No over-sampling)
 * 				- ODR = 200Hz | Pre-scaler = 1 | Sampling Period = 5ms
 * 				- IIR Filter is in Bypass mode (no filtering)
 * 				- Temperature/Pressure data ready interrupt for INT pin and INT_STATUS is enabled
 * ------------------------------------------------------------------------------------------------------ */
void BMP_DefaultInit(I2C_HandleTypeDef *pI2CHandle)
{
	/*- DeInit before proceeding -*/
	BMP_DeInit(pI2CHandle);

	/*- Get trimming coefficients for output compensation -*/
	get_compensationCoeff(pI2CHandle, &compCoeff);

	uint8_t data = 0;
	/*- Power COntrol: Enable Temperature and Pressure sensor -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (1<<0);	// Enable Pressure Sensor
	data |= (1<<1);	// Enable Temperature Sensor
	data |= (3<<4);	// Mode: Normal
	HAL_I2C_Mem_Write(pI2CHandle,BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

	/*- Over-sampling Settings: Temperature and Pressure measurements -*/
	/*- Default OSR configurations:
	 *  	- osr_p (Over-sampling for pressure measurement) 	= x4
	 *  	- osr_t (Over-sampling for temperature measurement) = x1 (no over-sampling)
	 *  -*/

	/*- Output Data Rate Configurations -*/
	/*- Default ODR configurations:
	 *  	- ODR = 200Hz | Pre-scaler = 1 | Sampling Period = 5ms
	 *  -*/

	/*- IIR Filter Coefficients -*/
	/*- Default IIR Filter Coefficient:
	 *  	- Bypass mode (no filtering)
	 *  -*/

	/* Enable Data Ready  interrupt */
	enable_drdy(pI2CHandle);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_UserInit
 * Description	:	Initialize BMP388 with user defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Over-sampling setting for pressure measurement
 * Parameter 3	:	Over-sampling setting for temperature measurement
 * Parameter 4	:	Output Data Rate
 * Parameter 5	:	IIR FIlter Coefficients
 * Return Type	:	none (void)
 * Note		:
 * 			- Please follow recommended values to get proper and valid readings.
 * 			- Pressure and Temperature sensor is enabled.
 * 			- Temperature/Pressure data ready interrupt for INT pin and INT_STATUS is enabled
 * 			- Possible arguments: OSR_P_x, OSR_T_x, ODR_x, IIR_COEFF_x
 * ------------------------------------------------------------------------------------------------------ */
void BMP_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t osr_p, uint8_t osr_t, uint8_t odr, uint8_t iir_coeff)
{
	/*- DeInit before proceeding -*/
	BMP_DeInit(pI2CHandle);

	/*- Get trimming coefficients for output compensation -*/
	get_compensationCoeff(pI2CHandle, &compCoeff);

	uint8_t data = 0;
	/*- Power Control: Enable Temperature and Pressure sensor -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (1<<0);	// Enable Pressure Sensor
	data |= (1<<1);	// Enable Temperature Sensor
	HAL_I2C_Mem_Write(pI2CHandle,BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);


	/*- Over-sampling Settings: Temperature and Pressure measurements -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, OSR, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data &= ~(0x3F);		// Bits[7:6] are Reserved
	data |= (osr_p << 0);	// Configure OS for Pressure measurement
	data |= (osr_t << 3);	// Configure OS for Temperature measurement
	HAL_I2C_Mem_Write(pI2CHandle,BMP388_ADDRESS, OSR, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

	/*- Output Data Rate Configurations -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, ODR, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data &= ~(0x1F);		// Bits[7:5] are Reserved
	data |= (odr << 0);		// Configure ODR
	HAL_I2C_Mem_Write(pI2CHandle,BMP388_ADDRESS, ODR, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

	/*- Power Control: Mode = Normal [Need to be done here] -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (3<<4);	// Mode: Normal
	HAL_I2C_Mem_Write(pI2CHandle,BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);


	/*- IIR Filter Coefficients -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data &= ~(7<<1);			// Bits[7:4] and bit[1] are Reserved
	data |= (iir_coeff << 1);	// Configure Filter Coefficient
	HAL_I2C_Mem_Write(pI2CHandle,BMP388_ADDRESS, CONFIG, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

	/* Enable Data Ready  interrupt */
	enable_drdy(pI2CHandle);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_DeInit
 * Description	:	De-initialize/soft-reset the BMP388
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	(Soft-reset) All user configuration settings are overwritten with their default state
 * ------------------------------------------------------------------------------------------------------ */
void BMP_DeInit(I2C_HandleTypeDef *pI2CHandle)
{

	uint8_t data = 0;
	data |= (0xB6);		// CMD:	Soft-Reset
	HAL_I2C_Mem_Write(pI2CHandle, BMP388_ADDRESS, CMD, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_getTemperature_C
 * Description	:	Returns temperature in degree celsius (°C)
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float BMP_getTemperature_C(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data[3];
	int32_t rawTemp;
	int64_t compensatedTemp;

	/* Get Raw Temperature value */
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, DATA_3, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);
	rawTemp = (int32_t)(data[2] << 16) | (int32_t)(data[1] << 8) | (data[0]);

	/* Get Compensated Temperature value */
	compensatedTemp = compensateTemperature(rawTemp, &compCoeff);

	/* Return Compensated Temperature in float and degree celsius */
	return (float) compensatedTemp/100;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_getPressure_Pa
 * Description	:	Returns pressure in Pascal (Pa)
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float BMP_getPressure_Pa(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data[3];
	int32_t rawPress;
	int64_t compensatedPress;

	/* Get Raw Pressure value */
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, DATA_0, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);
	rawPress = (int32_t)(data[2] << 16) | (int32_t)(data[1] << 8) | (data[0]);

	/* Get Compensated Pressure value */
	compensatedPress = compensatePressure(rawPress, &compCoeff);

	/* Return Compensated Pressure in float and Pascal */
	return (float) compensatedPress/100;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_getAltitude_m
 * Description	:	Returns altitude in meters (m)
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Sea level pressure in hPa
 * Return Type	:	float
 * Note		:	Calculations obtained from > adafruit/Adafruit_BMP3XX and BMP180 Datasheet.
 * ------------------------------------------------------------------------------------------------------ */
float BMP_getAltitude_m(I2C_HandleTypeDef *pI2CHandle, float seaLevel_hPa)
{
	float measuredPress = 0;
	float altitude = 0;

	measuredPress = (BMP_getPressure_Pa(pI2CHandle) / 100.0);	// /100 to convert Pa to hPa
	altitude = 44330.0f * (1.0f - pow((measuredPress / seaLevel_hPa), (1/5.225)));

	return altitude;
}



/* ------------------------------------------------------------------------------------------------------
 * Name		:	get_compensationCoeff
 * Description	:	Static function to get trimming coefficients for output compensation
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 1	:	Pointer to compensationCoeff_t structure Handle
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
static void get_compensationCoeff(I2C_HandleTypeDef *pI2CHandle, compensationCoeff_t *pcompensationCoeff)
{
	uint8_t data[2];

	/*- PAR_T1 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_T1, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_T1 = (uint16_t)data[1] << 8 | data[0];

	/*- PAR_T2 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_T2, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_T2 = (uint16_t)data[1] << 8 | data[0];

	/*- PAR_T3 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_T3, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_T3 = data[0];

	/*- PAR_P1 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P1, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P1 = (uint16_t)data[1] << 8 | data[0];

	/*- PAR_P2 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P2, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P2 = (uint16_t)data[1] << 8 | data[0];

	/*- PAR_P3 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P3, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P3 = data[0];

	/*- PAR_P4 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P4, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P4 = data[0];

	/*- PAR_P5 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P5, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P5 = (uint16_t)data[1] << 8 | data[0];

	/*- PAR_P6 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P6, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P6 = (uint16_t)data[1] << 8 | data[0];

	/*- PAR_P7 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P7, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P7 = data[0];

	/*- PAR_P8 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P8, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P8 = data[0];

	/*- PAR_P9 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P9, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P9 = (uint16_t)data[1] << 8 | data[0];

	/*- PAR_P10 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P10, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P10 = data[0];

	/*- PAR_P11 -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, NVM_PAR_P11, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	pcompensationCoeff->PAR_P11 = data[0];

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	compensateTemperature
 * Description	:	Static function to compensate the raw temperature data
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 1	:	Pointer to compensationCoeff_t structure Handle
 * Return Type	:	int64_t
 * Note		:	Calculations obtained from > boschsensortec/BMP3_SensorAPI
 * 				(https://github.com/boschsensortec/BMP3_SensorAPI/blob/master/bmp3.c)
 * ------------------------------------------------------------------------------------------------------ */
static int64_t compensateTemperature(int32_t rawTemp, compensationCoeff_t *pcompensationCoeff)
{
	/* Temporary variables for calculation */
	  int64_t partial_data1;
	  int64_t partial_data2;
	  int64_t partial_data3;
	  int64_t partial_data4;
	  int64_t partial_data5;
	  int64_t partial_data6;
	  int64_t compensatedTemp;

	  /* Compensating Raw Temperature data */
	  partial_data1 = ((int64_t)rawTemp - (256 * pcompensationCoeff->PAR_T1));
	  partial_data2 = pcompensationCoeff->PAR_T2 * partial_data1;
	  partial_data3 = (partial_data1 * partial_data1);
	  partial_data4 = (int64_t)partial_data3 * pcompensationCoeff->PAR_T3;
	  partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
	  partial_data6 = partial_data5 / 4294967296;
	  // Saving partial_data6 for pressure compensation calculations
	  pcompensationCoeff->t_lin = partial_data6;

	  compensatedTemp = (int64_t)((partial_data6 * 25) / 16384);

	  return compensatedTemp;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	compensatePressure
 * Description	:	Static function to compensate the raw pressure data
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 1	:	Pointer to compensationCoeff_t structure Handle
 * Return Type	:	uint64_t
 * Note		:	Calculations obtained from > boschsensortec/BMP3_SensorAPI
 * 				(https://github.com/boschsensortec/BMP3_SensorAPI/blob/master/bmp3.c)
 * ------------------------------------------------------------------------------------------------------ */
static uint64_t compensatePressure(int32_t rawPress, compensationCoeff_t *pcompensationCoeff)
{
	/* Temporary variables for calculation */
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t offset;
	int64_t sensitivity;
	uint64_t compensatedPress;

	/* Compensating Raw Pressure data */
	partial_data1 = pcompensationCoeff->t_lin * pcompensationCoeff->t_lin;
	partial_data2 = partial_data1 / 64;
	partial_data3 = (partial_data2 * pcompensationCoeff->t_lin) / 256;
	partial_data4 = (pcompensationCoeff->PAR_P8 * partial_data3) / 32;
	partial_data5 = (pcompensationCoeff->PAR_P7 * partial_data1) * 16;
	partial_data6 = (pcompensationCoeff->PAR_P6 * pcompensationCoeff->t_lin) * 4194304;
	offset = (pcompensationCoeff->PAR_P5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;
	partial_data2 = (pcompensationCoeff->PAR_P4 * partial_data3) / 32;
	partial_data4 = (pcompensationCoeff->PAR_P3 * partial_data1) * 4;
	partial_data5 = (pcompensationCoeff->PAR_P2 - 16384) * pcompensationCoeff->t_lin * 2097152;
	sensitivity = ((pcompensationCoeff->PAR_P1 - 16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5;
	partial_data1 = (sensitivity / 16777216) * rawPress;
	partial_data2 = pcompensationCoeff->PAR_P10 * pcompensationCoeff->t_lin;
	partial_data3 = partial_data2 + (65536 * pcompensationCoeff->PAR_P9);
	partial_data4 = (partial_data3 * rawPress) / 8192;
    /* dividing by 10 followed by multiplying by 10
     * To avoid overflow caused by (uncomp_data->pressure * partial_data4)
     */
	partial_data5 = (rawPress * (partial_data4 / 10)) / 512;
	partial_data5 = partial_data5 * 10;
	partial_data6 = (int64_t)((uint64_t)rawPress * (uint64_t)rawPress);
	partial_data2 = (pcompensationCoeff->PAR_P11 * partial_data6) / 65536;
	partial_data3 = (partial_data2 * rawPress) / 128;
	partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;

	compensatedPress = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

	return compensatedPress;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	enable_drdy
 * Description	:	Static function to enable temperature/pressure data ready interrupt for INT pin and INT_STATUS
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	Interrupt configuration just for drdy_en bit (Bit[6] of INT_CTRL)
 *
 * ------------------------------------------------------------------------------------------------------ */
static void enable_drdy(I2C_HandleTypeDef *pI2CHandle)
{

	  uint8_t data = 0;
	  /* Read INT_CTRL Register */
	  HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, INT_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	  /* Enable drdy_en, set bit[6] */
	  data |= (1<<6);
	  HAL_I2C_Mem_Write(pI2CHandle, BMP388_ADDRESS, INT_CTRL, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_INT_STATUS_drdy
 * Description	:	Reads the INT_STATUS register and returns the drdy bit status.
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	uint8_t, drdy bit status  (1 if data ready, 0 otherwise)
 * Note		:	Status is cleared after reading
 *
 * ------------------------------------------------------------------------------------------------------ */
uint8_t BMP_INT_STATUS_drdy(I2C_HandleTypeDef *pI2CHandle)
{
	  uint8_t data = 0;
	  /* Read INT_STATUS */
	  HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, INT_STATUS, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	  /* Extract DRDY bit[3] and return */
	  return (data >> 3) & 0x01;

}
