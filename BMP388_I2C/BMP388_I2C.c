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



/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_DefaultInit
 * Description	:	Initialize BMP388 with  pre-defined configuration
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	In this initialization:
 * 				- Both Temperature and Pressure sensors are enabled
 * 				- Over-sampling for pressure measurement is x2
 * 				- Over-sampling for temperature measurement is x1 (No over-sampling)
 * 				- ODR = 200Hz | Pre-scaler = 1 | Sampling Period = 5ms
 * 				- IIR Filter is in Bypass mode (no filtering)
 * ------------------------------------------------------------------------------------------------------ */
void BMP_DefaultInit(I2C_HandleTypeDef *pI2CHandle)
{
	/*- DeInit before proceeding -*/
	BMP_DeInit(pI2CHandle);

	/*- Get trimming coefficients for output compensation -*/
	compensationCoeff_t compCoeff;
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
	 *  	- osr_p (Over-sampling for pressure measurement) 	= x2
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
 * 			- Pressure and Temperature sensor is enabled.
 * 			- Possible arguments: OSR_P_x, OSR_T_x, ODR_x, IIR_COEFF_x
 * ------------------------------------------------------------------------------------------------------ */
void BMP_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t osr_p, uint8_t osr_t, uint8_t odr, uint8_t iir_coeff)
{
	/*- DeInit before proceeding -*/
	BMP_DeInit(pI2CHandle);

	/*- Get trimming coefficients for output compensation -*/
	compensationCoeff_t compCoeff;
	get_compensationCoeff(pI2CHandle, &compCoeff);

	uint8_t = 0;
	/*- Power COntrol: Enable Temperature and Pressure sensor -*/
	HAL_I2C_Mem_Read(pI2CHandle, BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (1<<0);	// Enable Pressure Sensor
	data |= (1<<1);	// Enable Temperature Sensor
	data |= (3<<4);	// Mode: Normal
	HAL_I2C_Mem_Write(pI2CHandle,BMP388_ADDRESS, PWR_CTRL, I2C_MEMADD_SIZE_8BIT, &data , 1, HAL_MAX_DELAY);

	/*- Over-sampling Settings: Temperature and Pressure measurements -*/
	data = 0;


	/*- Output Data Rate Configurations -*/
	data = 0;


	/*- IIR Filter Coefficients -*/
	data = 0;


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
	return 0;
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
	return 0;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	BMP_getAltitude_m
 * Description	:	Returns altitude in meters (m)
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Sea level pressure in hPa
 * Return Type	:	float
 * Note		:	Calculations obtained from > adafruit/Adafruit_BMP3XX
 * ------------------------------------------------------------------------------------------------------ */
float BMP_getAltitude_m(I2C_HandleTypeDef *pI2CHandle, float seaLevel_hPa)
{
	return 0;
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
	;
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
	;
}
