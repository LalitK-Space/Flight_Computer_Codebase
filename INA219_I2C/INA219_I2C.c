
/*
 * 									INA219_I2C.c
 *
 *  This file contains INA219 driver API implementations for I2C communication.
 *
 */
#include "INA219_I2C.h"


scalingData_t INAscaling;

/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_setConfiguration
 * Description	:	Private Function: To Configure the Configuration Register w.r.t to calibration settings
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	Bus Voltage
 * Parameter 3	:	PGA (Shunt Voltage) gain
 * Parameter 4	:	Bus ADC resolution
 * Parameter 5	:	Shunt ADC resolution
 * Return Type	:	none (void)
 * Note		:       Operating Mode is Shunt and Bus and, Continuous.
 * 			- Possible arguments: INA219_BUSVOLTAGE_x, INA219_PGA_x, INA219_BADC_x, INA219_SADC_x
 * ------------------------------------------------------------------------------------------------------ */
/*-  -*/
static void INA219_setConfiguration(I2C_HandleTypeDef *pI2CHandle, uint8_t busVoltage, uint8_t pga, uint8_t bADC, uint8_t sADC)
{
	uint8_t data[2];
	uint16_t configReg = 0;

	/*- SET: MODE -> Shunt and Bus, continuous -*/
	configReg |= (7<<0);
	/*- SET: SADC -*/

	configReg |= (sADC<<3);

	/*- SET: BADC -*/
	configReg |= (bADC<<7);

	/*- SET: PG -*/
	configReg |= (pga<<11);

	/*- SET: Bus Voltage Range -*/
	configReg |= (busVoltage<<13);

	/*- Configure with above settings -*/
	data[0] = 0;
	data[1] = 0;
	data[0] = ((configReg >> 8) & 0xFF);
	data[1] = ((configReg >> 0) & 0xFF);
	HAL_I2C_Mem_Write(pI2CHandle, INA219_ADDRESS, INA219_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_setCalibration_32V_2A
 * Description	:	Configure and calibrate the Current and Power Registers to measure
 * 			currents up to 2A and voltages up to 32V.
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:       Operating Mode (Configuration Register) is Shunt and Bus and, Continuous.
 * ------------------------------------------------------------------------------------------------------ */
void INA219_setCalibration_32V_2A(I2C_HandleTypeDef *pI2CHandle)
{
	/*- First, Reset scaling variables -*/
	memset(&INAscaling, 0, sizeof(INAscaling));
	/*- Calibration and scaling values -*/
	uint16_t calibration = 4096;
	INAscaling.currentDivider_mA = 10.0;
	INAscaling.powerMultiplier_mW = 2.0;

	/*- Calibrating: Calibration Register -*/
	uint8_t data[2];
	data[0] = ((calibration >> 8) & 0xFF);
	data[1] = ((calibration >> 0) & 0xFF);
	HAL_I2C_Mem_Write(pI2CHandle, INA219_ADDRESS, INA219_CALIBRATION, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

	/*- Configurations -*/
	INA219_setConfiguration(pI2CHandle, INA219_BUSVOLTAGE_32V, INA219_PGA_8_320MV, INA219_BADC_12BIT, INA219_SADC_12BIT);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_setCalibration_32V_1A
 * Description	:	Configure and calibrate the Current and Power Registers to measure
 * 			currents up to 1A and voltages up to 32V.
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	Operating Mode (Configuration Register) is Shunt and Bus and Continuous.
 * ------------------------------------------------------------------------------------------------------ */
void INA219_setCalibration_32V_1A(I2C_HandleTypeDef *pI2CHandle)
{
	/*- First, Reset scaling variables -*/
	memset(&INAscaling, 0, sizeof(INAscaling));
	/*- Calibration and scaling values -*/
	uint16_t calibration = 10240;
	INAscaling.currentDivider_mA = 25.0;
	INAscaling.powerMultiplier_mW = 0.8;

	/*- Calibrating: Calibration Register -*/
	uint8_t data[2];
	data[0] = ((calibration >> 8) & 0xFF);
	data[1] = ((calibration >> 0) & 0xFF);
	HAL_I2C_Mem_Write(pI2CHandle, INA219_ADDRESS, INA219_CALIBRATION, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

	/*- Configurations -*/
	INA219_setConfiguration(pI2CHandle, INA219_BUSVOLTAGE_32V, INA219_PGA_8_320MV, INA219_BADC_12BIT, INA219_SADC_12BIT);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_setCalibration_16V_400mA
 * Description	:	Configure and calibrate the Current and Power Registers to measure
 * 			currents up to 400mA and voltages up to 16V.
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	Highest precision for current measurement.
 * 			- Operating Mode (Configuration Register) is Shunt and Bus and Continuous.
 * ------------------------------------------------------------------------------------------------------ */
void INA219_setCalibration_16V_400mA(I2C_HandleTypeDef *pI2CHandle)
{
	/*- First, Reset scaling variables -*/
	memset(&INAscaling, 0, sizeof(INAscaling));
	/*- Calibration and scaling values -*/
	uint16_t calibration = 8192;
	INAscaling.currentDivider_mA = 20.0;
	INAscaling.powerMultiplier_mW = 1.0;

	/*- Calibrating: Calibration Register -*/
	uint8_t data[2];
	data[0] = ((calibration >> 8) & 0xFF);
	data[1] = ((calibration >> 0) & 0xFF);
	HAL_I2C_Mem_Write(pI2CHandle, INA219_ADDRESS, INA219_CALIBRATION, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

	/*- Configurations -*/
	INA219_setConfiguration(pI2CHandle, INA219_BUSVOLTAGE_16V, INA219_PGA_1_40MV, INA219_BADC_12BIT, INA219_SADC_12BIT);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_DeInit
 * Description	:	Resets INA219
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	(same as Power-on reset) All user registers are overwritten with their default values
 * ------------------------------------------------------------------------------------------------------ */
void INA219_DeInit(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data[2];
	uint16_t dataR = 0;

	/*- Set: reset bit -*/
	dataR |= (1<<15);
	data[0] = ((dataR >> 8) & 0xFF);
	data[1] = ((dataR >> 0) & 0xFF);
	HAL_I2C_Mem_Write(pI2CHandle, INA219_ADDRESS, INA219_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	/*- This bit self clears -*/
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_getShuntVoltage_mV
 * Description	:	Returns shunt voltage in milli-Volts (mV)
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float INA219_getShuntVoltage_mV(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data[2];
	uint16_t dataR = 0;
	/*- Read Shunt Voltage Register -*/
	HAL_I2C_Mem_Read(pI2CHandle, INA219_ADDRESS, INA219_SHUNT_VOLTAGE, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	dataR = ((data[0] << 8) | data[1]);

	/*- Return Shunt Voltage mV -*/
	return (float)dataR * 0.01f;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_getBusVoltage_V
 * Description	:	Returns bus voltage in Volts (V)
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float INA219_getBusVoltage_V(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data[2];
	uint16_t dataR = 0;
	/*- Read Bus Voltage Register -*/
	HAL_I2C_Mem_Read(pI2CHandle, INA219_ADDRESS, INA219_BUS_VOLTAGE, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	dataR = ((data[0] << 8) | data[1]);
	dataR = (dataR >> 3) * 4;

	/*- Return Bus Voltage V -*/
	return (float) dataR * 0.001f;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_getPower_mW
 * Description	:	Returns power in milli-Watts (mW)
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float INA219_getPower_mW(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data[2];
	uint16_t dataR = 0;
	/*- Read Power Register -*/
	HAL_I2C_Mem_Read(pI2CHandle, INA219_ADDRESS, INA219_POWER, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	dataR = ((data[0] << 8) | data[1]);

	/*- Return Power mW -*/
	return (float) dataR * INAscaling.powerMultiplier_mW;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	INA219_getcurrent_mA
 * Description	:	Returns current in milli-Amperes (mA)
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	float
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
float INA219_getcurrent_mA(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data[2];
	uint16_t dataR = 0;
	/*- Read Current Register -*/
	HAL_I2C_Mem_Read(pI2CHandle, INA219_ADDRESS, INA219_CURRENT, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	dataR = ((data[0] << 8) | data[1]);

	/*- Return Current mA -*/
	return (float) dataR / INAscaling.currentDivider_mA;
}
