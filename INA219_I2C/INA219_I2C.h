
/*
 * 									INA219_I2C.h
 *
 * This file contains all the INA219 related APIs supported by the driver.
 *
 */
#ifndef INC_INA219_I2C_H_
#define INC_INA219_I2C_H_

/* MCU Specific Header file */
#include "stm32f4xx_hal.h"
#include <string.h>

/* INA219 I2C Address */
#define INA219_ADDRESS	(0x40 << 1)

/* INA219 Register Addresses */
#define INA219_CONFIGURATION	(0x00)
#define INA219_SHUNT_VOLTAGE	(0x01)
#define INA219_BUS_VOLTAGE	(0x02)
#define INA219_POWER		(0x03)
#define INA219_CURRENT		(0x04)
#define INA219_CALIBRATION	(0x05)

/* INA219 Configuration: Bus Voltage */
#define INA219_BUSVOLTAGE_16V	(0x0)
#define INA219_BUSVOLTAGE_32V	(0x1)

/* INA219 Configuration: PGA */
#define INA219_PGA_1_40MV	(0x0)
#define INA219_PGA_2_80MV	(0x1)
#define INA219_PGA_4_160V	(0x2)
#define INA219_PGA_8_320MV	(0x3)

/* INA219 Configuration: ADC Resolution */
#define INA219_BADC_9BIT	(0x0)	/* Conversion Time: 84uS */
#define INA219_BADC_10BIT	(0x1)	/* Conversion Time: 148uS */
#define INA219_BADC_11BIT	(0x2)	/* Conversion Time: 276uS */
#define INA219_BADC_12BIT	(0x3)	/* Conversion Time: 532uS */
#define INA219_BADC_2S		(0x9)	/* Conversion Time: 1.06mS */
#define INA219_BADC_4S		(0xA)	/* Conversion Time: 2.13mS */
#define INA219_BADC_8S		(0xB)	/* Conversion Time: 4.26mS */
#define INA219_BADC_16S		(0xC)	/* Conversion Time: 8.51mS */
#define INA219_BADC_32S		(0xD)	/* Conversion Time: 17.02mS */
#define INA219_BADC_64S		(0xE)	/* Conversion Time: 34.05mS */
#define INA219_BADC_128S	(0xF)	/* Conversion Time: 68.10mS */

#define INA219_SADC_9BIT	(0x0)	/* Conversion Time: 84uS */
#define INA219_SADC_10BIT	(0x1)	/* Conversion Time: 148uS */
#define INA219_SADC_11BIT	(0x2)	/* Conversion Time: 276uS */
#define INA219_SADC_12BIT	(0x3)	/* Conversion Time: 532uS */
#define INA219_SADC_2S		(0x9)	/* Conversion Time: 1.06mS */
#define INA219_SADC_4S		(0xA)	/* Conversion Time: 2.13mS */
#define INA219_SADC_8S		(0xB)	/* Conversion Time: 4.26mS */
#define INA219_SADC_16S		(0xC)	/* Conversion Time: 8.51mS */
#define INA219_SADC_32S		(0xD)	/* Conversion Time: 17.02mS */
#define INA219_SADC_64S		(0xE)	/* Conversion Time: 34.05mS */
#define INA219_SADC_128S	(0xF)	/* Conversion Time: 68.10mS */


/* Structure to hold scaling values */
typedef struct
{

	float currentDivider_mA;
	float powerMultiplier_mW;

}scalingData_t;

/* INA219 APIs*/

// Initialization and Calibration
void INA219_setCalibration_32V_2A(I2C_HandleTypeDef *pI2CHandle);
void INA219_setCalibration_32V_1A(I2C_HandleTypeDef *pI2CHandle);
void INA219_setCalibration_16V_400mA(I2C_HandleTypeDef *pI2CHandle);

// DeInit
void INA219_DeInit(I2C_HandleTypeDef *pI2CHandle);

// Get Values
float INA219_getShuntVoltage_mV(I2C_HandleTypeDef *pI2CHandle);
float INA219_getBusVoltage_V(I2C_HandleTypeDef *pI2CHandle);
float INA219_getPower_mW(I2C_HandleTypeDef *pI2CHandle);
float INA219_getcurrent_mA(I2C_HandleTypeDef *pI2CHandle);

#endif /* INC_INA219_I2C_H_ */
