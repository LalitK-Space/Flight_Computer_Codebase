
/*
 * 									BMP388_I2C.h
 *
 * This file contains all the BMP388 related APIs supported by the driver.
 *
 */


#ifndef INC_BMP388_I2C_H_
#define INC_BMP388_I2C_H_

/* MCU Specific Header file */
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* BMP388 I2C Address */
#define BMP388_ADDRESS	(0x76 << 1)

/* BMP388 Register Addresses */
#define CHIP_ID			(0x00)
#define ERR_REG			(0x02)
#define STATUS			(0x03)
#define DATA_0			(0x04)
#define DATA_1			(0x05)
#define DATA_2			(0x06)
#define DATA_3			(0x07)
#define DATA_4			(0x08)
#define DATA_5			(0x09)
#define BMP_SENSORTIME_0	(0x0C)
#define BMP_SENSORTIME_1	(0x0D)
#define BMP_SENSORTIME_2	(0x0E)
#define EVENT			(0x10)
#define INT_STATUS		(0x11)
#define FIFO_LENGTH_0	(0x12)
#define FIFO_LENGTH_1	(0x13)
#define FIFO_DATA		(0x14)
#define FIFO_WTM_0		(0x15)
#define FIFO_WTM_1		(0x16)
#define FIFO_CONFIG_1	(0x17)
#define FIFO_CONFIG_2	(0x18)
#define INT_CTRL		(0x19)
#define IF_CONF			(0x1A)
#define PWR_CTRL		(0x1B)
#define OSR				(0x1C)
#define ODR				(0x1D)
#define CONFIG			(0x1F)
#define CMD				(0x7E)

/* Trimming Coefficients Register Addresses */
#define	NVM_PAR_T1		(0x31)
#define	NVM_PAR_T2		(0x33)
#define	NVM_PAR_T3		(0x35)
#define	NVM_PAR_P1		(0x36)
#define	NVM_PAR_P2		(0x38)
#define	NVM_PAR_P3		(0x3A)
#define	NVM_PAR_P4		(0x3B)
#define	NVM_PAR_P5		(0x3C)
#define	NVM_PAR_P6		(0x3E)
#define	NVM_PAR_P7		(0x40)
#define	NVM_PAR_P8		(0x41)
#define	NVM_PAR_P9		(0x42)
#define	NVM_PAR_P10		(0x44)
#define	NVM_PAR_P11		(0x45)


/* Trimming Coefficients for Output Compensation */
typedef struct
{
	uint16_t	PAR_T1;
	uint16_t	PAR_T2;
	int8_t		PAR_T3;
	int16_t		PAR_P1;
	int16_t		PAR_P2;
	int8_t		PAR_P3;
	int8_t		PAR_P4;
	uint16_t	PAR_P5;
	uint16_t	PAR_P6;
	int8_t		PAR_P7;
	int8_t		PAR_P8;
	int16_t		PAR_P9;
	int8_t 		PAR_P10;
	int8_t		PAR_P11;
	int64_t		t_lin;

}compensationCoeff_t;


/* BMP388 APIs*/

// Initialization
void BMP_DefaultInit(I2C_HandleTypeDef *pI2CHandle);
void BMP_UserInit(I2C_HandleTypeDef *pI2CHandle);

// DeInit
void BMP_DeInit(I2C_HandleTypeDef *pI2CHandle);

// Get Sensor Values
float BMP_getTemperature_C(I2C_HandleTypeDef *pI2CHandle);
float BMP_getPressure_Pa(I2C_HandleTypeDef *pI2CHandle);
float BMP_getAltitude_m(I2C_HandleTypeDef *pI2CHandle, float seaLevel_hPa);

#endif /* INC_BMP388_I2C_H_ */
