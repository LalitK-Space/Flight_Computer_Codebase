
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
#include <math.h>

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

/* IIR FIlter Coefficients */
#define IIR_COEFF_0			(0x00)	/*- Bypass mode (no filtering) -*/
#define IIR_COEFF_1			(0x01)
#define IIR_COEFF_3			(0x02)
#define IIR_COEFF_7			(0x03)
#define IIR_COEFF_15		(0x04)
#define IIR_COEFF_31		(0x05)
#define IIR_COEFF_63		(0x06)
#define IIR_COEFF_127		(0x07)

/* ODR: Output Data Rate */
#define ODR_200			(0x00)	/*- ODR: 200Hz 		| Pre-scaler: 1      | Sampling Period: 5ms     -*/
#define ODR_100			(0x01)	/*- ODR: 100Hz 		| Pre-scaler: 2      | Sampling Period: 10ms    -*/
#define ODR_50			(0x02)	/*- ODR: 50Hz  		| Pre-scaler: 4      | Sampling Period: 20ms    -*/
#define ODR_25			(0x03)	/*- ODR: 25Hz  		| Pre-scaler: 8      | Sampling Period: 40ms    -*/
#define ODR_12P5		(0x04)	/*- ODR: 25/2Hz 	| Pre-scaler: 16     | Sampling Period: 80ms    -*/
#define ODR_6P25		(0x05)	/*- ODR: 25/4z 		| Pre-scaler: 32     | Sampling Period: 160ms   -*/
#define ODR_3P1			(0x06)	/*- ODR: 25/8Hz 	| Pre-scaler: 64     | Sampling Period: 320ms   -*/
#define ODR_1P5			(0x07)	/*- ODR: 25/16Hz 	| Pre-scaler: 127    | Sampling Period: 640ms   -*/
#define ODR_0P78		(0x08)	/*- ODR: 25/32Hz 	| Pre-scaler: 256    | Sampling Period: 1.280s  -*/
#define ODR_0P39		(0x09)	/*- ODR: 25/64Hz 	| Pre-scaler: 512    | Sampling Period: 2.560s  -*/
#define ODR_0P2			(0x0A)	/*- ODR: 25/128Hz 	| Pre-scaler: 1024   | Sampling Period: 5.120s  -*/
#define ODR_0P1			(0x0B)	/*- ODR: 25/256Hz   | Pre-scaler: 2048   | Sampling Period: 10.24s  -*/
#define ODR_0P05		(0x0C)	/*- ODR: 25/512Hz   | Pre-scaler: 4096 	 | Sampling Period: 20.48s  -*/
#define ODR_0P02		(0x0D)	/*- ODR: 25/1024Hz  | Pre-scaler: 8192   | Sampling Period: 40.96s  -*/
#define ODR_0P01		(0x0E)	/*- ODR: 25/2048Hz  | Pre-scaler: 16384  | Sampling Period: 81.92s  -*/
#define ODR_0P006		(0x0F)	/*- ODR: 25/4096Hz  | Pre-scaler: 32768  | Sampling Period: 163.84s -*/
#define ODR_0P003		(0x10)	/*- ODR: 25/8192Hz 	| Pre-scaler: 65536  | Sampling Period: 327.68s -*/
#define ODR_0P0015		(0x11)	/*- ODR: 25/16284Hz | Pre-scaler: 131072 | Sampling Period: 655.36s -*/

/* OSR: Over Sampling */
// Over-sampling setting for pressure measurement
#define OSR_P_x1	(0x0)	/*- No Over-sampling -*/
#define OSR_P_x2	(0x1)	/*- x2 Over-sampling -*/
#define OSR_P_x4	(0x2)	/*- x4 Over-sampling -*/
#define OSR_P_x8	(0x3)	/*- x8 Over-sampling -*/
#define OSR_P_x16	(0x4)	/*- x16 Over-sampling -*/
#define OSR_P_x32	(0x5)	/*- x32 Over-sampling -*/

// Over-sampling setting for temperature measurement
#define OSR_T_x1	(0x0)	/*- No Over-sampling -*/
#define OSR_T_x2	(0x1)	/*- x2 Over-sampling -*/
#define OSR_T_x4	(0x2)	/*- x4 Over-sampling -*/
#define OSR_T_x8	(0x3)	/*- x8 Over-sampling -*/
#define OSR_T_x16	(0x4)	/*- x16 Over-sampling -*/
#define OSR_T_x32	(0x5)	/*- x32 Over-sampling -*/


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
void BMP_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t osr_p, uint8_t osr_t, uint8_t odr, uint8_t iir_coeff);

// DeInit
void BMP_DeInit(I2C_HandleTypeDef *pI2CHandle);

// Get Sensor Values
float BMP_getTemperature_C(I2C_HandleTypeDef *pI2CHandle);
float BMP_getPressure_Pa(I2C_HandleTypeDef *pI2CHandle);
float BMP_getAltitude_m(I2C_HandleTypeDef *pI2CHandle, float seaLevel_hPa);
// Get Status of Data Ready bit
uint8_t BMP_INT_STATUS_drdy(I2C_HandleTypeDef *pI2CHandle);

#endif /* INC_BMP388_I2C_H_ */
