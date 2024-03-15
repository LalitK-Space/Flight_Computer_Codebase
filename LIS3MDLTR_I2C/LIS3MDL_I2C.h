/*
 * 									LIS3MDL_I2C.h
 *
 * This file contains all the LIS3MDL related APIs supported by the driver.
 *
 */

#ifndef INC_LIS3MDL_I2C_H_
#define INC_LIS3MDL_I2C_H_

/* MCU Specific Header file */
#include "stm32f4xx_hal.h"
#include <stdint.h>
/* LIS3DML I2C Address */
#define MAG_ADDRESS	(0x1C << 1)

/* LIS3DML Registers Addresses */
#define WHO_AM_I	0x0F
#define CTRL_REG_1	0x20
#define CTRL_REG_2	0x21
#define CTRL_REG_3	0x22
#define CTRL_REG_4	0x23
#define CTRL_REG_5	0x24
#define STATUS_REG	0x27
#define OUT_X_L    0x28
#define OUT_X_H    0x29
#define OUT_Y_L    0x2A
#define OUT_Y_H    0x2B
#define OUT_Z_L    0x2C
#define OUT_Z_H    0x2D
#define TEMP_OUT_L	0x2E
#define TEMP_OUT_H	0x2F
#define INT_CFG    0x30
#define INT_SRC    0x31
#define INT_THS_L	0x32
#define INT_THS_H	0x33

/* LIS3DML Output Data Rate */
#define MAG_ODR_0_625HZ    (0)
#define MAG_ODR_1_25HZ    (1)
#define MAG_ODR_2_5HZ    (2)
#define MAG_ODR_5HZ    (3)
#define MAG_ODR_10HZ    (4)
#define MAG_ODR_20HZ    (5)
#define MAG_ODR_40HZ    (6)
#define MAG_ODR_80HZ    (7)

/* LIS3DML X, Y and Z Axis Operation Modes */
#define MAG_OM_LOW_POWER    (0)
#define MAG_OM_MEDIUM_POWER    (1)
#define MAG_OM_HIGH_POWER    (2)
#define MAG_OM_ULTRA_HIGH_POWER    (3)

/* LIS3DML Full Scale Selection */
#define MAG_FS_4G    (0)
#define MAG_FS_8G    (1)
#define MAG_FS_12G    (2)
#define MAG_FS_16G    (3)

/* LIS3DML System Operational Mode */
#define MAG_CONTINUOUS_CONV    (0)
#define MAG_SINGLE_CONV    (1)
#define MAG_POWER_DOWN    (2)

/* LIS3DML Temperature sensor Enable/Disable */
#define MAG_TEMP_EN    (1)
#define MAG_TEMP_DI    (0)


/* LIS3DML APIs*/
// Initialization
void LIS3MDL_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t temp_EnorDi, int8_t ODR, int8_t xyzOM, int8_t FScale, int8_t measurementMode);
void LIS3MDL_DefaultInit(I2C_HandleTypeDef *pI2CHandle);
// DeInit
void LIS3MDL_DeInit(I2C_HandleTypeDef *pI2CHandle);

// Get Sensor Values
float LIS3MDL_getTemperature_C(I2C_HandleTypeDef *pI2CHandle);

void LIS3MDL_getMAG_XYZ(I2C_HandleTypeDef *pI2CHandle, int16_t *pMagXYZ); // check argument
int16_t LIS3MDL_getMAG_X(I2C_HandleTypeDef *pI2CHandle);
int16_t LIS3MDL_getMAG_Y(I2C_HandleTypeDef *pI2CHandle);
int16_t LIS3MDL_getMAG_Z(I2C_HandleTypeDef *pI2CHandle);




#endif /* INC_LIS3MDL_I2C_H_ */
