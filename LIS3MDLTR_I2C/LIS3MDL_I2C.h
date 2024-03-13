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
#define MAG_ADDRESS	(0x0C << 1)

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

/* LIS3DML APIs*/
// Initialization
void LIS3MDL_UserInit(I2C_HandleTypeDef *pI2CHandle);
void LIS3MDL_DefaultInit(I2C_HandleTypeDef *pI2CHandle);
// DeInit
void LIS3MDL_DeInit(I2C_HandleTypeDef *pI2CHandle);

// Get Sensor Values
int16_t LIS3MDL_getTemperature_C(I2C_HandleTypeDef *pI2CHandle);

void LIS3MDL_getMAG_XYZ(I2C_HandleTypeDef *pI2CHandle, int16_t *pMagXYZ); // check argument
int16_t LIS3MDL_getMAG_X(I2C_HandleTypeDef *pI2CHandle);
int16_t LIS3MDL_getMAG_Y(I2C_HandleTypeDef *pI2CHandle);
int16_t LIS3MDL_getMAG_Z(I2C_HandleTypeDef *pI2CHandle);




#endif /* INC_LIS3MDL_I2C_H_ */
