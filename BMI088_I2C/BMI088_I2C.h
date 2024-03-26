/*
 * 									BMI088_I2C.h
 *
 * This file contains all the BMI088 related APIs supported by the driver.
 *
 */

#ifndef INC_BMI088_I2C_H_
#define INC_BMI088_I2C_H_

/* MCU Specific Header file */
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Accelerometer I2C Address */
#define ACCL_ADDRESS	(0x18 << 1)
/* Gyroscope I2C Address */
#define GYRO_ADDRESS	(0x68 << 1)





#endif /* INC_BMI088_I2C_H_ */
