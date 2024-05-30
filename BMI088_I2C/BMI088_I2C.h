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
#define ACC_ADDRESS	   (0x18 << 1)
/* Gyroscope I2C Address */
#define GYRO_ADDRESS	 (0x68 << 1)

/* Accelerometer Registers Addresses */
#define ACC_CHIP_ID    (0x00)
#define ACC_ERR_REG    (0x02)
#define ACC_STATUS     (0x03)
#define ACC_X_LSB      (0x12)
#define ACC_X_MSB      (0x13)
#define ACC_Y_LSB      (0x14)
#define ACC_Y_MSB      (0x15)
#define ACC_Z_LSB      (0x16)
#define ACC_Z_MSB      (0x17)
#define SENSORTIME_0   (0x18)
#define SENSORTIME_1   (0x19)
#define SENSORTIME_2   (0x1A)
#define TEMP_MSB       (0x22)
#define TEMP_LSB       (0x23)
#define ACC_CONF       (0x40)
#define ACC_RANGE      (0x41)
#define INT1_IO_CTRL   (0x53)
#define INT2_IO_CTRL	 (0x54)
#define INT_MAP_DATA	 (0x58)
#define ACC_SELF_TEST	 (0x6D)
#define ACC_PWR_CONF	 (0x7C)
#define ACC_PWR_CTRL	 (0x7D)
#define ACC_SOFTRESET	 (0x7E)

/* Accelerometer Output Data Rate */
#define ACC_ODR_12_5HZ	(1)
#define ACC_ODR_25HZ	  (2)
#define ACC_ODR_50HZ	  (3)
#define ACC_ODR_100HZ	  (4)
#define ACC_ODR_200HZ	  (5)
#define ACC_ODR_400HZ	  (6)
#define ACC_ODR_800HZ	  (7)
#define ACC_ODR_1600HZ	(8)

/* Accelerometer Range */
#define ACC_RANGE_3G	  (1)
#define ACC_RANGE_6G	  (2)
#define ACC_RANGE_12G	  (3)
#define ACC_RANGE_24G	  (4)

/* Accelerometer Low Pass Filter Bandwidth */
#define ACC_BWP_4FOLD	  (1)
#define ACC_BWP_2FOLD	  (2)
#define ACC_BWP_NORMAL	(3)

/* Gyroscope Registers Addresses */
#define GYRO_CHIP_ID              (0x00)
#define RATE_X_LSB                (0x02)
#define RATE_X_MSB                (0x03)
#define RATE_Y_LSB                (0x04)
#define RATE_Y_MSB                (0x05)
#define RATE_Z_LSB                (0x06)
#define RATE_Z_MSB                (0x07)
#define GYRO_INT_STAT_1           (0x0A)
#define GYRO_RANGE                (0x0F)
#define GYRO_BANDWIDTH            (0x10)
#define GYRO_LPM1                 (0x11)
#define GYRO_SOFTRESET            (0x14)
#define GYRO_INT_CTRL             (0x15)
#define GYRO_INT3_INT4_IO_CONF    (0x16)
#define GYRO_INT3_INT4_IO_MAP     (0x18)
#define GYRO_SELF_TEST            (0x3C)

/* Gyroscope Output Data Rate (Hz) and Filter Bandwidth (Hz) */
#define GYRO_ODR2000_BW532	      (0x00)
#define GYRO_ODR2000_BW230	      (0x01)
#define GYRO_ODR1000_BW116	      (0x02)
#define GYRO_ODR400_BW47	        (0x03)
#define GYRO_ODR200_BW23	        (0x04)
#define GYRO_ODR100_BW12	        (0x05)
#define GYRO_ODR200_BW64	        (0x06)
#define GYRO_ODR100_BW32	        (0x07)

/* Gyroscope Range */
#define GYRO_FS_2000	(0x00)	/*- FS = ±2000°/s | Resolution = 16.384 LSB/°/s -*/
#define GYRO_FS_1000	(0x01)	/*- FS = ±1000°/s | Resolution = 32.768 LSB/°/s -*/
#define GYRO_FS_500		(0x02) 	/*- FS = ±500°/s | Resolution = 65.546 LSB/°/s -*/
#define GYRO_FS_250		(0x03)	/*- FS = ±250°/s | Resolution = 131.072 LSB/°/s -*/
#define GYRO_FS_125		(0x04)	/*- FS = ±125°/s | Resolution = 262.144 LSB/°/s -*/

/* BMI088 APIs*/
// Initialization
void BMI_Acc_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t acc_bwp, uint8_t acc_odr, uint8_t acc_range);
void BMI_Acc_DefaultInit(I2C_HandleTypeDef *pI2CHandle);

void BMI_Gyro_UserInit(I2C_HandleTypeDef *pI2CHandle, uint8_t gyro_odr_bw, uint8_t gyro_range);
void BMI_Gyro_DefaultInit(I2C_HandleTypeDef *pI2CHandle);

// DeInit
void BMI_Acc_DeInit(I2C_HandleTypeDef *pI2CHandle);
void BMI_Gyro_DeInit(I2C_HandleTypeDef *pI2CHandle);

// Get Sensor Values
float BMI_getTemperature_C(I2C_HandleTypeDef *pI2CHandle);

float BMI_getAcc_X(I2C_HandleTypeDef *pI2CHandle);
float BMI_getAcc_Y(I2C_HandleTypeDef *pI2CHandle);
float BMI_getAcc_Z(I2C_HandleTypeDef *pI2CHandle);

float BMI_getGyro_X(I2C_HandleTypeDef *pI2CHandle);
float BMI_getGyro_Y(I2C_HandleTypeDef *pI2CHandle);
float BMI_getGyro_Z(I2C_HandleTypeDef *pI2CHandle);


#endif /* INC_BMI088_I2C_H_ */
