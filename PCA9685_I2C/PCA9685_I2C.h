/*
 * 									PCA9685_I2C.h
 *
 * This file contains all the PCA9685 related APIs supported by the driver.
 *
 */

#ifndef INC_PCA9685_I2C_H_
#define INC_PCA9685_I2C_H_

/* MCU Specific Header file */
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* PCA9685 I2C Address */
#define PCA9685_ADDRESS	        (0x41 << 1)

/* PCA Configurations */
#define PCA_OSCILLATOR_FREQ	(25000000)	/* PCA Internal Oscillator Frequency */
#define PCA_PRESCALE_MIN	(3)
#define PCA_PRESCALE_MAX	(255)
#define PCA_MAX_PWM_FREQUENCY	(3052)	        /* frequency_max => 50MHz / (4096 * (PCA_PRESCALE_MIN + 1)) = ~3052 Hz */

/* Servo Control */
/*- Calculate SERVO_MIN and SERVO_MAX for your servos [Range: [0-4096] ON and OFF Time/Length] -*/
#define SERVO_MAX	        (0)
#define SERVO_MIN	        (0)
#define SERVO_ANGLE_MIN	        (0)
#define SERVO_ANGLE_MAX	        (180)

/* (some) PCA9685 Register Addresses */
#define PCA_MODE1	        (0x00)
#define PCA_MODE2	        (0x01)
#define PCA_LED0_ON_L	        (0x06)
#define PCA_LED0_ON_H	        (0x07)
#define PCA_LED0_OFF_L	        (0x08)
#define PCA_LED0_OFF_H	        (0x09)
// PCA_LEDx_ON_L = PCA_LED0_ON_L + 4 * (x)	x: 0 to 15
#define PCA_ALL_LED_ON_L	(0xFA)
#define PCA_ALL_LED_ON_H	(0xFB)
#define PCA_ALL_LED_OFF_L	(0xFC)
#define PCA_ALL_LED_OFF_H	(0xFD)
#define PCA_PRE_SCALE	        (0xFE)
#define PCA_TESTMODE	        (0xFF)

/* Output Configurations */
#define PCA_OUTDRV_OPENDRAIN	(0x0)
#define PCA_OUTDRV_TOTEMPOLE	(0x1)

/* PCA9685 APIs*/

// Initialization
void PCA_Init(I2C_HandleTypeDef *pI2CHandle, uint8_t pwmFrequency);

// DeInit
void PCA_DeInit(I2C_HandleTypeDef *pI2CHandle);

// Control
void PCA_sleep(I2C_HandleTypeDef *pI2CHandle);
void PCA_setPWM(I2C_HandleTypeDef *pI2CHandle, uint8_t channel, uint16_t ONlength, uint16_t OFFlength);
void PCA_setPWMtoAll(I2C_HandleTypeDef *pI2CHandle, uint16_t ONlength, uint16_t OFFlength);
void PCA_setServoAngle(I2C_HandleTypeDef *pI2CHandle, uint8_t angle);


#endif /* INC_PCA9685_I2C_H_ */
