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

/* Servo Control */
/*- Calculate SERVO_MIN and SERVO_MAX for your servos, refer to README for calculations -*/
#define SERVO_MAX	        (491)	/*- Set carefully | puts servo at 180 Degrees -*/
#define SERVO_MIN	        (82)	/*- Set carefully | puts servo at 0 degrees -*/
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

/* PCA9685 PWM Output Ports */
#define PCA_PORT0	(0x0)
#define PCA_PORT1	(0x1)
#define PCA_PORT2	(0x2)
#define PCA_PORT3	(0x3)
#define PCA_PORT4	(0x4)
#define PCA_PORT5	(0x5)
#define PCA_PORT6	(0x6)
#define PCA_PORT7	(0x7)
#define PCA_PORT8	(0x8)
#define PCA_PORT9	(0x9)
#define PCA_PORT10	(0xA)
#define PCA_PORT11	(0xB)
#define PCA_PORT12	(0xC)
#define PCA_PORT13	(0xD)
#define PCA_PORT14	(0xE)
#define PCA_PORT15	(0xF)

/* PCA9685 APIs*/

// Initialization
void PCA_Init(I2C_HandleTypeDef *pI2CHandle, uint16_t pwmFrequency);

// Control
void PCA_sleep(I2C_HandleTypeDef *pI2CHandle);
void PCA_wakeUp(I2C_HandleTypeDef *pI2CHandle);
void PCA_setPWM(I2C_HandleTypeDef *pI2CHandle, uint8_t port, uint16_t ONcount, uint16_t OFFcount);
void PCA_setPWMtoAll(I2C_HandleTypeDef *pI2CHandle, uint16_t ONcount, uint16_t OFFcount);
void PCA_setServoAngle(I2C_HandleTypeDef *pI2CHandle, uint8_t port, uint8_t angle);


#endif /* INC_PCA9685_I2C_H_ */
