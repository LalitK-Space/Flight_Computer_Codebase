/*
 * 									PCA9685_I2C.c
 *
 * This file contains all the PCA9685 related APIs supported by the driver.
 *
 */

#include "PCA9685_I2C.h"


/*- Helper Function:  -*/
static uint16_t map_AngletToPWM(uint8_t angle, uint8_t servoAngle_min, uint8_t servoAngle_max, uint16_t servo_min, uint16_t servo_max);


/* ------------------------------------------------------------------------------------------------------
 * Name		:	PCA_Init
 * Description	:	Initialize PCA9685 with user defined PWM frequency
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	PWM Frequency | Maximum = 1526Hz and Minimum = 24Hz
 * Return Type	:	none (void)
 * Note		:
 * 			- Internal Oscillator of 25MHz is used
 * 			- Output Drive Mode is Totem Pole
 * ------------------------------------------------------------------------------------------------------ */
void PCA_Init(I2C_HandleTypeDef *pI2CHandle, uint16_t pwmFrequency)
{
	/*- Calculate Pre-scaler for PWM output frequency -*/
	if (pwmFrequency < 24)
	{
		/*- minimum PWM frequency is 24Hz with PCA_OSCILLATOR_FREQ of 25000000Hz | PCA_PRESCALE_MIN -*/
		pwmFrequency = 24;
	}
	else if (pwmFrequency > 1526)
	{
		/*- maximum PWM frequency is 1526Hz with PCA_OSCILLATOR_FREQ of 25000000Hz | PCA_PRESCALE_MAX -*/
		pwmFrequency = 1526;
	}

	uint8_t preScaler = ((PCA_OSCILLATOR_FREQ / (pwmFrequency * 4096.0)) + 0.5) - 1;

	uint8_t data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Make sure, device is in sleep Mode before writing pre-scaler value -*/
	data |= (1<<4);
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Write Pre-scaler value -*/
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_PRE_SCALE, I2C_MEMADD_SIZE_8BIT, &preScaler, 1, HAL_MAX_DELAY);
	/*- wait for oscillator to stabilize (500us), used 1ms -*/
	HAL_Delay(1);

	/*- Enable Normal Mode -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Clear Sleep bit for Normal Mode -*/
	data &= ~(1<<4);
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	/*- Enable Auto-Increment -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Enable RESTART -*/
	data |= (1<<7);
	/*- Enable Auto-Increment -*/
	data |= (1<<5);
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(10);

	/*- Set Output Drive: Totem Pole -*/
	data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, PCA9685_ADDRESS, PCA_MODE2, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	data |= (1<<2);
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_MODE2, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

}



/* ------------------------------------------------------------------------------------------------------
 * Name		:	PCA_sleep
 * Description	:	Put PCA9685 to sleep
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void PCA_sleep(I2C_HandleTypeDef *pI2CHandle)
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- Set: Sleep Bit -*/
	data |= (1<<4);
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	/*- wait for oscillator to stabilize (500us), using 1ms -*/
	HAL_Delay(1);
}



/* ------------------------------------------------------------------------------------------------------
 * Name		:	PCA_wakeUp
 * Description	:	Wake up from sleep
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void PCA_wakeUp(I2C_HandleTypeDef *pI2CHandle)
{

    uint8_t data = 0;
    HAL_I2C_Mem_Read(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    /*- Clear: Sleep Bit -*/
    data &= ~(1<<4);
    HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_MODE1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    /*- Wait for oscillator to stabilize (500us), using 1ms -*/
    HAL_Delay(1);

}



/* ------------------------------------------------------------------------------------------------------
 * Name		:	PCA_setPWM
 * Description	:	Set PWM output to the user's desired port.
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	PWM Output Port [0:15]
 * Parameter 3	:	ON Time/Count [0-4096]
 * Parameter 4	:	OFF Time/Count [0-4096]
 * Return Type	:	none (void)
 * Note		:
 * 			- Possible Arguments: port = PCA_PORTx
 * 			- The range of ONcount and OFFcount is [0-4096]
 * ------------------------------------------------------------------------------------------------------ */
void PCA_setPWM(I2C_HandleTypeDef *pI2CHandle, uint8_t port, uint16_t ONcount, uint16_t OFFcount)
{
	uint8_t pwmData[4];
	uint8_t portAddress = 0;
	pwmData[0] = ONcount;
	pwmData[1] = (ONcount>>8);
	pwmData[2] = OFFcount;
	pwmData[3] = (OFFcount>>8);

	/*- Calculating address of out PWM port -*/
	portAddress = PCA_LED0_ON_L + 4 * port;
	/*- Write to desired port -*/
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, portAddress, I2C_MEMADD_SIZE_8BIT, pwmData, 4, HAL_MAX_DELAY);
}



/* ------------------------------------------------------------------------------------------------------
 * Name		:	PCA_setPWMtoAll
 * Description	:	Set PWM output to the ports.
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	ON Time/Count [0-4096]
 * Parameter 3	:	OFF Time/Count [0-4096]
 * Return Type	:	none (void)
 * Note		:
 * 			- The range of ONcount and OFFcount is [0-4096]
 * ------------------------------------------------------------------------------------------------------ */
void PCA_setPWMtoAll(I2C_HandleTypeDef *pI2CHandle, uint16_t ONcount, uint16_t OFFcount)
{
	uint8_t pwmData[4];
	pwmData[0] = ONcount;
	pwmData[1] = (ONcount>>8);
	pwmData[2] = OFFcount;
	pwmData[3] = (OFFcount>>8);

	/*- Write to All ports -*/
	HAL_I2C_Mem_Write(pI2CHandle, PCA9685_ADDRESS, PCA_ALL_LED_ON_L, I2C_MEMADD_SIZE_8BIT, pwmData, 4, HAL_MAX_DELAY);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	PCA_setServoAngle
 * Description	:	Set servo angle
 * Parameter 1	:	Pointer to I2C Handle
 * Parameter 2	:	PWM Output Port [0:15]
 * Parameter 3	:	Angle [0:180]
 * Return Type	:	none (void)
 * Note		:
 * 			-
 * ------------------------------------------------------------------------------------------------------ */
void PCA_setServoAngle(I2C_HandleTypeDef *pI2CHandle, uint8_t port, uint8_t angle)
{
	/*- Calculate PWM OFFcount based on angle -*/
	uint16_t pwmOffcount = map_OffcountToAngle(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_MIN, SERVO_MAX);

	PCA_setPWM(pI2CHandle, port, 0, pwmOffcount);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	map_OffcountToAngle
 * Description	:	Static function to map PWM pulse width (OFFcount) to servo angle
 * Parameter 1	:	Angle [between SERVO_ANGLE_MIN and SERVO_ANGLE_MAX]
 * Parameter 2	:	Minimum Servo Angle [SERVO_ANGLE_MIN]
 * Parameter 3	:	Maximum Servo Angle [SERVO_ANGLE_MAX]
 * Parameter 2	:	Minimum value of OFFcount to put servo at angle 0 degrees [SERVO_MIN]
 * Parameter 3	:	Maximum value of OFFcount to put servo at angle 180 degrees [SERVO_MAX]
 * Return Type	:	PWM pulse count (OFFcount) (void)
 * Note		:
 * 			-	similar to Arduino's map() function
 * ------------------------------------------------------------------------------------------------------ */
static uint16_t map_AngletToPWM(uint8_t angle, uint8_t servoAngle_min, uint8_t servoAngle_max, uint16_t servo_min, uint16_t servo_max)
{
	/*- Ensuring values within range -*/
    if (angle < servoAngle_min)
    {
        angle = servoAngle_min;
    }
    else if (angle > servoAngle_max)
    {
        angle = servoAngle_max;
    }

	/*- Scaling -*/
    return (uint16_t) (angle - servoAngle_min) * (servo_max - servo_min) / (servoAngle_max - servoAngle_min) + servo_min;

}
