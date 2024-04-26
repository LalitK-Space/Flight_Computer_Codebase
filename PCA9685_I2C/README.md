# PWM Driver [PCA9685]

> This repository contains drivers for PCA9685 (PWM Driver)

> PCA9685 uses I2C communication only, and to keep naming similar to other drivers, file names "PCA9685_I2C.h" and "PCA9685_I2C.c "are used.

# Files
**`PCA9685_I2C.h`**
<p>Header file containing supported functions for sensor configuration and data retrieval. 

**`PCA9685_I2C.c`**
<p> Implementation of supported functions.

# How to use
**Device Address:**
<p>- If the sensors' addresses differ from the default, update it in the header file.

`[Address Macro : PCA9685_ADDRESS]` 

**Initialization:**
- ` PCA_Init(...)`

*PCA_Init(...)* 
<p>This function initializes the PWM Driver with the user-defined PWM frequency and the following configurations:

-   Output drive mode is Totem Pole
-   Device's internal oscillator of 25MHz is used to calculate Pre-Scale value

**Control:**
- ` PCA_sleep(...)`
<p> Put PCA9685 into sleep mode

- ` PCA_setPWM(...)`
<p> Set PWM output to the user's desired port

- ` PCA_setPWMtoAll(...)`
<p> Set PWM output to all the ports

- ` PCA_setServoAngle(...)`
<p> Set servo angle


# Servo Control
**Using PCA_setServoAngle() function**

`This function uses values SERVO_MIN and SERVO_MAX to calculate the minimum and maximum PWM pulse length/count needed to put the servo at 0 and 180 degrees.` 
- Given that these values can differ between servos, assigning them properly is essential. This ensures the servo operates within its safe range, avoiding any potential stress or damage.

- To calculate the SERVO_MIN and SERVO_MAX for your servos, use `PCA_setPWM(&hi2cx, PCA_PORTx, 0, OFFcount)`, *OFFcount* ranges from 0 to 4096. By using different values of OFFcount, estimate the 0 and 180 degrees of your servo and update the SERVO_MIN and SERVO_MAX accordingly.


```c
/* --- TIP --- */
/* As a starting point, assign *OFFcount* the same as SERVO_MIN or SERVO_MAX to get an idea of how far your servo is from 0 degrees or 180 degrees and proceed from there. */

PCA_setPWM(&hi2cx, PCA_PORTx, 0, SERVO_MIN) /* Should put your servo at 0 Degrees*/

PCA_setPWM(&hi2cx, PCA_PORTx, 0, SERVO_MAX) /* Should put your servo at 180 Degrees*/

/* - Be careful not to put stress on your servos */
/* - Let's say a value of SERVO_MIN as 100 puts your servo at 0 degrees; you must refrain from putting further stress on your servo by writing 99 or below. Same for SERVO_MAX. */

```
