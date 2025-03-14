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

`This function calculates the correct PWM pulse width using SERVO_MIN and SERVO_MAX, ensuring the servo moves accurately between 0° and 180°.` 

Since servo PWM timing varies across different models, properly configuring SERVO_MIN and SERVO_MAX is critical to prevent servo over-travel, excessive current draw, and unwanted mechanical stress.

- To calculate the SERVO_MIN and SERVO_MAX for your servos, use `PCA_setPWM(&hi2cx, PCA_PORTx, 0, OFFcount)`, *OFFcount* ranges from 0 to 4096. By using different values of OFFcount, estimate the 0 and 180 degrees of your servo and update the SERVO_MIN and SERVO_MAX accordingly.

**Calculations to Determine SERVO_MIN and SERVO_MAX**

- `Step 1: Find Pulse Width Requirements for Your Servo.`

    Check the datasheet for your servo to determine the required pulse width range. Typical servos operate between 500µs - 2500µs, but some use 400µs - 2400µs.
- `Step 2: Understanding PCA9685 PWM Resolution`
    * PCA9685 generates a 12-bit PWM signal with 4096 steps per cycle.
    * At 50Hz (20ms period), each step is (20ms / 4096) = 4.88µs
    * Use `PCA_Init(&hi2cx, 50)` to set 50Hz frequency.
    * To convert pulse width (in milliseconds) to a PWM value:<p>`PWM_value = (Pulse Width (ms) / period (ms)) * 4096`
- `Step 3: Example Calculations`
    For a servo requiring 540µs - 2400µs pulse width:
    * Convert pulse width to milliseconds. [if the pulse width is in µs (microseconds), convert it to ms (milliseconds)]
    * 0° Position **(SERVO_MIN)**<p>`PWM_value = (0.54 (ms) / 20 (ms)) * 4096 = ~112`
    * Convert pulse width to milliseconds. [if the pulse width is in µs (microseconds), convert it to ms (milliseconds)]
    * 180° Position **(SERVO_MAX)**<p>`PWM_value = (2.4 (ms) / 20 (ms)) * 4096 = ~492`
    


```c
/* --- TIP --- */
/* For servos with a Pulse Cycle of 20ms and a Pulse Width range of ~540µs-2.4ms, the above SERVO_MIN and SERVO_MAX are correct.

If you don’t know your servo’s exact pulse width range, use the PCA_setPWM() function to manually adjust it.

as a starting point, assign *OFFcount* the same as SERVO_MIN or SERVO_MAX to get an idea of how far your servo is from 0 degrees or 180 degrees and proceed from there. */

PCA_setPWM(&hi2cx, PCA_PORTx, 0, SERVO_MIN) /* Should put your servo at 0 Degrees*/

PCA_setPWM(&hi2cx, PCA_PORTx, 0, SERVO_MAX) /* Should put your servo at 180 Degrees*/

/* - Be careful not to put stress on your servos */
/* - Let's say a value of SERVO_MIN as 112 puts your servo at 0 degrees; you must refrain from putting further stress on your servo by writing lower value. Same for SERVO_MAX. */

```
