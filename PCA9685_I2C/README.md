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

**Calculations to Determine SERVO_MIN and SERVO_MAX**

- Step 1: Refer to the datasheet to understand the pulse width requirements for your servo motor.
- Step 2: Calculate PWM Values:
    * PCA9685 PWM Resolution: The PCA9685 generates PWM signals with a 12-bit resolution, allowing for 4096 different PWM values (ranging from 0 to 4095) within one cycle.
    * Pulse Width to PWM Value: The complete cycle is 20 ms (corresponding to a frequency of 50 Hz,  `PCA_Init(&hi2cx, 50)`), and this cycle is mapped to 4096 counts.
- Step 3: Calculate the PWM Value for Minimum and Maximum Pulse Width:
    * Convert pulse width to milliseconds. [if the pulse width is in µs (microseconds), convert it to ms (milliseconds)]
    * Use the formula:<p>`PWM_value = (Pulse Width (ms) / Cycle Time (ms)) * 4096`
- Example: 
    * For a minimum pulse width of 400 µs: <p>`PWM_value (SERVO_MIN)  = (0.4 (ms) / 20 (ms)  ) * 4096 ≈ 82`
    * For a maximum pulse width of 2400 µs: <p>`PWM_value (SERVO_MAX)  = (2.4 (ms) / 20 (ms)  ) * 4096 ≈ 491`      



```c
/* --- TIP --- */
/* For servos with a Pulse Cycle of 20ms and a Pulse Width range of 400-2400µs, the above SERVO_MIN and SERVO_MAX are correct.
To manually calculate, if you do not know the pulse width requirements for your servo motor (WHY?? refer to datasheet), 
as a starting point, assign *OFFcount* the same as SERVO_MIN or SERVO_MAX to get an idea of how far your servo is from 0 degrees or 180 degrees and proceed from there. */

PCA_setPWM(&hi2cx, PCA_PORTx, 0, SERVO_MIN) /* Should put your servo at 0 Degrees*/

PCA_setPWM(&hi2cx, PCA_PORTx, 0, SERVO_MAX) /* Should put your servo at 180 Degrees*/

/* - Be careful not to put stress on your servos */
/* - Let's say a value of SERVO_MIN as 82 puts your servo at 0 degrees; you must refrain from putting further stress on your servo by writing 81 or below. Same for SERVO_MAX. */

```
