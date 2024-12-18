# Inertial Measurement Unit [BMI088]

> This repository contains drivers for BMI088 (6-Axis Inertial Measurement Unit) for I2C communication.

# Files
**`BMI088_I2C.h`**
<p>Header file containing supported functions for sensor configuration and data retrieval. 

**`BMI088_I2C.c`**
<p> Implementation of supported functions.

# How to use
**Device Address:**
<p>- If the sensors' addresses differ from the default, update it in the header file.

`[Accelerometer Address : ACC_ADDRESS]`  
`[Gyroscope Address     : GYRO_ADDRESS]`  

**Initialization:**
- `BMI_Acc_DefaultInit(...)`
- `BMI_Acc_UserInit(...)`
- `BMI_Gyro_DefaultInit(...)`
- `BMI_Gyro_UserInit(...)`

*BMI_Acc_DefaultInit(...)* 
<p>This function initializes the accelerometer with the following configurations:

- Accelerometer range is ±6g
- Accelerometer ODR is 100Hz
- Accelerometer bandwidth of low pass filter is normal

*BMI_Acc_UserInit(...)* 
<p>This function initializes the accelerometer with the user-defined configurations.
Users can configure the following:

- Filter Settings: Bandwidth of the Low Pass Filter
- Output Data Rate
- Measurement Range

>BMI088_I2C.h contains the possible arguements for each user configuration.

NOTE: This User Initialization *`BMI_Acc_UserInit(&hi2cx, ACC_BWP_NORMAL, ACC_ODR_100HZ, ACC_RANGE_6G);`* is same as *`BMI_Acc_DefaultInit(&hi2cx);`*


*BMI_Gyro_DefaultInit(...)* 
<p>This function initializes the accelerometer with the following configurations:

- Gyroscope range: Full Scale = ±2000 °/s and Resolution = 16.384 LSB/°/s
- Gyroscope Bandwidth: ODR = 2000Hz and Filter Bandwidth = 532Hz
- Gyroscope Power Mode: Normal Mode

*BMI_Gyro_UserInit(...)* 
<p>This function initializes the accelerometer with the user-defined configurations.
Users can configure the following:

- Output Data Rate and Filter Bandwidth
- Angular Rate Range and Resolution

>BMI088_I2C.h contains the possible arguements for each user configuration.

NOTEL: This User Initialization *`BMI_Gyro_UserInit(&hi2cx, GYRO_ODR2000_BW532, GYRO_FS_2000);`* is same as *`BMI_Gyro_DefaultInit(&hi2cx);`*

**Deinitialization:**
- `BMI_Acc_DeInit(...)`
<p> All configuration settings are overwritten with their reset value.

- `BMI_Gyro_DeInit(...)`
<p> All configuration settings are overwritten with their reset value.

**Data Retrieval:**
- `BMI_getTemperature_C(...)`: Returns temperature in degrees Celsius. [Make sure the Accelerometer is enabled to get valid temperature readings]

- `BMI_getAcc_X(...)`: Returns acceleration along X-axis in m/s².

- `BMI_getAcc_Y(...)`: Returns acceleration along Y-axis in m/s².

- `BMI_getAcc_Z(...)`: Returns acceleration along Z-axis in m/s².

- `BMI_getGyro_X(...)`: Returns angular velocity along X-axis in degrees per second (°/s).

- `BMI_getGyro_Y(...)`: Returns angular velocity along Y-axis in degrees per second (°/s).

- `BMI_getGyro_Z(...)`: Returns angular velocity along Z-axis in degrees per second (°/s).

- `BMI_getAcc_DrdyStatus(...)`: Returns the acc_drdy bit status (1 if data is ready, 0 otherwise).

- `BMI_getGyro_DrdyStatus(...)`: Returns the gyro_drdy bit status (1 if data is ready, 0 otherwise).

**Interrupt Configuration and Mapping**
- `BMI_ACC_enable_INTx(...)`: Enables the Accelerometer interrupt and map the data ready interrupt to selected pin with the following configurations:
    - INTx pin behavior is configured to Push-Pull
    - INTx pin active state is configured to Active High
- `BMI_GYRO_enable_INTx(...)`: Enables the Gyroscope interrupt and map the data ready interrupt to selected pin with the following configurations:
    - INTx pin behavior is configured to Push-Pull
    - INTx pin active state is configured to Active High

**Example Usage - Interrupt Configuration and Data Ready Status**
```c
/* Initialize Accelerometer and Gyroscope */
  BMI_Acc_DefaultInit(&hi2cx);
  BMI_Gyro_DefaultInit(&hi2cx);

/* Interrupt Configuration and Mapping */
  BMI_ACC_enable_INTx(&hi2cx, ACC_INTx_OUT); /* INT1 and INT2 for accelerometer */
  BMI_GYRO_enable_INTx(&hi2cx, GYRO_INTx_OUT); /* INT3 and INT4 for gyroscope */

  /* In main loop or task */

 /* Data Ready Status for accelerometer */
 if (BMI_getAcc_DrdyStatus(&hi2cx))
 {
        /* Read accelerometer data */
 }

 /* Data Ready Status for gyroscope */
 if (BMI_getGyro_DrdyStatus(&hi2cx))
 {
        /* Read gyroscope data */
 }
```