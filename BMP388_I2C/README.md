# Digital Pressure Sensor [BMP388]

> This repository contains drivers for BMP388 (Digital Pressure Sensor) for I2C communication.

# Files
**`BMP388_I2C.h`**
<p>Header file containing supported functions for sensor configuration and data retrieval. 

**`BMP388_I2C.c`**
<p> Implementation of supported functions.

# How to use

**Device Address:**
<p>- If the sensors' addresses differ from the default, update it in the header file.

`[Address Macro : BMP388_ADDRESS]`  
 
**Initialization:**
- `BMP_DefaultInit(...)`
- `BMP_UserInit(...)`

*BMP_DefaultInit(...)* 
<p>This function initializes the pressure sensor with the following configurations:

- Both pressure and temperature sensors are enabled
- Over-sampling for pressure measurement is x4
- Over-sampling for temperature measurement is x1 (No over-sampling)
- ODR is 200Hz, Pre-scaler is 1 and Sampling Period is 5ms
- IIR Filter is in Bypass mode (no filtering)

*BMP_UserInit(...)* 
<p>This function initializes the pressure sensor with the user-defined configurations.
Pressure and Temperature sensors are enabled and users can configure the following:

- Over-sampling setting for pressure measurement
- Over-sampling setting for temperature measurement
- Output Data Rate
- IIR Filter Coefficients

>BMP388_I2C.h contains the possible arguements for each user configuration.

NOTE: 
- This User Initialization *`BMP_UserInit(&hi2cx, OSR_P_x4, OSR_T_x1, ODR_200, IIR_COEFF_0);`* is same as *`BMP_DefaultInit(&hi2cx);`*
- In user initialization, follow the recommended settings to get valid results. Randomly picking the over-sampling and output data rate settings may automatically put the device in sleep mode.
- According to the datasheet,  with pressure oversampling settings of x1, x2, x4, and x8, use x1 temperature oversampling settings. With pressure oversampling of x16 and x32, use x2 temperature oversampling settings.
- The same applies to ODR and filter settings; recommended settings based on the use cases are provided in the datasheet. 

**Deinitialization:**
- `BMP_DeInit(...)`
<p> All configuration settings are overwritten with their default state.

**Data Retrieval:**
- `BMP_getTemperature_C(...)`: Returns temperature in degrees Celsius. 
- `BMP_getPressure_Pa(...)`: Returns pressure in Pascal (Pa). 
- `BMP_getAltitude_m(...)`: Returns altitude in meters (m)

Note:
-  Function BMP_getAltitude_m() expects the user to provide pressure at sea level in hPa. According to Wikipedia, the standard atmosphere is a unit of pressure defined as 101325 Pa (1013.25 hPa). 1013.25 can be provided as an expected argument to the function. 
- But, to get a more precise reading, pressure at sea level can be calculated using measured pressure and the known altitude values.
```c
/* Calculating pressure at sea level */
float pressure_hPa  = BMP_getPressure_Pa() / 100; /* /100: Pa to hPa */
float knownAltitude_m = x.x;                        /* Known elevation in m */

float pressure_seaLevel_hPa = pressure_hPa / pow((1.0 - (knownAltitude/44330.0)), 5.255);

/* Calculate it once and use the value in BMP_getAltitude_m() as the second expected argument to get more precise altitude readings. */
/* The equation is from BMP180 Datasheet */
```