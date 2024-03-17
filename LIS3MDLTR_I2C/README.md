# Magnetometer [LIS3MDL]

> This repository contains drivers for LIS3MDL (Digital output magnetic sensor) for I2C communication.

# Files
**`LIS3MDL_I2C.h`**
<p>Header file containing supported functions for sensor configuration and data retrieval. 

**`LIS3MDL_I2C.c`**
<p> Implementation of supported functions.

# How to use
**Device Address:**
<p>- If the sensor address differs from **0x1C**, update it in the header file. `MAG_ADDRESS`

**Initialization:**
- `LIS3MDL_DefaultInit(...)`
- `LIS3MDL_UserInit(...)`

*LIS3MDL_DefaultInit(...)* 
<p>This function initializes the magnetometer with the following configurations:

- Temperature sensor is enabled
- X, Y, and Z axis Operating mode is set for Ultra-high performance
- Output data rate is configured for 80 Hz
- FAST_ODR is disabled
- Self-test is disabled
- Full-scale selection is configured for ±4 Gauss
- System operating mode is configured for continuous-conversion
- Data LSB is at the lower address
- Fast read is disabled

*LIS3MDL_UserInit(...)* 
<p>This function initializes the magnetometer with the user-defined configurations.
Users can configure the following:

- Whether to enable Temperature sensor or not
- Output Data Rate
- X, Y, and Z axis Operating Mode
- Full-Scale Selection
- Measurement Modes

>LIS3MDL_I2C.h contains the possible arguements for each user configuration.

NOTEL: This User Initialization *`LIS3MDL_UserInit(&hi2cx, MAG_TEMP_EN, MAG_ODR_80HZ, MAG_OM_ULTRA_HIGH_POWER, MAG_FS_4G, MAG_M_CONTINUOUS_CONV);`* is same as *`LIS3MDL_DefaultInit(&hi2cx);`*

**Deinitialization:**
- `LIS3MDL_DeInit(...)`
<p> Resets registers (REG_1 to REG_5) to default states.

**Data Retrieval:**
- `LIS3MDL_getTemperature_C(...)`: Returns temperature in degrees Celsius. 

- `LIS3MDL_getMAG_X(...)`: Returns a 16-bit signed value representing the magnetic field along the X-axis.

- `LIS3MDL_getMAG_Y(...)`: Returns a 16-bit signed value representing the magnetic field along the Y-axis.

- `LIS3MDL_getMAG_Z(...)`: Returns a 16-bit signed value representing the magnetic field along the Z-axis.
