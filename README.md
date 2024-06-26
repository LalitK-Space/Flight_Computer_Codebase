# Flight_Computer_Codebase

> This repository contains the drivers for onboard sensors and modules of Flight Computer that I am developing.

[Take a look! - A New Capable Flight Computer](https://www.youtube.com/watch?v=mmJXlWljd3Q)

![Flight Computer](/Images/FlightComputer.jpg)

## Onboard Sensors and Modules
### Microcontrollers:
The board has two STM32F4-based microcontrollers, one called the NAV (navigation) computer and the other serving as the primary Flight Computer (FC).
### Onboard Sensors:
**`NAV's sensors and modules:`**
- [Inertial Measurement Unit (BMI088)](https://github.com/LalitK-Space/Flight_Computer_Codebase/tree/main/BMI088_I2C)	    
- [Digital Pressure Sensor (BMP388)](https://github.com/LalitK-Space/Flight_Computer_Codebase/tree/main/BMP388_I2C)      
- [Magnetometer (LIS3MDLTR)](https://github.com/LalitK-Space/Flight_Computer_Codebase/tree/main/LIS3MDLTR_I2C)	
- [EEPROM (AT24C256C)](https://github.com/LalitK-Space/Flight_Computer_Codebase/tree/main/AT24Cxxx_I2C)				

**`FC's sensors and modules:`**
- [Power Monitor (INA219)](https://github.com/LalitK-Space/Flight_Computer_Codebase/tree/main/INA219_I2C)		
- [PWM Driver (PCA9685)](https://github.com/LalitK-Space/Flight_Computer_Codebase/tree/main/PCA9685_I2C)
- FLASH (W25N01GV)				
- [EEPROM (AT24C256C)](https://github.com/LalitK-Space/Flight_Computer_Codebase/tree/main/AT24Cxxx_I2C)					

## Drivers Information
<p> This repository contains drivers for the sensors and modules listed, developed using  STM32 HAL (Hardware Abstraction Layer) and low-layer drivers.
<p> All sensors are interfaced using I2C communication, except for FLASH, which utilizes SPI.
<p> The respective folder for each sensor and module provides detailed descriptions and documentation.


