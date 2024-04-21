# Power Monitor [INA219]

> This repository contains drivers for INA219 (Power Monitor) for I2C communication.

# Files
**`INA219_I2C.h`**
<p>Header file containing supported functions for sensor configuration and data retrieval. 

**`INA219_I2C.c`**
<p> Implementation of supported functions.

**Device Address:**
<p>- If the sensors' addresses differ from the default, update it in the header file.

`[Address Macro : INA219_ADDRESS]`

**Initialization and Device Configurations:**

- `INA219_setCalibration_32V_2A(...)`
- `INA219_setCalibration_32V_1A(...)`
- `INA219_setCalibration_16V_400mA(...)`

*INA219_setCalibration_32V_2A(...)* 
<p>This function calibrates the Current and Power Registers to measure currents up to 2A and voltages up to 32V with the following configurations:

- Operating mode: Shunt and Bus, Continuous.
- PGA gain is /8 and with range of ±320 mV
- Shunt ADC resolution of 12Bit (Conversion Time = 532us)
- Bus ADC resolution of 12Bit (Conversion Time = 532us)

*INA219_setCalibration_32V_1A(...)* 
<p>This function calibrates the Current and Power Registers to measure currents up to 1A and voltages up to 32V with the following configurations:

- Operating mode: Shunt and Bus, Continuous.
- PGA gain is /8 and with range of ±320 mV
- Shunt ADC resolution of 12Bit (Conversion Time = 532us)
- Bus ADC resolution of 12Bit (Conversion Time = 532us)

*INA219_setCalibration_16V_400mA(...)* 
<p>This function calibrates the sensor for the highest precision for current measurement with up to 400mA and voltages up to 16V with the following configurations:

- Operating mode: Shunt and Bus, Continuous.
- PGA gain is 1 and with range of ±40 mV
- Shunt ADC resolution of 12Bit (Conversion Time = 532us)
- Bus ADC resolution of 12Bit (Conversion Time = 532us)

**Deinitialization:**
- `INA219_DeInit(...)`
<p>All user registers are overwritten with their default values

**Data Retrieval:**

>Note: The drivers use a 0.1Ω shunt resistor for the calculations.
- `INA219_getShuntVoltage_mV(...)`: Returns shunt voltage in milli-Volts (mV)
- `INA219_getBusVoltage_V(...)`: Returns bus voltage in Volts (V) 
- `INA219_getPower_mW(...)`: Returns power in milli-Watts (mW)
- `INA219_getcurrent_mA(...)`: Returns current in milli-Amperes (mA)

