# EEPROM [AT24Cxxx]

> This repository contains drivers for AT24Cxxx (EEPROM)

> AT24Cxxx uses I2C communication only, and to keep naming similar to other drivers, file names "AT24Cxxx_I2C.h" and "AT24Cxxx_I2C.c" are used.

# Files
**`AT24Cxxx_I2C.h`**
<p>Header file containing supported functions for sensor configuration and data retrieval. 

**`AT24Cxxx_I2C.c`**
<p> Implementation of supported functions.

# How to use
**Device Address:**
<p>- If the device's addresses differ from the default, update it in the header file.

`[Address Macro : EEPROM_x_ADDRESS]` 

**Reading and Writing:**
- `EEPROM_Writebyte(...)`: Writes 1 byte of data to a given memory address 
- `EEPROM_Readbyte(...)` : Reads 1 byte of data from a given memory address
- `EEPROM_Write(...)`    : Writes data to a given memory address
- `EEPROM_Read(...)`     : Reads data from a given memory address

**Example**
```c
/*- Write 1 byte of data at address 0x0000 -*/
uint8_t writeData = 'L';
EEPROM_Writebyte(&hi2cx, 0x0000, writeData);

/*- Read 1 byte of data from address 0x0000 -*/
uint8_t readData = 0;
readData = EEPROM_Readbyte(&hi2cx, 0x0000);

/*- Writing a data buffer to an address starting from 0x0005 -*/
uint8_t telemetry[50];
sprintf((char *)telemetry,"LalitK.space");
uint16_t length = strlen((char*)telemetry);
EEPROM_Write(&hi2cx, 0x0005, telemetry, length);

/*- Reading a data buffer from address 0x0005 -*/
uint8_t readTelemetry[50];
EEPROM_Read(&hi2cx, 0x0005, readTelemetry, length);
```