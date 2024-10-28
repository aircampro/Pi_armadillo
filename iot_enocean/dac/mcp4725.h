#ifndef __MCP4725_DAC_H
#define __MCP4725_DAC_H
/*

	ref:-  some of this driver was adapted from this pico library https://github.com/gavinlyonsrepo/MCP4725_PICO
		
*/

// Libraries
#include <stdio.h>                                               // optional for printf debug error messages
#include <cmath>                                                 // for pow() function
#include <stdint.h>

#include "i2c-dev-wrapper.h"                                     // include wrapper functions for i2c-dev

// Section Enums

/*! 8-bit i2c address. */
// For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
// For MCP4725A0 the address is 0x60 or 0x61
// For MCP4725A2 the address is 0x64 or 0x65
typedef enum : uint8_t
{
	MCP4725A0_Addr_A00 = 0x60, /**< MCP4725A0 with A0 = GND */
	MCP4725A0_Addr_A01 = 0x61, /**< MCP4725A0 with A0 = VCC */
	MCP4725A1_Addr_A00 = 0x62, /**< MCP4725A1 with A0 = GND */
	MCP4725A1_Addr_A01 = 0x63, /**< MCP4725A1 with A0 = VCC */
	MCP4725A2_Addr_A00 = 0x64, /**< MCP4725A2 with A0 = GND */
	MCP4725A2_Addr_A01 = 0x65  /**< MCP4725A2 with A0 = VCC */
}MCP4725_I2C_Addr_e;           // 8-bit i2c address

/*! DAC register, command bits C2C1C0 */
typedef enum : uint8_t
{
	MCP4725_FastMode     = 0x00, /**< Writes data to DAC register */
	MCP4725_RegisterMode = 0x40, /**< Writes data & config bits to DAC register */
	MCP4725_EEPROM_Mode  = 0x60  /**< Writes data & config bits to DAC register & EEPROM */
}MCP4725_CmdType_e;              // DAC register, command bits C2C1C0 */

/*! DAC register, power down bits PD1 PD0 , BSY,POR,xx,xx,xx,PD1,PD0,xx */
typedef enum : uint8_t
{
	MCP4725_PowerDown_Off     = 0x00, /**< Power down off draws 0.40mA no load & 0.29mA max load */
	MCP4725_PowerDown_1kOhm   = 0x01, /**< Power down on, with 1.0 kOhm to GND, draws ~60nA */
	MCP4725_PowerDown_100kOhm = 0x02, /**< Power down on, with 100 kOhm to GND */
	MCP4725_PowerDown_500kOhm = 0x03  /**< Power down on, with 500 kOhm to GND */
}MCP4725_PowerDownType_e;             // DAC register, power down bits PD1 PD0 , BSY,POR,xx,xx,xx,PD1,PD0,xx */

/*! DAC library read register type  */
typedef enum : uint8_t
{
	MCP4725_ReadSettings = 1,         /**< Read 1 byte,  Settings data  */
	MCP4725_ReadDACReg   = 3,         /**< Read 3 bytes, DAC register data */
	MCP4725_ReadEEPROM   = 5          /**< Read 5 bytes, EEPROM data */
}MCP4725_ReadType_e;                  // DAC library read register type 

/*! DAC general call command datasheet 7.3 */
typedef enum : uint8_t
{
	MCP4725_GeneralCallAddress = 0x00, /**< General call address */ 
	MCP4725_GeneralCallReset  =  0x06, /**< General call reset command */
	MCP4725_GeneralCallWakeUp =  0x09  /**< General call wake-up command */
}MCP4725_GeneralCallType_e;            // DAC general call command datasheet 7.3

// Section Definition's

// I2C interface Comms  related
#define MCP4725_I2C_DELAY            50000                         /**<  uS delay , I2C timeout */
#define MCP4725_ERROR                0xFFFF                        /**<  returns this value if I2C bus error from some methods */
#define MCP4725_EEPROM_WRITE_TIME    25                            /**<  mSec Memory write time, maximum 50 mSec */

// DAC voltage levels 
#define MCP4725_REFERENCE_VOLTAGE  3.3                             /**<  supply-reference Voltage in volts */
#define MCP4725_RESOLUTION         12                              /**<  resolution in bits , 12-bit */
#define MCP4725_STEPS              pow(2, (MCP4725_RESOLUTION))    /**<  quantity of DAC steps 2^12-bits = 4096 */
#define MCP4725_MAX_VALUE          ((MCP4725_STEPS) - 1)           /**<  Max value = 4096 -1 , 0 to 4095 */

// define the address we are using for our DAC on I2C
const U08 _dev_addr = static_cast<U08>(MCP4725_I2C_Addr_e::MCP4725A1_Addr_A01);  // define the DAC address in this example i'm using 0x63 (ADDR pin tied to VCC)
bool _serialDebug = false;

uint16_t readRegister(MCP4725_ReadType_e readType, U08 i2c_bus);
bool getEEPROMBusyFlag(U08 i2c_bus);
uint16_t getStoredInputCode(U08 i2c_bus);
uint16_t getDACInputCode(U08 i2c_bus);
uint16_t getStoredPowerType(U08 i2c_bus);
bool writeCommand2(uint16_t inputCode, MCP4725_CmdType_e mode, MCP4725_PowerDownType_e powerType, U08 i2c_bus);

uint16_t _bitsPerVolt = 0;
bool _safetyCheck;
double _refVoltage = MCP4725_REFERENCE_VOLTAGE;                    // default for ref voltage 0 vdc

void setReferenceVoltage(double voltage);
bool setVoltage(double voltage, MCP4725_CmdType_e mode, MCP4725_PowerDownType_e powerType, U08 i2c_bus);
bool setRawDACValue(uint16_t InputCode, MCP4725_CmdType_e mode, MCP4725_PowerDownType_e powerType, U08 i2c_bus);

#endif