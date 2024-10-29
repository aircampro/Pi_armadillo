/*!
	@brief Read DAC register 
	@param mode MCP4725DAC datatype 1 3 or 5, see enum MCP4725_ReadType_e.
	@return  Requested value of read or type 0XFFFF if I2c error
*/
uint16_t readRegister(MCP4725_ReadType_e readType, U08 i2c_bus)
{
	uint16_t dataWord = readType;
	uint8_t dataBuffer[6];
	bool ReturnCode = false;
    uint16_t ret = 0;
	
	/*Format of read data :
	== Settings data one byte
	BSY,POR,xx,xx,xx,PD1,PD0,xx, 
	== DAC register data 3 byte(1st 1 don't care)
	D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx, 
	== EEPROM data 5 byte (1st 3 don't care)
	xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0
	*/
	switch (readType)
	{
		case MCP4725_ReadSettings: // Read one byte settings
			//ReturnCode = i2c_read_timeout_us(_i2c, _i2cAddress, dataBuffer, 1, false, MCP4725_I2C_DELAY);
			ReturnCode = I2cCtl_Read(_dev_addr, (void *)&dataBuffer, sizeof(dataBuffer), i2c_bus);
			if (ReturnCode == false) {
		        if (_serialDebug == true)
		        {
			        printf("1205 I2C Error readRegister : \r\n");
			        usleep(100);
		        }   
	 	        ret = MCP4725_ERROR;
            } else {
			    dataWord = dataBuffer[0];
                ret = dataWord;
            }
			break;

		case MCP4725_ReadDACReg: // Read 3 bytes  DAC register data, skip first 1 don't care
			// ReturnCode = i2c_read_timeout_us(_i2c, _i2cAddress, dataBuffer, 3, false, MCP4725_I2C_DELAY);
			ReturnCode = I2cCtl_Read(_dev_addr, (void *)&dataBuffer, sizeof(dataBuffer), i2c_bus));
			if (ReturnCode == false) {
		        if (_serialDebug == true)
		        {
			        printf("1205 I2C Error readRegister : \r\n");
			        usleep(100);
		        }   
	 	        ret = MCP4725_ERROR;
            } else {
			    dataWord = dataBuffer[1];
			    dataWord = (dataWord << 8) | dataBuffer[2];
                ret = dataWord;
            }
			break;

		case MCP4725_ReadEEPROM: // Read 5 bytes EEPROM data , first 3 don't care
			// ReturnCode = i2c_read_timeout_us(_i2c, _i2cAddress, dataBuffer, 5, false, MCP4725_I2C_DELAY);
			ReturnCode = I2cCtl_Read(_dev_addr, (void *)&dataBuffer, sizeof(dataBuffer), i2c_bus);
			if (ReturnCode == false) {
		        if (_serialDebug == true)
		        {
			        printf("1205 I2C Error readRegister : \r\n");
			        usleep(100);
		        }   
	 	        ret = MCP4725_ERROR;
            } else {
			    dataWord = dataBuffer[3];
			    dataWord = (dataWord << 8) | dataBuffer[4];
                ret = dataWord;
            }
			break;
	}
    usleep(MCP4725_I2C_DELAY*1000);
    return ret;
}

/*!
	@brief Get direct DAC register
	@param i2c_bus the i2c bus number
	@return  power type or 0xFFFF if I2C error
	@note Power type corresponds to enum MCP4725_PowerDownType_e
*/
uint16_t getDACInputCode(U08 i2c_bus)
{
	uint16_t inputValue = readRegister(MCP4725_ReadDACReg, i2c_bus); 
	//powerTypeValue = BSY,POR,xx,xx,xx,PD1,PD0,xx

	if (inputValue != MCP4725_ERROR){
		inputValue = inputValue & 0x0FFF;
	}
	return inputValue;
}

/*!
	@brief Get stored power type from EEPROM
	@return  EEPROM power type or 0xFFFF if I2C error
	@param i2c_bus the i2c bus number
	@note Power type corresponds to enum MCP4725_PowerDownType_e
*/
uint16_t getStoredPowerType(U08 i2c_bus)
{
	uint16_t powerTypeValue = readRegister(MCP4725_ReadEEPROM, i2c_bus); 
	//powerTypeValue = x,PD1,PD0,xx,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0

	if (powerTypeValue != MCP4725_ERROR)
	{
		powerTypeValue = powerTypeValue << 1;  //PD1,PD0,xx,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,00
		powerTypeValue = powerTypeValue >> 14; //00,00,00,00,00,00,00,00,00,00,00,00,00,00,PD1,PD0
	}
	return powerTypeValue;

}

/*!
	@brief Read DAC inputCode from EEPROM
	@param i2c_bus the i2c bus number
	@return  stored EEPROM inputcode value or 0xFFFF if I2C error
*/
uint16_t getStoredInputCode(U08 i2c_bus)
{
	uint16_t inputCode = readRegister(MCP4725_ReadEEPROM, i2c_bus); 
	//InputCode = x,PD1,PD0,x,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0

	if (inputCode != MCP4725_ERROR) 
		inputCode = inputCode & 0x0FFF;  //0,0,0,0,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0													
	return inputCode; // i2c Error return 0xFFFF
}

/*!
	@brief get EEPROM writing status from DAC register
	@param i2c_bus the i2c bus number
	@return  1 for completed or 0( busy or I2C error)
	@note The BSY bit is low (during the EEPROM writing)
*/
bool getEEPROMBusyFlag(U08 i2c_bus)
{
	uint16_t registerValue = readRegister(MCP4725_ReadSettings, i2c_bus);          
	//register value = BSY,POR,xx,xx,xx,PD1,PD0,xx
	bool ReturnValue = false;
	if (registerValue != MCP4725_ERROR)
	{
		ReturnValue = ((registerValue >> 7) & 0x01);    //1 - Not Busy, 0 - Busy
	}
    return ReturnValue; // I2C error

}

/*!
	@brief Writes data to DAC register or EEPROM 
	@param inputCode  0 to MCP4725_MAX_VALUE input code
	@param mode MCP4725DAC mode, see enum MCP4725_CmdType_e.
	@param PowerType MCP4725 power type, see enum MCP4725_PowerType_e
	@param i2c_bus the i2c bus number
	@return   true for success, false for failure.
*/
bool writeCommand2(uint16_t inputCode, MCP4725_CmdType_e mode, MCP4725_PowerDownType_e powerType, U08 i2c_bus)
{
	uint8_t dataBuffer[3];
	uint8_t lowByte = 0;
	uint8_t highByte = 0;
	bool ReturnCode = false;
    bool ret = true;
    
	switch (mode)
	{
		case MCP4725_FastMode:  
			//C2=0,C1=0,PD1,PD0,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0
			lowByte  = (uint8_t)(inputCode & 0x00FF);
			highByte = (uint8_t)((inputCode >> 8) & 0x00FF);
            for (ii=0; ii<3 ; ++ii) {
                dataBuffer[ii] = 0;
            }
			dataBuffer[0] = mode | (powerType << 4) | highByte; //C2,C1,PD1,PD0,D11,D10,D9,D8
  			dataBuffer[1] = lowByte;                            //D7,D6,D5,D4,D3,D2,D1,D0
			//ReturnCode = i2c_write_timeout_us(_i2c, _i2cAddress, dataBuffer, 2 , false, MCP4725_I2C_DELAY);
			ReturnCode = I2cCtl_Write(_dev_addr, (void *) &dataBuffer, sizeof(dataBuffer), i2c_bus);
			if (ReturnCode == false)
			{
				if (_serialDebug == true)
				{
					printf("1203 : I2C error :: WriteCommand 1 \r\n");
				}
			    ret = false;
			}	
			break;

		case MCP4725_RegisterMode: 
		    //C2=0,C1=1,C0=0,x,x,PD1,PD0,x,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,x,x,x,x
		case MCP4725_EEPROM_Mode: 
		    //C2=0,C1=1,C0=1,x,x,PD1,PD0,x,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,x,x,x,x	
			inputCode = inputCode << 4; //D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,x,x,x,x
			lowByte  = (uint8_t)(inputCode & 0x00FF);
			highByte = (uint8_t)((inputCode >> 8) & 0x00FF);
			dataBuffer[0] = mode | (powerType << 1);                       // C2,C1,C0,x,x,PD1,PD0,x
  			dataBuffer[1] = highByte;                                      // D11,D10,D9,D8,D7,D6,D5,D4
			dataBuffer[2] = lowByte;                                       // D3,D2,D1,D0,x,x,x,x
			
			// ReturnCode = i2c_write_timeout_us(_i2c, _i2cAddress, dataBuffer, 3 , false, MCP4725_I2C_DELAY);
			ReturnCode = I2cCtl_Write(_dev_addr, (void *) &dataBuffer, sizeof(dataBuffer), i2c_bus);
			if (ReturnCode == false)
			{
				if (_serialDebug == true)
				{
					printf("1204 : I2C error :: writeCommand 2 \r\n");
				}
				ret = false;
			}
			break;
	}
    usleep(MCP4725_I2C_DELAY*1000);		
			
	if ((mode == MCP4725_EEPROM_Mode) && (ret == true))
	{
		if !(getEEPROMBusyFlag(1) == true) {
		    usleep(MCP4725_EEPROM_WRITE_TIME*1000);                                //typical EEPROM write time 25 mSec
		    if !(getEEPROMBusyFlag(1) == true) {
		        usleep(MCP4725_EEPROM_WRITE_TIME*1000);                            //maximum EEPROM write time 25*2 mSec
		        if !(getEEPROMBusyFlag(1) == true) {
		            ret = false;                                                   // seems like an error writing to EEPROM
				    if (_serialDebug == true)
				    {
					    printf("1204 : I2C error :: BUSY not cleared when writing to EEPROM \r\n");
				    }
				}
			}
		}
	}

	return ret;
}

/*!
	@brief Sets the reference voltage. 
	@param voltage the reference voltage to be set, called from constructor.
*/
void setReferenceVoltage(double voltage)
{
	if (voltage == 0)
		_refVoltage = MCP4725_REFERENCE_VOLTAGE;
	else
		_refVoltage = voltage;

	_bitsPerVolt = static_cast<double>(MCP4725_STEPS / _refVoltage);
}

/*!
	@brief Set voltage out based on voltage input in volts. 
	@param voltage  0 to_MCP4725_REFERENCE_VOLTAGE, voltage out
	@param mode MCP4725DAC mode, see enum MCP4725_CmdType_e.
	@param powerType MCP4725DAC power type, see enum MCP4725_PowerType_e
	@param i2c_bus the i2c bus number
	@return  output of writeCommand method, true for success, false for failure.
*/
bool setVoltage(double voltage, MCP4725_CmdType_e mode, MCP4725_PowerDownType_e powerType, U08 i2c_bus)
{
	uint16_t voltageValueRaw = 0;

	// Convert voltage to DAC bits
	//xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
	if (_safetyCheck  == true)
	{
		if (voltage >= _refVoltage)
			voltageValueRaw = MCP4725_MAX_VALUE;
		else if (voltage <= 0)
			voltageValueRaw = 0; //make sure value never below zero
		else
			voltageValueRaw = static_cast<uint16_t>(std::round(voltage * _bitsPerVolt));
	}
	else if (_safetyCheck ==  false)
	{
		voltageValueRaw = static_cast<uint16_t>(std::round(voltage * _bitsPerVolt));
	}

	return writeCommand2(voltageValueRaw, mode, powerType, i2c_bus);
}

/*!
	@brief Set voltage out based on DAC input code. 
	@param InputCode 0 to MCP4725_MAX_VALUE.
	@param mode MCP4725DAC mode, see enum MCP4725_CmdType_e.
	@param powerType MCP4725DAC power type, see enum MCP4725_PowerType_e
	@param i2c_bus the i2c bus number
	@return  output of writeCommand method, true for success, false for failure.
*/
bool setRawDACValue(uint16_t InputCode, MCP4725_CmdType_e mode, MCP4725_PowerDownType_e powerType, U08 i2c_bus)
{
	if (_safetyCheck  == true)
	{
		if (InputCode > MCP4725_MAX_VALUE)
			InputCode = MCP4725_MAX_VALUE;
	}

	return writeCommand2(InputCode, mode, powerType, i2c_bus);
}


