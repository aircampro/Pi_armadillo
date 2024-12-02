# a library containing the registers in novus and procon add-ons which can be read over modbus
#
#
MODBUS_FUNCTION_READ_COILS                = 1
MODBUS_FUNCTION_READ_DISCRETE_INPUTS      = 2
MODBUS_FUNCTION_READ_HOLDING_REGS         = 3
MODBUS_FUNCTION_READ_INPUT_REGS           = 4
MODBUS_FUNCTION_WRITE_SINGLE_COIL         = 5
MODBUS_FUNCTION_WRITE_SINGLE_REG          = 6
MODBUS_FUNCTION_READ_EXCEPTION_STATUS     = 7
MODBUS_FUNCTION_DIAGNOSTICS               = 8
MODBUS_FUNCTION_GET_COMM_EVENT_COUNTER    = 11
MODBUS_FUNCTION_GET_COMM_EVENT_LOG        = 12
MODBUS_FUNCTION_WRITE_MULTIPLE_COILS      = 15
MODBUS_FUNCTION_WRITE_MULTIPLE_REGS       = 16
MODBUS_FUNCTION_REPORT_SLAVE_ID           = 17
MODBUS_FUNCTION_READ_FILE_RECORD          = 20
MODBUS_FUNCTION_WRITE_FILE_RECORD         = 21
MODBUS_FUNCTION_MASK_WRITE_REG            = 22
MODBUS_FUNCTION_READ_WRITE_MULTIPLE_REGS  = 23
MODBUS_FUNCTION_READ_FIFO_QUEUE           = 24
MODBUS_FUNCTION_GET_DETECTIONS            = 65
MODBUS_FUNCTION_READ_LEDDAR               = 66
MODBUS_FUNCTION_WRITE_LEDDAR              = 67
MODBUS_FUNCTION_OPCODE_LEDDAR             = 68
MODBUS_FUNCTION_PORT_LEDDAR               = 69
MODBUS_FUNCTION_ENCAPSULATED_IF_TRANSPORT = 43

MB_EX_NONE = 0x00                                                         # MBSuccess
MB_EX_ILLEGAL_FUNCTION = 0x01                                             # MBIllegalFunction
MB_EX_ILLEGAL_DATA_ADDRESS = 0x02                                         # MBIllegalDataAddress A request for a register that does not exist will return error code 2
MB_EX_ILLEGAL_DATA_VALUE = 0x03                                           # MBIllegalDataValue Trying to set a register to an invalid value will return error code 3
MB_EX_SLAVE_DEVICE_FAILURE = 0x04                                         # MBSlaveDeviceFailure If an error occurs while trying to execute the function, error code 4 will be returned
MB_EX_ACKNOWLEDGE = 0x05                                                  # Acknowledge
MB_EX_SLAVE_BUSY = 0x06                                                   # Slave Device Busy
MB_EX_NACK = 0x07                                                         # NACK
MB_EX_MEMORY_PARITY_ERROR = 0x08                                          # Memory Parity Error
MB_EX_GATEWAY_PATH_FAILED = 0x0A                                          # Gateway Path Unavailable
MB_EX_GATEWAY_TGT_FAILED = 0x0B                                           # Gateway Target Device Failed to Respond

MBSuccess                    = 0x00                                       # no error occurred
MBInvalidSlaveID             = 0xE0                                       # wrong slave id
MBInvalidFunction            = 0xE1                                       # invalid function code sent
MBResponseTimedOut           = 0xE2                                       # time out
MBInvalidCRC                 = 0xE3                                       # wrong crc
    
# %%%%%%%%%%% NOVS DigiRail-2A universal Analog Inpt Module %%%%%%%%%%%%%%%
NOVS_DR2A_DEF_ADDR = 247     
                                          
# READ INPUT REGISTERS - 04H
NOVS_DR2A_PV_CH1 = 0                                                    # PV of Channel 1 in percentage. Range from 0 to 62000.
NOVS_DR2A_PV_CH2 = 1                                                    # PV of Channel 2 in percentage. Range from 0 to 62000.
NOVS_DR2A_ENGV_CH1 = 5                                                  # PV of Channel 1 in engineering nit. Range: defined by the limits of the temperatre sensor, or by the indication limits as defined by the parameters 42 and 43 of the Holding Registers.
NOVS_DR2A_ENGV_CH2 = 6

# Novus_Dr2A_UpLim_e set for NOVS_DR2A_CH(1/2)_HIGH
NDR2A_ulDISABLED = -1 
NDR2A_ulTCJ = 940 
HDR2A_ulTCK=1370 
NDR2A_ulTCT=400 
NDR2A_ulTCE=720 
NDR2A_ulTCN=1300 
NDR2A_ulTCR=1760 
NDR2A_ulTCS=1760 
NDR2A_ulTCB=1800 
NDR2A_ulPt100=650 
NDR2A_ul50mV=31000 
NDR2A_ul20mV=31000 
NDR2A_ulminus10to20mV=31000 
NDR2A_ul5V=31000 
NDR2A_ul10V=31000 
NDR2A_ul0to20mA=31000 
NDR2A_ul4to20mA=31000  
# Novus_Dr2A_LoLim_e set for NOVS_DR2A_CH(1/2)_LOW
NDR2A_loDISABLED = -1 
NDR2A_loTCJ = -130 
HDR2A_loTCK=-200 
NDR2A_loTC=-200 
NDR2A_loTCE=-100 
NDR2A_loTCN=-200 
NDR2A_loTCR=0 
NDR2A_loTCS=0 
NDR2A_loTCB=500 
NDR2A_loPt100=-200 
NDR2A_lo50mV=0 
NDR2A_lo20mV=0 
NDR2A_lominus10to20mV=0 
NDR2A_loul5V=0 
NDR2A_lo10V=0 
NDR2A_lo0to20mA=0 
NDR2A_lo4to20mA=0 

# MODBUS_FUNCTION_READ HOLDING REGISTERS – 03H
# MODBUS_FUNCTION_WRITE SINGLE REGISTER – 06H
NOVS_DR2A_CH1_LOW = 41                                                  # Lower indication limit of channel 1 when inpt type is linear (RW)
NOVS_DR2A_CH1_HIGH = 42                                                 # upper indication limit of channel 1 when inpt type is linear (RW) Novus_Dr2A_UpLim_e 
NOVS_DR2A_CH2_LOW = 43                                                  # Lower indication limit of channel 2 when inpt type is linear (RW)
NOVS_DR2A_CH2_HIGH = 44                                                 # upper indication limit of channel 2 when inpt type is linear (RW) Novus_Dr2A_UpLim_e 
NOVS_DR2A_MOD_ADDR = 3                                                  # module address (RW)
NOVS_DR2A_FILTER = 8                                                    # 0-20 filter (RW)
NOVS_DR2A_CHSTAT = 7                                                    # channel stats (RO)
NOVS_DR2A_SQRT1 = 36                                                    # Square root fitness for channel 1 (0 or 1) (RW)
NOVS_DR2A_SQRT2 = 37                                                    # Square root fitness for channel 2 (0 or 1) (RW)

# Novs_Dr2A_Typ_e -- the type of input is defined by the following
NDR2A_DISABLED = -1
NDR2A_TCJ = 0 
HDR2A_TCK=1 
NDR2A_TC=2 
NDR2A_TCE=3 
NDR2A_TCN=4 
NDR2A_TCR=5 
NDR2A_TCS=6 
NDR2A_TCB=7 
NDR2A_Pt100=8 
NDR2A_50mV=9 
NDR2A_20mV=10 
NDR2A_minus10to20mV=11 
NDR2A_5V=12 
NDR2A_10V=13 
NDR2A_0to20mA=18 
NDR2A_4to20mA=19 
# above to be used with the following addresses below
NOVS_DR2A_TYPE1 = 21                                                    # Type for channel 1 (as Novs_Dr2A_Typ_e ) (RW)
NOVS_DR2A_TYPE2 = 22                                                    # Type for channel 2 (as Novs_Dr2A_Typ_e ) (RW)

# %%%%%%%%%%% NOVS DigiRail-2R universal 2 relay out Module %%%%%%%%%%%%%%%
# MODBUS_FUNCTION_WRITE SINGLE COIL - 05H
NOVS_DR2R_ROT1_STAT = 0
NOVS_DR2R_ROT2_STAT = 1

# MODBUS_FUNCTION_WRITE MLTIPLE COILS – 0FH
NOVS_DR2R_ROT_BEGIN = 0
NOVS_DR2R_ROT_END = 1

# MODBUS_FUNCTION_READ HOLDING REGISTERS – 03H  PLC 40001 = 0 WRITE SINGLE REGISTER – 06H
NOVS_DR2R_ROT_STAT = 7                                                  # bit 1 = ot 2 bit 0 = ot1
NOVS_DR2R_ROT1_TH = 12                                                  # Vale of timing for digital otpt 1, in hndredths of seconds - word high
NOVS_DR2R_ROT1_LH = 13                                                  # Vale of timing for digital otpt 1, in hndredths of seconds - word low
NOVS_DR2R_ROT2_TH = 14                                                  # Vale of timing for digital otpt 2, in hndredths of seconds - word high
NOVS_DR2R_ROT2_LH = 15                                                  # Vale of timing for digital otpt 2, in hndredths of seconds - word low
NOVS_DR2R_ROT1_CrTH = 20                                               # for deactivating digital otpt 1, in hndredths of seconds - word high
NOVS_DR2R_ROT1_CrLH = 21                                               # for deactivating digital otpt 1, in hndredths of seconds - word low
NOVS_DR2R_ROT2_CrTH = 22                                               # for deactivating digital otpt 2, in hndredths of seconds - word high
NOVS_DR2R_ROT2_CrLH = 23                                               # for deactivating digital otpt 2, in hndredths of seconds - word low

# %%%%%%%%%%% NOVS DigiRail-4C universal counter Module %%%%%%%%%%%%%%%
# MODBUS_FUNCTION_READ HOLDING REGISTERS – 03H  PLC 40001 = 0 WRITE SINGLE REGISTER – 06H
NOVS_DR4C_DIN_STAT = 7                                                  # State of the digital inpts, where bit 0 represents inpt 1, bit 1 represents inpt 2 (0=off; 1=on),  etc
NOVS_DR4C_CNT1_TH = 18                                                  # Vale of the inpt 1 counts - most significant word
NOVS_DR4C_CNT1_LH = 19                                                  # Vale of the inpt 1 counts - least significant word
NOVS_DR4C_CNT2_TH = 20                                                  # Vale of the inpt 2 counts - most significant word
NOVS_DR4C_CNT2_LH = 21                                                  # Vale of the inpt 2 counts - least significant word
NOVS_DR4C_CNT3_TH = 22                                                  # Vale of the inpt 3 counts - most significant word
NOVS_DR4C_CNT3_LH = 23                                                  # Vale of the inpt 3 counts - least significant word
NOVS_DR4C_CNT4_TH = 24                                                  # Vale of the inpt 4 counts - most significant word
NOVS_DR4C_CNT4_LH = 25                                                  # Vale of the inpt 4 counts - least significant word
NOVS_DR4C_inCFG = 16                                                    # Configration of the digital inpts Bit 0 in 1 - Inpt 1 operates as fast conter inpt. Debonce for inpt 1 is ignored
NOVS_DR4C_pResetDir = 17                                                # set-p preset conter (e.g. distance tacho) and direction of plse rising edge for conter as per D4C_Config_e
NOVS_DR4C_DB1_TH = 12                                                   # Debonce of the digital inpt 1 (in ms) 0-10000
NOVS_DR4C_DB2_TH = 13                                                   # Debonce of the digital inpt 2 (in ms) 0-10000
NOVS_DR4C_DB3_TH = 14                                                   # Debonce of the digital inpt 3 (in ms) 0-10000
NOVS_DR4C_DB4_TH = 15                                                   # Debonce of the digital inpt 4 (in ms) 0-10000
NOVS_DR4C_PlsInt = 59                                                  # Plse conting interval time (in 0.1 seconds) 0-36000 a vale of 10 means a 1 second interval. When we configre a 1 second interval, the conting will be in Hertz
NOVS_DR4C_PlsPInt = 60                                                 # Peak plse conting interval time (in 0.1 seconds) 0-36000 Time interval on which inpt plses are totalized and, in case they are bigger than the crrent vale, they are stored on registers 71 to 78
NOVS_DR4C_PlsRt1H = 63                                                 # counted plses in the last interval for inpt 1 - most significant word
NOVS_DR4C_PlsRt1L = 64                                                 # counted plses in the last interval for inpt 1 - least significant word
NOVS_DR4C_PlsRt2H = 65                                                 # counted plses in the last interval for inpt 2 - most significant word
NOVS_DR4C_PlsRt2L = 66                                                 # counted plses in the last interval for inpt 2 - least significant word
NOVS_DR4C_PlsRt3H = 67                                                 # counted plses in the last interval for inpt 3 - most significant word
NOVS_DR4C_PlsRt3L = 68                                                 # counted plses in the last interval for inpt 3 - least significant word
NOVS_DR4C_PlsRt4H = 69                                                 # counted plses in the last interval for inpt 4 - most significant word
NOVS_DR4C_PlsRt4L = 70                                                 # counted plses in the last interval for inpt 4 - least significant word
NOVS_DR4C_PlsPk1H = 71                                                 # Maximm of plses counted in the last interval for inpt 1 - most significant word
NOVS_DR4C_PlsPk1L = 72                                                 # Maximm of plses counted in the last interval for inpt 1 - least significant word
NOVS_DR4C_PlsPk2H = 73                                                 # Maximm of plses counted in the last interval for inpt 2 - most significant word
NOVS_DR4C_PlsPk2L = 74                                                 # Maximm of plses counted in the last interval for inpt 2 - least significant word
NOVS_DR4C_PlsPk3H = 75                                                 # Maximm of plses counted in the last interval for inpt 3 - most significant word
NOVS_DR4C_PlsPk3L = 76                                                 # Maximm of plses counted in the last interval for inpt 3 - least significant word
NOVS_DR4C_PlsPk4H = 77                                                 # Maximm of plses counted in the last interval for inpt 4 - most significant word
NOVS_DR4C_PlsPk4L = 78                                                 # Maximm of plses counted in the last interval for inpt 4 - least significant word
# states of NOVS_DR4C_pResetDir1
NOVS_DR4C_enabPre1 = (1<<0)                                            # enable preset no.1
NOVS_DR4C_enabPre2 = (1<<1)                                            # enable preset no.2
NOVS_DR4C_enabPre3 = (1<<2)                                            # enable preset no.3
NOVS_DR4C_enabPre4 = (1<<3)                                            # enable preset no.4
NOVS_DR4C_inhib1 = (1<<4)                                              # inhibit preset no.1
NOVS_DR4C_inhib2 = (1<<5)                                              # inhibit preset no.2
NOVS_DR4C_inhib3 = (1<<6)                                              # inhibit preset no.3
NOVS_DR4C_inhib4 = (1<<7)                                              # inhibit preset no.4
NOVS_DR4C_edge1 = (1<<8)                                               # rising edge = 0 falling edge = 1 to count i.e Bit in 1 - Cont at the negative margin (1 for 0)
NOVS_DR4C_edge2 = (1<<9)                                               # rising edge = 0 falling edge = 1 to count
NOVS_DR4C_edge3 = (1<<10)                                              # rising edge = 0 falling edge = 1 to count
NOVS_DR4C_edge4 = (1<<12)                                              # rising edge = 0 falling edge = 1 to count

NOVS_DR4C_Preset1H = 26                                                 # Preset vale of the inpt 1 counts - most significant word
NOVS_DR4C_Preset1L = 27                                                 # Preset vale of the inpt 1 counts - least significant word
NOVS_DR4C_Preset2H = 28                                                 # Preset vale of the inpt 2 counts - most significant word
NOVS_DR4C_Preset2L = 29                                                 # Preset vale of the inpt 2 counts - least significant word
NOVS_DR4C_Preset3H = 30                                                 # Preset vale of the inpt 3 counts - most significant word
NOVS_DR4C_Preset3L = 31                                                 # Preset vale of the inpt 3 counts - least significant word
NOVS_DR4C_Preset4H = 32                                                 # Preset vale of the inpt 4 counts - most significant word
NOVS_DR4C_Preset4L = 33                                                 # Preset vale of the inpt 4 counts - least significant word

# %%%%%%% Procon Electronic Modbus TCP 16DI logic and counter up to 1KHz %%%%%
#
PROCON_TCP16DI_DEF_IP = "169.254.111.11"                                  # Defalt IP Address

# READ HOLDING REGISTERS – 03H
# The specified addresses correspond to the low level physical addresses, 
# where zero (0) corresponds to the address of PLC 40001
PROCON_PT16DI_Mode = 100                                                   # 40101 0=Disable, 1=p Counting, 2=p/Down Count
PROCON_PT16DI_Filter = 101                                                 # 40102 Filter in 0.1 secs units
# MODBUS_FUNCTION_READ_DISCRETE_INPTS
PROCON_PT16DI_DIStart = 0                                                # 10001 Stats of Digital Inpts. (RO)
PROCON_PT16DI_DIEnd = 15                                                 # 10016 Stats of Digital Inpts. (RO)
# MODBUS_FUNCTION_READ INPUT REGISTERS - 04H
PROCON_PT16DI_DIWord = 1                                                 # 30002 Stats of Digital Inpts. (RO)
# MODBUS_FUNCTION_READ_DISCRETE_INPTS
PROCON_PT8DIO_DIStart = 0                                                # 10001 Stats of Digital Inpts. (RO)
PROCON_PT8DIO_DIEnd = 7                                                  # 10008 Stats of Digital Inpts. (RO)
# MODBUS_FUNCTION_WRITE_SINGLE_COIL   05H
# MODBUS_FUNCTION_WRITE MLTIPLE COILS – 0FH
# MODBUS_FUNCTION_READ COILS – 01H
# The specified addresses correspond to the low level physical addresses, where zero (0) corresponds to the address of PLC 00001
PROCON_PT8DIO_DOStart = 17                                               # Stats of Digital Otpts. (RW)
PROCON_PT8DIO_DOEnd = 24                                                 # Stats of Digital Otpts. (RW)
# MODBUS_FUNCTION__READ_INPUT_REGS
PROCON_PT8DIO_DIWord = 1                                                 # 30002 Stats of Digital Inpts. (RO)
# MODBUS_FUNCTION_READ_HOLDING_REGS
PROCON_PT8DIO_DOWord = 2                                                 # 40003 Stats of Digital Otpts. (RW)
# MODBUS_FUNCTION_READ_HOLDING_REGS
PROCON_PT4RO_DOWord = 1                                                  # 40002 4 Relay Opts (RW) X,X,X,X,X,X,X,X, X,X,X,X,4,3,2,1
# MODBUS_FUNCTION_READ_HOLDING_REGS
PROCON_PT8AVOI_AO1Start = 1                                              # 40002 either PT8AO or PT8VO volts or crrent
PROCON_PT8AVOI_AO8Stop = 8                                               # 40009

# --------- Procon PL101 PLC Webserver or Logic Interface ---------------------

# func3or4 Reads a range of registers from RAM, EEPROM and BBRAM Range M0-M1219 max number 100
# 5 Writes a single Bit to any part of RAM M0-M1219 max number 1
# func6 Writes a single register to RAM, EEPROM and BBRAM Range M0-M1219 max number 1
# func15 Writes a range of bits to RAM. range M9-M999 max number 1600
# func16 Writes a range of registers to RAM, EEPROM and BBRAM. range M9-M1219 max number 100

# user RAM stored and not saved
# M161 – M199 start 40162 161 end 40200 199 
# shown in realtime in webpage where referenced as %M0000161 to %M0000199
# where %Mfwdxxxx 
# f= Format Field 0 Unsigned Single 1 Signed Single 2 Unsigned Double 3 Signed Double 4 Float
# w – Width Field  This field is used to specify the minimum number of characters to generate 
# for the conversion. A value of zero (0) will let it the function generate an unrestricted number of characters
# d – Decimal Places Field  This field is used to specify the number of fraction characters to generate for the conversion after the decimal point
PROCON_PL101_URAM1_START = 161
PROCON_PL101_URAM1_STOP = 199

# M401 – M999  start 40402 401 end 41000 999
PROCON_PL101_URAM2_START = 401
PROCON_PL101_URAM2_STOP = 999

# user EEPROM stored and not often
# M1000 – M1199 start 41001 1000 end 41170 1169
PROCON_PL101_EEPROM_START = 1000
PROCON_PL101_EEPROM_STOP = 1169

# user BBRAM stored saved and often
# M1200 – M1219 start 41208 1207 end 41220 1219
PROCON_PL101_BBRAM_START = 1207
PROCON_PL101_BBRAM_STOP = 1219

# enron flow computer extended modbus defs
ENRON_ADDR_BOOL_START = 0x03E9                                          # Address boundaries as defined by ENRON
ENRON_ADDR_BOOL_STOP = 0x07CF
ENRON_ADDR_UINT16_START = 0x0BB9
ENRON_ADDR_UINT16_STOP = 0x0F9F
ENRON_ADDR_UINT32_START = 0x1389
ENRON_ADDR_UINT32_STOP = 0x176F
ENRON_ADDR_FLOAT32_START = 0x1B59
ENRON_ADDR_FLOAT32_STOP = 0x1F3F