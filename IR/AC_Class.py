#
# use with this library https://github.com/Ericmas001/HVAC-IR-Control
# adapted from to add toshiba, panasonic and mitsubishi W001CP R61Y23304 Remote Controller by Aircampro April 2025
#
# HVAC-IR-Control - Python port for RPI3
# Eric Masse (Ericmas001) - 2017-06-30
# 
# Tested on Mitsubishi Model MSZ-FE12NA
#
# From original: https://github.com/r45635/HVAC-IR-Control
# (c)  Vincent Cruvellier - 10th, January 2016 - Fun with ESP8266
#
import ir_sender
import pigpio
from datetime import datetime
import ctypes

from enum import Enum

# for Toshiba AC system
class ToshibaAC_Mode(Enum):   
    HVAC_HOT = 3
    HVAC_COLD = 1
    HVAC_DRY = 2
    HVAC_AUTO = 0

class ToshibaAC_FanSpd(Enum): 
    SPEED_1 = 96
    SPEED_2 = 01100000
    SPEED_3 = 128
    SPEED_4 = 160
    SPEED_5 = 192 
    AUTO = 0
    SPEED_SILENT = 0 

# for Panasonic AC system
class PanasonicAC_Mode(Enum):   
    HVAC_HOT = 64
    HVAC_FAN = 96
    HVAC_COLD = 24
    HVAC_DRY = 32
    HVAC_AUTO = 0

class PanasonicAC_FanMode(Enum):
    FAN_SPEED_1 = 0b00110000                                        # (Slowest)
    FAN_SPEED_2 = 0b01000000
    FAN_SPEED_3 = 0b01010000
    FAN_SPEED_4 = 0b01100000
    FAN_SPEED_5 = 0b01010000
    FAN_SPEED_AUTO = 0b10100000
    VANNE_AUTO = 0b00001111
    VANNE_H1 = 0b00000001                                           # (Horizontal)
    VANNE_H2 = 0b00000010
    VANNE_H3 = 0b00000011
    VANNE_H4 = 0b00000100
    VANNE_H5 = 0b00000101                                           # (Ground)

class PanasonicAC_Profile(Enum):
    NORMAL = 0b00010000
    QUIET = 0b01100000
    BOOST = 0b00010001
    
# for mitsubishi AC system  
class Mitsi_w001cp_Mode(Enum):  
    HOT = 0b00000010
    COLD = 0b00000001
    DRY = 0b00000101
    FAN = 0b00000000
    AUTO = 0b00000011

class Mitsi_w001cp_Fan(Enum):
    SPEED_1 = 0b00000001
    SPEED_2 = 0b00000011
    SPEED_3 = 0b00000101
    SPEED_4 = 0b00000111
    SPEED_5 = 0b00000111
    SPEED_AUTO = 0b00000101 
    SPEED_SILENT = 0b00000001
    VANNE_AUTO = 0b11000000
    VANNE_H1 = 0b00000000
    VANNE_H2 = 0b00010000
    VANNE_H3 = 0b00100000
    VANNE_H4 = 0b00110000
    VANNE_H5 = 0b00110000
    VANNE_AUTO_MOVE = 0b11000000 

class PowerMode:
    """
    PowerMode
    """
    PowerOff = 0b00000000       # 0x00      0000 0000        0
    PowerOn = 0b00100000        # 0x20      0010 0000       32

class ClimateMode:
    """
    ClimateMode
    """
    Hot = 0b00001000            # 0x08      0000 1000        8
    Cold = 0b00011000           # 0x18      0001 1000       24
    Dry = 0b00010000            # 0x10      0001 0000       16
    Auto = 0b00100000           # 0x20      0010 0000       32

    __Hot2 = 0b00000000         # 0x00      0000 0000        0
    __Cold2 = 0b00000110        # 0x06      0000 0110        6
    __Dry2 = 0b00000010         # 0x02      0000 0010        2
    __Auto2 = 0b00000000        # 0x00      0000 0000        0

    @classmethod
    def climate2(cls, climate_mode):
        """
        climate2: Converts to the second climate value (For ClimateAndHorizontalVanne)
        """
        if climate_mode == cls.Hot:
            return cls.__Hot2
        if climate_mode == cls.Cold:
            return cls.__Cold2
        if climate_mode == cls.Dry:
            return cls.__Dry2
        if climate_mode == cls.Auto:
            return cls.__Auto2


class ISeeMode:
    """
    ISeeMode
    """
    ISeeOff = 0b00000000        # 0x00      0000 0000        0
    ISeeOn = 0b01000000         # 0x40      0100 0000       64
    
class PowerfulMode:
    """
    PowerfulMode
    """
    PowerfulOff = 0b00000000        # 0x00      0000 0000        0
    PowerfulOn = 0b00001000         # 0x08      0000 1000        8

class VanneHorizontalMode:
    """
    VanneHorizontalMode
    """
    NotSet = 0b00000000         # 0x00      0000 0000        0
    Left = 0b00010000           # 0x10      0001 0000       16
    MiddleLeft = 0b00100000     # 0x20      0010 0000       32
    Middle = 0b00110000         # 0x30      0011 0000       48
    MiddleRight = 0b01000000    # 0x40      0100 0000       64
    Right = 0b01010000          # 0x50      0101 0000       80
    Swing = 0b11000000          # 0xC0      1100 0000      192

class FanMode:
    """
    FanMode
    """
    Speed1 = 0b00000001         # 0x01      0000 0001        1
    Speed2 = 0b00000010         # 0x02      0000 0010        2
    Speed3 = 0b00000011         # 0x03      0000 0011        3
    Auto = 0b10000000           # 0x80      1000 0000      128

class VanneVerticalMode:
    """
    VanneVerticalMode
    """
    Auto = 0b01000000           # 0x40      0100 0000       64
    Top = 0b01001000            # 0x48      0100 1000       72
    MiddleTop = 0b01010000      # 0x50      0101 0000       80
    Middle = 0b01011000         # 0x58      0101 1000       88
    MiddleBottom = 0b01100000   # 0x60      0110 0000       96
    Bottom = 0b01101000         # 0x68      0110 1000      104
    Swing = 0b01111000          # 0x78      0111 1000      120

class TimeControlMode:
    """
    TimeControlMode
    """
    NoTimeControl = 0b00000000  # 0x00      0000 0000        0
    ControlStart = 0b00000101   # 0x05      0000 0101        5
    ControlEnd = 0b00000011     # 0x03      0000 0011        3
    ControlBoth = 0b00000111    # 0x07      0000 0111        7

class AreaMode:
    """
    AreaMode
    """
    NotSet = 0b00000000         # 0x00      0000 0000        0
    Left = 0b01000000           # 0x40      0100 0000       64
    Right = 0b11000000          # 0xC0      1100 0000      192
    Full = 0b10000000           # 0x80      1000 0000      128

# send format is :- HdrMark, HdrSpace [ data...  BitMark, OneSpace == 1 or BitMark, ZeroSpace == 0  end of data...] RptMark   
# 
class Mitsi_Delay:
    """
    Mitsubishi AC Delay
    """
    HdrMark = 3400
    HdrSpace = 1750
    BitMark = 450
    OneSpace = 1300
    ZeroSpace = 420
    RptMark = 440
    RptSpace = 17100

class Tosh_Delay:
    """
    Toshiba`AC Delay
    """
    HdrMark = 4400
    HdrSpace = 4300
    BitMark = 543
    OneSpace = 1623
    ZeroSpace = 472
    RptMark = 440
    RptSpace = 7048

class Pana_Delay:
    """
    Panasonic`AC Delay
    """
    HdrMark = 3500
    HdrSpace = 1750
    BitMark = 435
    OneSpace = 1300
    ZeroSpace = 435
    RptMark = 435
    RptSpace = 10000

class Sanyo_Delay:
    """
    Panasonic`AC Delay
    """
    HdrMark = 3500
    HdrSpace = 950
    OneMark = 2400
    ZeroMark = 700
    FlexSpace = 800
    Len = 45000
    
class NEC_Delay:
    """
    NEC 
    """
    HdrMark = 9000
    HdrSpace = 4500
    BitMark = 560
    OneSpace = 1600
    ZeroSpace = 560
    RptMark = 0
    RptSpace = 2250

class SONY_Delay:
    """
    NEC 
    """
    HdrMark = 2400
    HdrSpace = 600
    BitMark = 0
    OneSpace = 1200
    ZeroSpace = 600
    RptLen = 45000
    DoubleSpace = 500
    RptMark = 0
    RptSpace = 2250

class AC_Types(Enum):  
    """
    types of AC to choose 
    """ 
    MITSUBISHI = 0
    TOSHIBA = 1
    PANASONIC = 2
    
class Index:
    """
    Index
    """
    Header0 = 0
    Header1 = 1
    Header2 = 2
    Header3 = 3
    Header4 = 4
    Power = 5
    ClimateAndISee = 6
    Temperature = 7
    ClimateAndHorizontalVanne = 8
    FanAndVerticalVanne = 9
    Clock = 10
    EndTime = 11
    StartTime = 12
    TimeControlAndArea = 13
    Unused14 = 14
    PowerfulMode = 15
    Unused16 = 16
    CRC = 17

class Constants:
    """
    Constants
    """
    Frequency = 38000       # 38khz
    MinTemp = 16
    MaxTemp = 31
    MaxMask = 0xFF
    NbBytes = 18
    NbPackets = 2           # For Mitsubishi IR protocol we have to send two time the packet data

# reverse the bits in a byte
def reverse_bits( data_byte ):
    data_byte_in = (ctypes.c_uint8 * 1)()                                  # force uint8 type so it works with -1 -ve numbers
    data_byte_in[0] = data_byte
    ss = str(bin(data_byte_in[0]))
    ss = ss[2:]                                                            # cut pre-amble
    leni = len(ss)
    if leni == 7:
        ss = "0"+ss
    elif leni == 6:
        ss = "00"+ss   
    elif leni == 5:
        ss = "000"+ss   
    elif leni == 4:
        ss = "0000"+ss  
    elif leni == 3:
        ss = "00000"+ss  
    elif leni == 2:
        ss = "000000"+ss    
    elif leni == 1:
        ss = "0000000"+ss  
    elif leni == 0 or leni >= 9:
        print("invalid length or object to reverse bits should be a byte")
        return "-1", -1         
    i = len(ss) - 1                                                               # index
    aa = ''
    val = 0
    while i >= 0:
        aa += ss[i]
        if int(ss[i]) == 1:
            val |= int(math.pow(2,i)) 
        i -= 1
    str_val = '0b' + aa
    return str_val, val
    
class IR_AC:
    """
    Mitsubishi AC, Toshiba AC, Panasonic AC
    """
    def __init__(self, gpio_pin, log_level=ir_sender.LogLevel.Minimal):
        self.log_level = log_level
        self.gpio_pin = gpio_pin
        self.delay_list = [Mitsi_Delay, Tosh_Delay, Pana_Delay]                # allows choice of pulse timings according to AC_Types class
        
    def power_off(self):
        """
        power_off
        """
        self.__send_command(
            ClimateMode.Auto,
            21,
            FanMode.Auto,
            VanneVerticalMode.Auto,
            VanneHorizontalMode.Swing,
            ISeeMode.ISeeOff,
            AreaMode.NotSet,
            None,
            None,
            PowerfulMode.PowerfulOff,
            PowerMode.PowerOff)

    def send_command_mitsi(self,
                     climate_mode=ClimateMode.Auto,
                     temperature=21,
                     fan_mode=FanMode.Auto,
                     vanne_vertical_mode=VanneVerticalMode.Auto,
                     vanne_horizontal_mode=VanneHorizontalMode.NotSet,
                     isee_mode=ISeeMode.ISeeOff,
                     area_mode=AreaMode.NotSet,
                     start_time=None,
                     end_time=None,
                     powerful=PowerfulMode.PowerfulOff):
        """
        send_command
        """
        self.__send_command(
            climate_mode,
            temperature,
            fan_mode,
            vanne_vertical_mode,
            vanne_horizontal_mode,
            isee_mode,
            area_mode,
            start_time,
            end_time,
            powerful,
            PowerMode.PowerOn)

    def __log(self, min_log_level, message):
        if min_log_level <= self.log_level:
            print(message)

    def __send_command(self, climate_mode, temperature, fan_mode, vanne_vertical_mode, vanne_horizontal_mode, isee_mode, area_mode, start_time, end_time, powerful, power_mode, typ=AC_Types.MITSUBISHI.value):

        Delay = self.delay_list[typ]
        
        sender = ir_sender.IrSender(self.gpio_pin, "NEC", dict(
            leading_pulse_duration=Delay.HdrMark,
            leading_gap_duration=Delay.HdrSpace,
            one_pulse_duration=Delay.BitMark,
            one_gap_duration=Delay.OneSpace,
            zero_pulse_duration=Delay.BitMark,
            zero_gap_duration=Delay.ZeroSpace,
            trailing_pulse_duration=Delay.RptMark,
            trailing_gap_duration=Delay.RptSpace), self.log_level)

        # data array is a valid frame, only byte to be changed will be updated.
        data = [0x23, 0xCB, 0x26, 0x01, 0x00, 0x20,
                0x08, 0x06, 0x30, 0x45, 0x67, 0x00,
                0x00, 0x00, 0x10, 0x00, 0x00, 0x1F]

        self.__log(ir_sender.LogLevel.Verbose, '')
        data[Index.Power] = power_mode
        self.__log(ir_sender.LogLevel.Verbose, 'PWR: {0:03d}  {0:02x}  {0:08b}'.format(data[Index.Power]))
        self.__log(ir_sender.LogLevel.Verbose, '')
        data[Index.ClimateAndISee] = climate_mode | isee_mode
        self.__log(ir_sender.LogLevel.Verbose, 'CLI: {0:03d}  {0:02x}  {0:08b}'.format(climate_mode))
        self.__log(ir_sender.LogLevel.Verbose, 'SEE: {0:03d}  {0:02x}  {0:08b}'.format(isee_mode))
        self.__log(ir_sender.LogLevel.Verbose, 'CLS: {0:03d}  {0:02x}  {0:08b}'.format(data[Index.ClimateAndISee]))
        self.__log(ir_sender.LogLevel.Verbose, '')
        data[Index.Temperature] = max(Constants.MinTemp, min(Constants.MaxTemp, temperature)) - 16
        self.__log(ir_sender.LogLevel.Verbose, 'TMP: {0:03d}  {0:02x}  {0:08b} (asked: {1})'.format(data[Index.Temperature], temperature))
        self.__log(ir_sender.LogLevel.Verbose, '')
        data[Index.ClimateAndHorizontalVanne] = ClimateMode.climate2(climate_mode) | vanne_horizontal_mode
        self.__log(ir_sender.LogLevel.Verbose, 'CLI: {0:03d}  {0:02x}  {0:08b}'.format(ClimateMode.climate2(climate_mode)))
        self.__log(ir_sender.LogLevel.Verbose, 'HOR: {0:03d}  {0:02x}  {0:08b}'.format(vanne_horizontal_mode))
        self.__log(ir_sender.LogLevel.Verbose, 'CLH: {0:03d}  {0:02x}  {0:08b}'.format(data[Index.ClimateAndHorizontalVanne]))
        self.__log(ir_sender.LogLevel.Verbose, '')
        data[Index.FanAndVerticalVanne] = fan_mode | vanne_vertical_mode
        self.__log(ir_sender.LogLevel.Verbose, 'FAN: {0:03d}  {0:02x}  {0:08b}'.format(data[Index.FanAndVerticalVanne]))
        self.__log(ir_sender.LogLevel.Verbose, '')

        now = datetime.today()
        data[Index.Clock] = (now.hour*6) + (now.minute//10)
        self.__log(ir_sender.LogLevel.Verbose, 'CLK: {0:03d}  {0:02x}  {0:08b} {1}'.format(data[Index.Clock], now))
        self.__log(ir_sender.LogLevel.Verbose, '')

        data[Index.EndTime] = 0 if end_time is None else ((end_time.hour*6) + (end_time.minute//10))
        self.__log(ir_sender.LogLevel.Verbose, 'ETI: {0:03d}  {0:02x}  {0:08b} {1}'.format(data[Index.EndTime], end_time))
        self.__log(ir_sender.LogLevel.Verbose, '')
        data[Index.StartTime] = 0 if start_time is None else ((start_time.hour*6) + (start_time.minute//10))
        self.__log(ir_sender.LogLevel.Verbose, 'STI: {0:03d}  {0:02x}  {0:08b} {1}'.format(data[Index.StartTime], start_time))
        self.__log(ir_sender.LogLevel.Verbose, '')

        time_control = TimeControlMode.NoTimeControl
        if end_time is not None and start_time is not None:
            time_control = TimeControlMode.ControlBoth
        elif end_time is not None:
            time_control = TimeControlMode.ControlEnd
        elif start_time is not None:
            time_control = TimeControlMode.ControlStart
        else:
            time_control = TimeControlMode.NoTimeControl
        data[Index.TimeControlAndArea] = time_control | area_mode 
        self.__log(ir_sender.LogLevel.Verbose, 'TIC: {0:03d}  {0:02x}  {0:08b}'.format(time_control))
        self.__log(ir_sender.LogLevel.Verbose, 'AEA: {0:03d}  {0:02x}  {0:08b}'.format(area_mode))
        self.__log(ir_sender.LogLevel.Verbose, 'TCA: {0:03d}  {0:02x}  {0:08b}'.format(data[Index.TimeControlAndArea]))
        self.__log(ir_sender.LogLevel.Verbose, '')
        
        data[Index.PowerfulMode] = powerful
        self.__log(ir_sender.LogLevel.Verbose, 'FUL: {0:03d}  {0:02x}  {0:08b}'.format(data[Index.PowerfulMode]))
        self.__log(ir_sender.LogLevel.Verbose, '')

        # CRC is a simple bits addition
        # sum every bytes but the last one
        data[Index.CRC] = sum(data[:-1]) % (Constants.MaxMask + 1)
        self.__log(ir_sender.LogLevel.Verbose, 'CRC: {0:03d}  {0:02x}  {0:08b}'.format(data[Index.CRC]))
        self.__log(ir_sender.LogLevel.Verbose, '')

        sender.send_data(data, Constants.MaxMask, True, Constants.NbPackets)
   
    def send_command_toshiba(self, HVAC_Temp=22, mode=ToshibaAC_Mode.HVAC_AUTO.value, off=False, fanmode=ToshibaAC_FanSpd.AUTO.value, typ=AC_Types.TOSHIBA.value ):

        Delay = self.delay_list[typ]
        
        # change these if needed i think they are the same as mitsi
        sender = ir_sender.IrSender(self.gpio_pin, "NEC", dict(
            leading_pulse_duration=Delay.HdrMark,
            leading_gap_duration=Delay.HdrSpace,
            one_pulse_duration=Delay.BitMark,
            one_gap_duration=Delay.OneSpace,
            zero_pulse_duration=Delay.BitMark,
            zero_gap_duration=Delay.ZeroSpace,
            trailing_pulse_duration=Delay.RptMark,
            trailing_gap_duration=Delay.RptSpace), self.log_level)

        # data array is a valid frame, only byte to be changed will be updated.
        data=(ctypes.c_uint8 * 9)()
        data=[0xF2, 0x0D, 0x03, 0xFC, 0x01, 0x00, 0x00, 0x00, 0x00]

        # Byte 7 - Mode
        data[6] = mode;

        # Byte 7 - On / Off
        if off = True:
            data[6] = 0x07                                  # Turn OFF HVAC

        # Byte 6 - Temperature
        if (HVAC_Temp > 30):
            Temp = 30
        elif HVAC_Temp < 17):
            Temp = 17 
        else:
            Temp = HVAC_Temp
        data[5] = Temp - 17<<4

        # Byte 10 - FAN / VANNE
        data[6] |= fanmode

        # Byte 9 - CRC
        for i in range(0,8):
            data[8] ^= data[i] 
        data[8] %= (Constants.MaxMask + 1)
        
        sender.send_data(data, Constants.MaxMask, True, Constants.NbPackets)
        
    def send_command_panasonic(self, state=1, mode=PanasonicAC_Mode.HVAC_AUTO.value, HVAC_Temp=22, fan_mode=PanasonicAC_FanMode.FAN_SPEED_AUTO | PanasonicAC_FanMode.VANNE_AUTO, pro=PanasonicAC_Profile.NORMAL, typ=AC_Types.PANASONIC.value ):

        Delay = self.delay_list[typ]
        
        # change these if needed i think they are the same as mitsi
        sender = ir_sender.IrSender(self.gpio_pin, "NEC", dict(
            leading_pulse_duration=Delay.HdrMark,
            leading_gap_duration=Delay.HdrSpace,
            one_pulse_duration=Delay.BitMark,
            one_gap_duration=Delay.OneSpace,
            zero_pulse_duration=Delay.BitMark,
            zero_gap_duration=Delay.ZeroSpace,
            trailing_pulse_duration=Delay.RptMark,
            trailing_gap_duration=Delay.RptSpace), self.log_level)

        # data array is a valid frame, only byte to be changed will be updated.
        data=(ctypes.c_uint8 * 19)()
        dataconst=(ctypes.c_uint8 * 8)()
        mask = 1
        data = [ 0x02, 0x20, 0xE0, 0x04, 0x00, 0x48, 0x3C, 0x80, 0xAF, 0x00, 0x00, 0x0E, 0xE0, 0x10, 0x00, 0x01, 0x00, 0x06, 0xBE ]
        dataconst = [ 0x02, 0x20, 0xE0, 0x04, 0x00, 0x00, 0x00, 0x06 ]       

        # Byte 6 - On / Off
        if state == 1:
            data[5] = 0x08                                     # Switch HVAC Power
        else:
            data[5] = 0x09                                     # Do not switch HVAC Power

        # Byte 6 - Mode
        data[5] = mode

        # Byte 7 - Temp  
        Temp - 0;
        if (HVAC_Temp > 30): 
            Temp = 30
        elif (HVAC_Temp < 16):
            Temp = 16 
        else: 
            Temp = HVAC_Temp
        data[6] = (Temp - 16) <<1
        data[6] = data[6] | 0x20

        # Byte 9 - FAN / VANNE
        data[8] = fan_mode
        
        # Byte 14 - Profile
        data[13] = pro
        
        # CRC
        for i in range(0,18):
            data[18] += data[i]  
        data[18] %= (Constants.MaxMask + 1)
        
        # frame 1 is dataconst not sent in reverse order (uncomment of you need this)  0x4004072000000060
        # dataconst.reverse()                           -- this is reverse byte order
        for ii, d in enumerate(dataconst):
            dataconst[ii] = reverse_bits(d)[1]          # -- this is reverse bit order
            
        sender.send_data(dataconst, Constants.MaxMask, True, Constants.NbPackets)

        # frame 2 is data not sent in reverse order
        # data.reverse()                               -- this is reverse byte order
        for ii, d in enumerate(data):
            data[ii] = reverse_bits(d)[1]              # -- this is reverse bit order        

        sender.send_data(data, Constants.MaxMask, True, Constants.NbPackets)

    def send_command_mitsiw001cp(self, HVAC_Temp=20, state=1, mode=Mitsi_w001cp_Mode.AUTO.value, fan_mode=Mitsi_w001cp_Fan.SPEED_AUTO.value | Mitsi_w001cp_Fan.VANNE_AUTO.value  , typ=AC_Types.MITSUBISHI.value ):

        Delay = self.delay_list[typ]
        
        # change these if needed i think they are the same as mitsi
        sender = ir_sender.IrSender(self.gpio_pin, "NEC", dict(
            leading_pulse_duration=Delay.HdrMark,
            leading_gap_duration=Delay.HdrSpace,
            one_pulse_duration=Delay.BitMark,
            one_gap_duration=Delay.OneSpace,
            zero_pulse_duration=Delay.BitMark,
            zero_gap_duration=Delay.ZeroSpace,
            trailing_pulse_duration=Delay.RptMark,
            trailing_gap_duration=Delay.RptSpace), self.log_level)

        # data array is a valid frame, only byte to be changed will be updated.
        data=(ctypes.c_uint8 * 17)()
        mask = 1
        data = [ 0x23, 0xCB, 0x26, 0x21, 0x00, 0x40, 0x52, 0x35, 0x04, 0x00, 0x00, 0xBF, 0xAD, 0xCA, 0xFB, 0xFF, 0xFF ]
      
        # Byte 6 - On / Off
        if state == 1:
            data[5] = 0x40                                       # Switch HVAC Power
        else:
            data[5] = 0x0                                        # Do not switch HVAC Power

        # Byte 7 - Temperature / Mode 
        tmpTM = mode
        Temp = 0                                                 # Temperature, support 17~28 in HVAC_HOT mode, 19~30 in HVAC_COLD and HVAC_DRY mode
        if (HVAC_Temp > 30 and tmpTM != Mitsi_w001cp_Mode.HOT.value):
            Temp = 30
        elif (HVAC_Temp > 28 and tmpTM == Mitsi_w001cp_Mode.HOT.value): 
            Temp = 28
        elif (HVAC_Temp < 19 and tmpTM != Mitsi_w001cp_Mode.HOT.value): 
            Temp = 19 
        elif (HVAC_Temp < 17 and tmpTM == Mitsi_w001cp_Mode.HOT.value): 
            Temp = 17 
        else: 
            Temp = HVAC_Temp
        Temp = Temp - 16 
        data[6] = Temp * 16 | tmpTM                               # Temperature bits are the high 4 bits, Mode bits are the low 4 bits.

        data[7] = fan_mode
  
        # Byte 11 - XOR of Byte 5
        data[11] = 0b11111111 ^ data[5]
  
        # Byte 12 - XOR of Byte 6
        data[12] = 0b11111111 ^ data[6]
  
        # Byte 13 - XOR of Byte 7
        data[13] = 0b11111111 ^ data[7]
        
        # CRC
        # data[16] = sum(data[:-1]) % (Constants.MaxMask + 1)

        sender.send_data(data, Constants.MaxMask, True, 1)       # no repeat needed



