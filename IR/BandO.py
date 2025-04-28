#
# Bang & Olufsen beo IR library interface
#
from enum import Enum

class IR_Freq(Enum):   
    BandO_KHZ      = 455                                                # some B&O are also 38 KHz (std)
    SONY_KHZ       = 40
    BOSEWAVE_KHZ   = 38
    DENON_KHZ      = 38
    JVC_KHZ        = 38
    LG_KHZ         = 38
    NEC_KHZ        = 38
    SAMSUNG_KHZ    = 38
    KASEIKYO_KHZ   = 37                                                 # panasonic
    RC5_RC6_KHZ    = 36

# Defining the Bang & Olufsen timing marks
class BEO4Pw(Enum):
    BEO_CARR = 200                                                      # carrier pulse length
    BEO_ZERO = 3125                                  
    BEO_SAME = 6250                                  
    BEO_ONE = 9375                                   
    BEO_STOP = 12500 
    BEO_START = 15625                             
    BEO_EOT = 0                                                         # last symbol has no duration
	
# beo4 addresses
class BEO4Addr(Enum):
    SOURCE_VIDEO = 0x00
    SOURCE_AUDIO = 0x01
    SOURCE_VIDEOTAPE = 0x05
    SOURCE_ALL = 0x0F
    SOURCE_SPDEMO = 0x1D
    SOURCE_LIGHT = 0x1B

# source codes
class BEO4Src(Enum):
    VIDEO        = 0x00
    AUDIO        = 0x01
    VTAPE        = 0x05
    ALL          = 0x0F
    SPDEMO       = 0x1D
    LIGHT        = 0x1B

class BEO4CmdCodes(Enum):
    NUM_0        = 0x00
    NUM_1        = 0x01
    NUM_2        = 0x02
    NUM_3        = 0x03
    NUM_4        = 0x04
    NUM_5        = 0x05
    NUM_6        = 0x06
    NUM_7        = 0x07
    NUM_8        = 0x08
    NUM_9        = 0x09
    CLEAR        = 0x0A
    STORE        = 0x0B
    STANDBY      = 0x0C
    MUTE         = 0x0D
    INDEX        = 0x0E
    UP           = 0x1E
    DOWN         = 0x1F
    TUNE         = 0x20
    CLOCK        = 0x28
    FORMAT       = 0x2A
    LEFT         = 0x32
    RETURN       = 0x33
    RIGHT        = 0x34
    GO           = 0x35
    STOP         = 0x36
    RECORD       = 0x37
    SELECT       = 0x3F
    SPEAKER      = 0x44
    PICTURE      = 0x45
    TURN         = 0x46                   # list->mono
    LOUDNESS     = 0x48
    BASS         = 0x4D
    TREBLE       = 0x4E
    BALANCE      = 0x4F
    LIST         = 0x58
    MENU         = 0x5C
    VOL_UP       = 0x60
    VOL_DOWN     = 0x64
    LEFT_REPEAT  = 0x70
    RIGHT_REPEAT = 0x71
    UP_REPEAT    = 0x72
    DOWN_REPEAT  = 0x73
    GO_REPEAT    = 0x75
    GREEN_REPEAT = 0x76
    YELLOW_REPEAT= 0x77
    BLUE_REPEAT  = 0x78
    RED_REPEAT   = 0x79
    EXIT         = 0x7F
    TV           = 0x80
    RADIO        = 0x81
    VIDEO_AUX    = 0x82
    AUDIO_AUX    = 0x83
    VTAPE        = 0x85
    DVD          = 0x86
    CAMCORD      = 0x87
    TEXT         = 0x88
    SP_DEMO      = 0x89
    SAT          = 0x8A
    PC           = 0x8B
    DOOR_CAM     = 0x8D
    ATAPE        = 0x91
    CD           = 0x92
    PHONO        = 0x93
    ATAPE2       = 0x94
    CD2          = 0x97
    LIGHT        = 0x9B
    AV           = 0xBF
    YELLOW       = 0xD4
    GREEN        = 0xD5
    BLUE         = 0xD8
    RED          = 0xD9
    STAND        = 0xf7