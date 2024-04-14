// toshiba teli wrapper class for python
from ctypes import *
from enum import IntEnum
from struct import calcsize
from sys import platform
from types import SimpleNamespace
import numpy as np

MAX_INFO_STR = 64

class CAM_API_STATUS(IntEnum):
    # Operation completed successfully.
    SUCCESS                  = 0x00000000
    # API have not been made ready by Sys_Initialize.
    NOT_INITIALIZED          = 0x00000001
    # API is already in ready state.
    ALREADY_INITIALIZED      = 0x00000002
    # Camera is not found.
    NOT_FOUND                = 0x00000003
    # Specified handle is already opened.
    ALREADY_OPENED           = 0x00000004
    # Specified event is already registered.
    ALREADY_ACTIVATED        = 0x00000005
    # The specified camera index was not valid.
    INVALID_CAMERA_INDEX     = 0x00000006
    # The specified camera handle was not valid.
    INVALID_CAMERA_HANDLE    = 0x00000007
    # The specified node handle was not valid.
    INVALID_NODE_HANDLE      = 0x00000008
    # The specified stream handle was not valid.
    INVALID_STREAM_HANDLE    = 0x00000009
    # The specified buffer handle was not valid.
    INVALID_REQUEST_HANDLE    = 0x0000000A
    # The specified event handle was not valid.
    INVALID_EVENT_HANDLE     = 0x0000000B
    # The specified image handle was not valid.
    INVALID_IMAGE_HANDLE     = 0x0000000C
    # The specified parameter was not valid.
    INVALID_PARAMETER        = 0x0000000D
    # Buffer size specified by user was too small to complete the request.
    BUFFER_TOO_SMALL         = 0x0000000E
    # Request cannot be completed because insufficient memory resource.
    NO_MEMORY                = 0x0000000F
    # Memory location specified by user was not valid.
    MEMORY_NO_ACCESS         = 0x00000010
    # Feature is not implemented in the camera or API.
    NOT_IMPLEMENTED          = 0x00000011
    # Timeout expired.
    TIMEOUT                  = 0x00000012
    # The opened camera may be lost. You should re-enum the camera to confirm whether it exist.
    CAMERA_NOT_RESPONDING    = 0x00000013
    # Stream or event "get request" is failed because there is no complete queue.
    EMPTY_COMPLETE_QUEUE     = 0x00000014
    # Information is not yet ready.
    NOT_READY                = 0x00000015
    # Failed to set the access mode.
    ACCESS_MODE_SET_ERR      = 0x00000016
    # Controller caused an I/O error.
    IO_DEVICE_ERROR          = 0x00000020
    # Passed parameters have logical error(s).
    LOGICAL_PARAM_ERROR      = 0x00000021 
    # Failed to load the XML.
    XML_LOAD_ERR             = 0x00000101
    # GenICam error occurred.
    GENICAM_ERR              = 0x00000102
    # Failed to load the dll file.
    DLL_LOAD_ERR             = 0x00000103
    # Insufficient system resources exist to complete the requested service.
    NO_SYSTEM_RESOURCES      = 0x000005AA
    # Attempt to access a not existing register address.
    INVALID_ADDRESS          = 0x00000801
    # Attempt to write to a read only register.
    WRITE_PROTECT            = 0x00000802
    # Access registers with an address which is not aligned according to the underlying technology.
    BAD_ALIGNMENT            = 0x00000803
    # Read a non-readable or write a non-writable register address.
    ACCESS_DENIED            = 0x00000804
    # Camera is currently busy.
    BUSY                     = 0x00000805
    # Not readable.
    NOT_READABLE             = 0x00000806
    # Not writable.
    NOT_WRITABLE             = 0x00000807
    # Function or the camera of the register is not currently available.
    NOT_AVAILABLE            = 0x00000808
    # Verify error occurred.
    VERIFY_ERR               = 0x00000809
    # User request had not be able to complete while user specified timeoout limit.
    REQUEST_TIMEOUT          = 0x00001001
    # Stream resend request timeoout.
    RESEND_TIMEOUT           = 0x00001002
    # Stream resend data receive timeoout.
    RESPONSE_TIMEOUT         = 0x00001003
    # Transferred frame data was larger than user specified max payload size.
    BUFFER_FULL              = 0x00001004
    # Actual received payload size was not equal to the size notified by Trailer.
    UNEXPECTED_BUFFER_SIZE   = 0x00001005
    # Exceeded the MAX number of packets that is realized by Leader or Trailer.
    UNEXPECTED_NUMBER        = 0x00001006
    # Any error was notified by stream or event status on transaction header.
    PACKET_STATUS_ERROR      = 0x00001007
    # Packet resend command is not supported by the camera.
    RESEND_NOT_IMPLEMENTED   = 0x00001008
    # The requested packet is not available anymore.
    PACKET_UNAVAILABLE       = 0x00001009
    # Frame data is terminated with next Leader or Trailer's BlockId is different from Leader's.
    # Some packets may be lost.
    MISSING_PACKETS          = 0x0000100A
    # Requests were flushed by user request.
    FLUSH_REQUESTED          = 0x0000100B
    # The loss of packet exceeded the specified value.
    TOO_MANY_PACKET_MISSING  = 0x0000100C
    # Requests were flushed because power state was chaned into save mode.
    FLUSHED_BY_D0EXIT        = 0x0000100D
    # Requests were flushed by camera remove event.
    FLUSHED_BY_CAMERA_REMOVE = 0x0000100E
    # Failed to load the driver.
    DRIVER_LOAD_ERR          = 0x0000100F
    # Mapping user buffer to system-space virtual address is failed.
    # It may be caused by low system resources.
    MAPPING_ERROR            = 0x00001010
    # Failed to open file.
    FILE_OPEN_ERROR          = 0x00002001
    # Failed to write data to file.
    FILE_WRITE_ERROR         = 0x00002002
    # Failed to read data from file.
    FILE_READ_ERROR          = 0x00002003
    # Failed to find file.
    FILE_NOT_FOUND           = 0x00002004
    # At least one command parameter of CCD or SCD is invalid or out of range. 
    INVALID_PARAMETER_FROM_CAM = 0x00008002
    # The value written to the SI streaming size registers is not aligned
    # to Payload Size Alignment value of the SI Info register.
    SI_PAYLOAD_SIZE_NOT_ALIGNED = 0x0000A003
    # Some data in the block has been discarded.
    DATA_DISCARDED           = 0x0000A100
    # The Camera cannot send all data because the data does not fit within the programmed SIRM register settings.
    DATA_OVERRUN             = 0x0000A101
    # Unspecified error occurred.
    UNSUCCESSFUL             = 0xFFFFFFFF

class CAM_TYPE(IntEnum):
    Unknown = 0x00
    U3V     = 0x01   # USB3 Vision Camera
    GEV     = 0x02   # GigE Vision Camera
    All     = 0xFFFF

class CAM_ACCESS_MODE(IntEnum):
    Open      = 0 # Open access
    Control   = 1  # Control access
    Exclusive = 3 # Exclusive access

class CAM_ACQ_MODE_TYPE(IntEnum):
    Continuous      = 8   # Continuous
    MultiFrame      = 9   # MultiFrame
    ImageBufferRead = 10  # Camera Image Buffer Mode
    SingleFrame     = 109 # SingleFrame

class CAM_PIXEL_FORMAT(IntEnum):
    Unknown    = 0x00000000
                                #  Grouping  Padding
                                #   Pixels    Bits
    Mono8      = 0x01080001,    #     1        0       8-bit pixel in one byte
    Mono10     = 0x01100003,    #     1        6       10-bit pixel padded to 16 bits
    Mono12     = 0x01100005,    #     1        4       12-bit pixel padded to 16 bits
    Mono16     = 0x01100007,    #     1        0       16-bit pixel in two bytes
    BayerGR8   = 0x01080008,    #     1        0       8-bit pixel in one byte
    BayerGR10  = 0x0110000C,    #     1        6       10-bit pixel padded to 16 bits
    BayerGR12  = 0x01100010,    #     1        4       12-bit pixel padded to 16 bits
    BayerRG8   = 0x01080009,    #     1        0       8-bit pixel in one byte
    BayerRG10  = 0x0110000D,    #     1        6       10-bit pixel padded to 16 bits
    BayerRG12  = 0x01100011,    #     1        4       12-bit pixel padded to 16 bits
    BayerGB8   = 0x0108000A,    #     1        0       8-bit pixel in one byte
    BayerGB10  = 0x0110000E,    #     1        6       10-bit pixel padded to 16 bits
    BayerGB12  = 0x01100012,    #     1        4       12-bit pixel padded to 16 bits
    BayerBG8   = 0x0108000B,    #     1        0       8-bit pixel in one byte
    BayerBG10  = 0x0110000F,    #     1        6       10-bit pixel padded to 16 bits
    BayerBG12  = 0x01100013,    #     1        4       12-bit pixel padded to 16 bits
    RGB8       = 0x02180014,    #     1        0       8-bit color component in one byte
    BGR8       = 0x02180015,    #     1        0       8-bit color component in one byte
    BGR10      = 0x02300019,    #     1        6       10-bit color component padded to 16 bits
    BGR12      = 0x0230001B,    #     1        4       12-bit color component padded to 16 bits
    YUV411_8   = 0x020C001E,    #     1        0       8-bit color component in one byte, UYYVYY component sequence
    YUV422_8   = 0x0210001F,    #     1        0       8-bit color component in one byte, UYVY component sequence
    YUV8       = 0x02180020,    #     1        0       8-bit color component in one byte, UYV component sequence

class DST_FORMAT(IntEnum):
    BGRA32   = 0 # BGRA 32bit format (  0)
    BGR24    = 1 # BGR  24bit format (  1)
    BGR24PAD = 2 # BGR  24bit format (  2)

class U3V_CAMERA_INFO(Structure):
    _fields_ = [
        ('familyName', c_char * MAX_INFO_STR),
        ('deviceVersion', c_char * MAX_INFO_STR),
        ('manufacturerInfo', c_char * MAX_INFO_STR),
        ('adapterVendorId', c_uint32),
        ('adapterDeviceId', c_uint32),
        ('adapterDfltMaxPacketSize', c_uint32),
    ]

class GEV_CAMERA_INFO(Structure):
    _fields_ = [
        ('displayName', c_char * 512),
        ('macAddress', c_uint8 * 6),
        ('supportIP_LLA', c_int8),
        ('supportIP_DHCP', c_int8),
        ('supportIP_Persistent', c_int8),
        ('currentIP_LLA', c_int8),
        ('currentIP_DHCP', c_int8),
        ('currentIP_Persistent', c_int8),
        ('ipAddress', c_uint8 * 4),
        ('subnet', c_uint8 * 4),
        ('gateway', c_uint8 * 4),
        ('adapterMACAddress', c_uint8 * 6),
        ('adapterIPAddress', c_uint8 * 4),
        ('adapterSubnet', c_uint8 * 4),
        ('adapterGateway', c_uint8 * 4),
        ('adapterDisplayName', c_char * 1024)
    ]

class CAM_INFO(Structure):
    _fields_ = [
        ('camType', c_int32),
        ('manufacturer', c_char * MAX_INFO_STR),
        ('modelName', c_char * MAX_INFO_STR),
        ('serialNumber', c_char * MAX_INFO_STR),
        ('userDefinedName', c_char * MAX_INFO_STR),
        ('u3vCamInfo', U3V_CAMERA_INFO),
        ('gevCamInfo', GEV_CAMERA_INFO)
    ]

class CAM_IMAGE_INFO(Structure):
    _fields_ = [
        ('timestamp', c_uint64),
        ('pixelFormat', c_int32), # CAM_PIXEL_FORMAT
        ('sizeX', c_uint32),
        ('sizeY', c_uint32),
        ('offsetX', c_uint32),
        ('offsetY', c_uint32),
        ('paddingX', c_uint32),
        ('blockId', c_uint64),
        ('buf', c_void_p),
        ('size', c_uint32),
        ('imageId', c_uint64),
        ('status', c_int32) # CAM_API_STATUS
    ]

class TeliError(Exception):
    def __init__(self, status, message = None):
        self.status = CAM_API_STATUS(status)
        if (message):
            self.message = message
        else:
            self.message = self.status.name
        super().__init__(self.message)

def _checkLib():
    if (not _api):
        raise TeliError(CAM_API_STATUS.NOT_INITIALIZED)

def _checkStatus(status):
    if (status):
        raise TeliError(status)

def _toCstring(s):
    if (isinstance(s, bytes)):
        return s
    else:
        return s.encode()

def _mac2string(v):
    return ':'.join(['{:02x}'.format(v[i]) for i in range(0, 6)])

def _ip2string(v):
    return '.'.join(['{:d}'.format(v[i]) for i in range(0, 4)])

_api = None
_utl = None

def initialize(camType: CAM_TYPE = CAM_TYPE.All, apiPath: str = None, urlPath: str = None):
    global _api
    global _utl
    b64 = calcsize("P") == 8
    if (platform == 'win32'):                                                           
        p = 'c:\\Program Files\\TOSHIBA TELI\\TeliCamSDK\\TeliCamApi\\bin\\x'                  # this is where you installed the c++ sdk on your machine if windows
        if (b64):
            n = p + '64\TeliCamApi64.dll'
            u = p + '64\TeliCamUtl64.dll'
        else:
            n = p + '86\TeliCamApi.dll'
            u = p + '86\TeliCamUtl.dll'
    else:
        p = '/opt/TeliCamSDK/lib/'                                                             # this is where you installed the c++ sdk in linux (either x86 or arm)
        if (b64):
            n = p + 'libTeliCamApi_64.so'
            u = p + 'libTeliCamUtl_64.so'
        else:
            n = p + 'libTeliCamApi.so'
            u = p + 'libTeliCamUtl.so'
    if (not apiPath):
        apiPath = n
    if (not urlPath):
        urlPath = u
    _api = CDLL(n)                                                                            # this invokes the c functions for TeliCamApi
    _utl = CDLL(u)                                                                            # this invokes the c functions for TeliCamUtl
    # CAMAPI Sys_Initialize(CAM_TYPE eCamType = CAM_TYPE_ALL)
    _checkStatus(_api.Sys_Initialize(c_int32(camType)))
    # IMGAPI PrepareLUT()
    _checkStatus(_utl.PrepareLUT())

def terminate():
    _checkLib()
    # CAMAPI Sys_Terminate(void)
    _api.Sys_Terminate()

def getNumOfCameras():
    _checkLib()
    num = c_uint32()
    # CAMAPI Sys_GetNumOfCameras(uint32_t *puiNum)
    _checkStatus(_api.Sys_GetNumOfCameras(byref(num)))
    return num.value

def getCameraInfo(cam):
    _checkLib()
    info = CAM_INFO()
    # CAMAPI Cam_GetInformation(CAM_HANDLE hCam, uint32_t uiCamIdx, CAM_INFO *psCamInfo)
    if (isinstance(cam, int)):
        _checkStatus(_api.Cam_GetInformation(None, c_uint32(cam), byref(info)))
    else:
        _checkStatus(_api.Cam_GetInformation(cam.hCam, c_uint32(-1), byref(info)))
    # Convert to proper python structure
    return SimpleNamespace(
        camType = CAM_TYPE(info.camType),
        manufacturer = info.manufacturer.decode(),
        modelName = info.modelName.decode(),
        serialNumber = info.serialNumber.decode(),
        userDefinedName = info.userDefinedName.decode(),
        u3vCamInfo = SimpleNamespace(
            familyName = info.u3vCamInfo.familyName.decode(),
            deviceVersion = info.u3vCamInfo.deviceVersion.decode(),
            manufacturerInfo = info.u3vCamInfo.manufacturerInfo.decode(),
            adapterVendorId = info.u3vCamInfo.adapterVendorId,
            adapterDeviceId = info.u3vCamInfo.adapterDeviceId,
            adapterDfltMaxPacketSize = info.u3vCamInfo.adapterDfltMaxPacketSize
        ),
        gevCamInfo = SimpleNamespace(
            displayName = info.gevCamInfo.displayName.decode(),
            macAddress = _mac2string(info.gevCamInfo.macAddress),
            supportIP_LLA = bool(info.gevCamInfo.supportIP_LLA),
            supportIP_DHCP = bool(info.gevCamInfo.supportIP_DHCP),
            supportIP_Persistent = bool(info.gevCamInfo.supportIP_Persistent),
            currentIP_LLA = bool(info.gevCamInfo.currentIP_LLA),
            currentIP_DHCP = bool(info.gevCamInfo.currentIP_DHCP),
            currentIP_Persistent = bool(info.gevCamInfo.currentIP_Persistent),
            ipAddress = _ip2string(info.gevCamInfo.ipAddress),
            subnet = _ip2string(info.gevCamInfo.subnet),
            gateway = _ip2string(info.gevCamInfo.gateway),
            adapterMACAddress = _mac2string(info.gevCamInfo.adapterMACAddress),
            adapterIPAddress = _ip2string(info.gevCamInfo.adapterIPAddress),
            adapterSubnet = _ip2string(info.gevCamInfo.adapterSubnet),
            adapterGateway = _ip2string(info.gevCamInfo.adapterGateway),
            adapterDisplayName = info.gevCamInfo.adapterDisplayName.decode()
        )
    )

class Camera:
    def __init__(self, cameraIndex: int, accessMode: CAM_ACCESS_MODE = CAM_ACCESS_MODE.Control):
        self.hCam = None
        self.hSignal = None
        self.hStream = None
        _checkLib()
        hCam = c_uint64()
        # CAMAPI Cam_Open(uint32_t uiCamIdx, CAM_HANDLE *phCam, SIGNAL_HANDLE hRmv = NULL, bool8_t bUseGenICam = true, void *pvXml = NULL, CAM_ACCESS_MODE eAccessMode = CAM_ACCESS_MODE_CONTROL)
        _checkStatus(_api.Cam_Open(c_uint32(cameraIndex), byref(hCam), None, True, None, c_int32(accessMode)))
        self.hCam = hCam

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self):
        if (self.hStream):
            #  CAMAPI Strm_Abort(CAM_STRM_HANDLE hStrm)
            _api.Strm_Abort(self.hStream)
            # CAMAPI Strm_Close(CAM_STRM_HANDLE hStrm)
            _api.Strm_Close(self.hStream)
            self.hStream = None
        if (self.hSignal):
            # CAMAPI Sys_CloseSignal(SIGNAL_HANDLE hHandle)
            _api.Sys_CloseSignal(self.hSignal)
            self.hSignal = None
        if (self.hCam):
            # CAMAPI Cam_Close(CAM_HANDLE hCam)
            _api.Cam_Close(self.hCam)
            self.hCam = None

    def getInfo(self):
        return getCameraInfo(self)

    def getIntValue(self, featureName: str, verify: bool = False, ignoreCache: bool = False):
        # CAMAPI GenApi_GetIntValue(CAM_HANDLE hCam, const char *pszFeatureName, int64_t *pllValue, bool8_t bVerify = false, bool8_t bIgnoreCache = false);
        v = c_int64()
        _checkStatus(_api.GenApi_GetIntValue(self.hCam, _toCstring(featureName), byref(v), c_bool(verify), c_bool(ignoreCache)))
        return v.value

    def setIntValue(self, featureName: str, value: int, verify: bool = True):
        # CAMAPI GenApi_SetIntValue(CAM_HANDLE hCam, const char *pszFeatureName, int64_t llValue, bool8_t bVerify = true);
        _checkStatus(_api.GenApi_SetIntValue(self.hCam, _toCstring(featureName), c_int64(value), c_bool(verify)))

    def getFloatValue(self, featureName: str, verify: bool = False, ignoreCache: bool = False):
        # CAMAPI GenApi_GetFloatValue(CAM_HANDLE hCam, const char *pszFeatureName, float64_t *pdValue, bool8_t bVerify = false, bool8_t bIgnoreCache = false)
        v = c_double()
        _checkStatus(_api.GenApi_GetFloatValue(self.hCam, _toCstring(featureName), byref(v), c_bool(verify), c_bool(ignoreCache)))
        return v.value

    def setFloatValue(self, featureName: str, value: float, verify: bool = True):
        # CAMAPI GenApi_SetFloatValue(CAM_HANDLE hCam, const char *pszFeatureName, float64_t dValue, bool8_t bVerify = true);
        _checkStatus(_api.GenApi_SetFloatValue(self.hCam, _toCstring(featureName), c_double(value), c_bool(verify)))

    def getStringValue(self, featureName: str, verify: bool = False, ignoreCache: bool = False):
        # CAMAPI GenApi_GetStrValue(CAM_HANDLE hCam, const char *pszFeatureName, char *pszBuf, uint32_t *puiSize, bool8_t bVerify = false, bool8_t bIgnoreCache = false);
        sz = c_uint32()
        _checkStatus(_api.GenApi_GetStrValue(self.hCam, _toCstring(featureName), None, byref(sz), c_bool(verify), c_bool(ignoreCache)))
        s = create_string_buffer(sz.value)
        _checkStatus(_api.GenApi_GetStrValue(self.hCam, _toCstring(featureName), s, byref(sz), c_bool(verify), c_bool(ignoreCache)))
        return s.value.decode()

    def setStringValue(self, featureName: str, value: str, verify: bool = True):
        # CAMAPI GenApi_SetStrValue(CAM_HANDLE hCam, const char *pszFeatureName, const char *pszBuf, bool8_t bVerify = true);
        _checkStatus(_api.GenApi_SetStrValue(self.hCam, _toCstring(featureName), _toCstring(value), c_bool(verify)))

    def getBoolValue(self, featureName: str, verify: bool = False, ignoreCache: bool = False):
        # CAMAPI GenApi_GetBoolValue(CAM_HANDLE hCam, const char *pszFeatureName, bool8_t *pbValue, bool8_t bVerify = false, bool8_t bIgnoreCache = false);
        v = c_bool()
        _checkStatus(_api.GenApi_GetBoolValue(self.hCam, _toCstring(featureName), byref(v), c_bool(verify), c_bool(ignoreCache)))
        return v.value

    def setBoolValue(self, featureName: str, value: bool, verify: bool = True):
        # CAMAPI GenApi_SetBoolValue(CAM_HANDLE hCam, const char *pszFeatureName, bool8_t bValue, bool8_t bVerify = true);
        _checkStatus(_api.GenApi_SetBoolValue(self.hCam, _toCstring(featureName), c_bool(value), c_bool(verify)))

    def getEnumIntValue(self, featureName: str, verify: bool = False, ignoreCache: bool = False):
        # CAMAPI GenApi_GetEnumIntValue(CAM_HANDLE hCam, const char *pszFeatureName, int64_t *pllValue, bool8_t bVerify = false, bool8_t bIgnoreCache = false);
        v = c_int64()
        _checkStatus(_api.GenApi_GetEnumIntValue(self.hCam, _toCstring(featureName), byref(v), c_bool(verify), c_bool(ignoreCache)))
        return v.value

    def setEnumIntValue(self, featureName: str, value: int, verify: bool = True):
        # CAMAPI GenApi_SetEnumIntValue(CAM_HANDLE hCam, const char *pszFeatureName, int64_t llValue, bool8_t bVerify = true);
        _checkStatus(_api.GenApi_SetEnumIntValue(self.hCam, _toCstring(featureName), c_int64(value), c_bool(verify)))

    def getEnumStringValue(self, featureName: str, verify: bool = False, ignoreCache: bool = False):
        # CAMAPI GenApi_GetEnumStrValue(CAM_HANDLE hCam, const char *pszFeatureName, char *pszBuf, uint32_t *puiSize, bool8_t bVerify = false, bool8_t bIgnoreCache = false);
        sz = c_uint32()
        _checkStatus(_api.GenApi_GetEnumStrValue(self.hCam, _toCstring(featureName), None, byref(sz), c_bool(verify), c_bool(ignoreCache)))
        s = create_string_buffer(sz.value)
        _checkStatus(_api.GenApi_GetEnumStrValue(self.hCam, _toCstring(featureName), s, byref(sz), c_bool(verify), c_bool(ignoreCache)))
        return s.value.decode()

    def setEnumStringValue(self, featureName: str, value: str, verify: bool = True):
        # CAMAPI GenApi_SetEnumStrValue(CAM_HANDLE hCam, const char *pszFeatureName, const char *pszBuf, bool8_t bVerify = true);
        _checkStatus(_api.GenApi_SetEnumStrValue(self.hCam, _toCstring(featureName), _toCstring(value), c_bool(verify)))

    def commandExecute(self, featureName: str, verify: bool = True):
        # CAMAPI GenApi_CmdExecute(CAM_HANDLE hCam, const char *pszFeatureName, bool8_t bVerify = true);
        _checkStatus(_api.GenApi_CmdExecute(self.hCam, _toCstring(featureName), c_bool(verify)))

    def isCommandDone(self, featureName: str, verify: bool = True):
        done = c_bool()
        # CAMAPI GenApi_GetCmdIsDone(CAM_HANDLE hCam, const char *pszFeatureName, bool8_t *pbDone, bool8_t bVerify = false);
        _checkStatus(_api.GenApi_GetCmdIsDone(self.hCam, _toCstring(featureName), byref(done), c_bool(verify)))
        return done.value

    def startStream(self, aquisitionMode: CAM_ACQ_MODE_TYPE = CAM_ACQ_MODE_TYPE.Continuous):
        # CAMAPI Sys_CreateSignal(SIGNAL_HANDLE *phHandle)
        hSignal = c_uint64()
        _checkStatus(_api.Sys_CreateSignal(byref(hSignal)))
        self.hSignal = hSignal
        hStream = c_uint64();
        self.maxSize = c_uint32()
        # CAMAPI Strm_OpenSimple(CAM_HANDLE hCam, CAM_STRM_HANDLE *phStrm, uint32_t *puiMaxPayloadSize, SIGNAL_HANDLE hCmpEvt = NULL, uint32_t uiApiBufferCount = DEFAULT_API_BUFFER_CNT, uint32_t uiMaxPacketSize = 0)
        _checkStatus(_api.Strm_OpenSimple(self.hCam, byref(hStream), byref(self.maxSize), self.hSignal, c_uint32(8), c_uint32(0)))
        self.hStream = hStream
        # CAMAPI Strm_Start(CAM_STRM_HANDLE hStrm, CAM_ACQ_MODE_TYPE eAcqMode = CAM_ACQ_MODE_CONTINUOUS)
        _checkStatus(_api.Strm_Start(self.hStream, c_int32(aquisitionMode)))

    def stopStream(self):
        # CAMAPI Strm_Stop(CAM_STRM_HANDLE hStrm)
        _api.Strm_Stop(self.hStream)
        # CAMAPI Strm_Close(CAM_STRM_HANDLE hStrm)
        _api.Strm_Close(self.hStream)
        # CAMAPI Sys_CloseSignal(SIGNAL_HANDLE hHandle)
        _api.Sys_CloseSignal(self.hSignal)
        hStream = None
        hSignal = None

    def waitForImage(self, timeoutMs: int):
        # CAMAPI Sys_WaitForSignal(SIGNAL_HANDLE hHandle, uint32_t uiMilliseconds)
        status = _api.Sys_WaitForSignal(self.hSignal, c_uint32(timeoutMs))
        if (status == CAM_API_STATUS.TIMEOUT):
            return False
        elif (status == CAM_API_STATUS.SUCCESS):
            return True
        else:
            _checkStatus(status)

    def getCurrentImage(self):
        buf = np.empty(self.maxSize.value, dtype='uint8')
        size = c_uint32(buf.size)
        info = CAM_IMAGE_INFO()
        # CAMAPI Strm_ReadCurrentImage(CAM_STRM_HANDLE hStrm, void *pvBuf, uint32_t *puiSize, CAM_IMAGE_INFO *psImageInfo)
        _checkStatus(_api.Strm_ReadCurrentImage(self.hStream, buf.ctypes, byref(size), byref(info)))
        # Convert to RGB
        # IMGAPI ConvImage(DST_FORMAT eDstFormat, CAM_PIXEL_FORMAT uiSrcPixelFormat, bool bBayerConversion, void *pvDst, void *pvSrc, uint32_t uiWidth, uint32_t uiHeight)
        bgr = np.empty(info.sizeX * info.sizeY * 3)
        _checkStatus(_utl.ConvImage(c_int32(DST_FORMAT.BGR24), c_int32(info.pixelFormat), True, bgr.ctypes, buf.ctypes, c_uint32(info.sizeX), c_uint32(info.sizeY)))
        # Return as numpy multi-dim array
        return np.ndarray((info.sizeY, info.sizeX, 3), dtype=np.uint8, buffer=bgr)
		