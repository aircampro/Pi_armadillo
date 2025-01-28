#
# library of common crc generation using the crcmod python library
# before installing crcmod set PYTHONUTF8 = "1"
# 
import crcmod
from intelhex import IntelHex

# crcmod.predefined module offers the following predefined algorithms https://crcmod.sourceforge.net/crcmod.predefined.html
#
PRE_DEF_ALGO = [ "crc-8", "crc-8-darc", "crc-8-i-code", \	
"crc-8-itu", "crc-8-maxim", "crc-8-rohc", "crc-8-wcdma", \	
"crc-16", "crc-16-buypass", "crc-16-dds-110", "crc-16-dect", \	
"crc-16-dnp", "crc-16-en-13757", "crc-16-genibus", \	
"crc-16-maxim", "crc-16-mcrf4xx", "crc-16-riello" \	
"crc-16-t10-dif", "crc-16-teledisk", "crc-16-usb" \	
"x-25", "xmodem", "modbus", "kermit", "crc-ccitt-false", \ 
"crc-aug-ccitt", "crc-24", "crc-24-flexray-a", "crc-24-flexray-b", \	
"crc-32", "crc-32-bzip2", "crc-32c", "crc-32d", "crc-32-mpeg", \	
"posix", "crc-32q", "jamcrc", "xfer", "crc-64", "crc-64-we", "crc-64-jones" ]	

def calculate_crc64_file(file_path):
    # CRC-64  ECMA-182 
    crc64_func = crcmod.mkCrcFun(0x142F0E1EBA9EA3693, initCrc=0, xorOut=0xFFFFFFFFFFFFFFFF)
    with open(file_path, 'rb') as file:
        data = file.read()
    crc_value = crc64_func(data)
    return crc_value

def calculate_crc64(data):
    # CRC-64  ECMA-182 
    crc64_func = crcmod.mkCrcFun(0x142F0E1EBA9EA3693, initCrc=0, xorOut=0xFFFFFFFFFFFFFFFF)
    return crc64_func(data)

def calculate_crc32(data):
    # CRC-32 
    crc32 = crcmod.predefined.Crc('crc-32')
    crc32.update(data)
    calculated = crc32.digest()
    return calculated

def calculate_crc16(data):
    # CRC-16 
    crc16 = crcmod.predefined.Crc('crc-16')
    crc16.update(data)
    calculated = crc16.digest()
    return calculated
    
def calculate_crc16_dnp(data):
    # CRC-16-DNP 
    crc16_func = crcmod.mkCrcFun(0x13D65, 0xFFFF, True, 0xFFFF)
    return crc16_func(data)

def calculate_crc16_vov(data):
    # VÖV 04.05.1" CRC - 
    crc16_func = crcmod.mkCrcFun(0x16f63, rev=False, initCrc=0xFFFF, xorOut=0xFFFF)
    return crc16_func(data)
   
def calculate_crc16_vov_bin(msg="00000000000000000000000010000000"):
    # VÖV 04.05.1" CRC - 
    crc16_func = crcmod.mkCrcFun(0x16f63, rev=False, initCrc=0xFFFF, xorOut=0xFFFF)
    #interpret string as binary, convert to bytes, calculate CRC
    crc_result = crc16_func(int(msg, 2).to_bytes((len(msg) + 7) // 8, 'big'))
    print(bin(crc_result))
    print(hex(crc_result))
    return crc_result

def calculate_crc_from_param(data, a=0x16f63, rev=False, initCrc=0xFFFF, xorOut=0xFFFF):
    # default VÖV 04.05.1" CRC - 
    crc16_func = crcmod.mkCrcFun(a, rev=rev, initCrc=initCrc, xorOut=xorOut)
    return crc16_func(data)
    
def calculate_crc16_modbus(data):
    #  CRC-Modbus 
    crc16_func = crcmod.predefined.mkPredefinedCrcFun("modbus")
    return crc16_func(data)
	
def calculate_crc_from_predefined(data, func_nm="modbus"):
    # default is CRC-Modbus 
    crc16_func = crcmod.predefined.mkPredefinedCrcFun(func_nm)
    return crc16_func(data)

def calculate_crc_from_predefined2(data_path, algo_num=4, is_file=False):
    # default is crc-8-maxim 
    crc16_func = crcmod.predefined.mkPredefinedCrcFun(PRE_DEF_ALGO[algo_num])
    if is_file == True:
        with open(data_path, 'rb') as file:
            data = file.read()
    else:
        data = data_path
    return crc16_func(data)
    
def calculate_crc_from_predefined_crc(data, func_nm="modbus"):
    # default is CRC-Modbus 
    crc16 = crcmod.predefined.mkPredefinedCrc(func_nm)
    crc16.update(data)
    return crc16.crcValue
	
def test_crc64_with_file(fl='your_file.txt'):
    file_path = fl                                
    crc_value = calculate_crc64_file(file_path)
    print(file_path + ": " + f"CRC-64 Checksum: {crc_value:016x}")
    
def load_hex(self, path, crc_id=29):
    print("using ",PRE_DEF_ALGO[crc_id]," for crc calc")
    ih = IntelHex()
    ih.loadhex(path)
    flash_hex_data = []

    for start, end in ih.segments():
        data = ih.tobinstr(start=start, end=end - 1)
        crc32 = crcmod.predefined.Crc(PRE_DEF_ALGO[crc_id])
        crc32.update(data)
        calculated = crc32.digest()
        self.flash_hex_data.append({
            "start_address": start,
            "end_address": end - 1,
            "size": end - start,
            "data": data,
            "crc32": calculated
        })
    return flash_hex_data

class CRC16:
    ARC          = lambda: CRC16(0x8005, 0x0000,  True,  True, 0x0000)
    AUG_CCITT    = lambda: CRC16(0x1021, 0x1d0f, False, False, 0x0000)
    BUYPASS      = lambda: CRC16(0x8005, 0x0000, False, False, 0x0000)
    CCITT_FALSE  = lambda: CRC16(0x1021, 0xffff, False, False, 0x0000)
    CDMA2000     = lambda: CRC16(0xc867, 0xffff, False, False, 0x0000)
    CMS          = lambda: CRC16(0x8005, 0xffff, False, False, 0x0000)
    DDS_110      = lambda: CRC16(0x8005, 0x800d, False, False, 0x0000)
    DECT_R       = lambda: CRC16(0x0589, 0x0000, False, False, 0x0001)
    DECT_X       = lambda: CRC16(0x0589, 0x0000, False, False, 0x0000)
    DNP          = lambda: CRC16(0x3d65, 0x0000,  True,  True, 0xffff)
    EN_13757     = lambda: CRC16(0x3d65, 0x0000, False, False, 0xffff)
    GENIBUS      = lambda: CRC16(0x1021, 0xffff, False, False, 0xffff)
    GSM          = lambda: CRC16(0x1021, 0x0000, False, False, 0xffff)
    LJ1200       = lambda: CRC16(0x6f63, 0x0000, False, False, 0x0000)
    MAXIM        = lambda: CRC16(0x8005, 0x0000,  True,  True, 0xffff)
    MCRF4XX      = lambda: CRC16(0x1021, 0xffff,  True,  True, 0x0000)
    OPENSAFETY_A = lambda: CRC16(0x5935, 0x0000, False, False, 0x0000)
    OPENSAFETY_B = lambda: CRC16(0x755b, 0x0000, False, False, 0x0000)
    PROFIBUS     = lambda: CRC16(0x1dcf, 0xffff, False, False, 0xffff)
    RIELLO       = lambda: CRC16(0x1021, 0x554d,  True,  True, 0x0000)
    T10_DIF      = lambda: CRC16(0x8bb7, 0x0000, False, False, 0x0000)
    TELEDISK     = lambda: CRC16(0xa097, 0x0000, False, False, 0x0000)
    TMS37157     = lambda: CRC16(0x1021, 0x3791,  True,  True, 0x0000)
    USB          = lambda: CRC16(0x8005, 0xffff,  True,  True, 0xffff)
    CRC_A        = lambda: CRC16(0x1021, 0x6363,  True,  True, 0x0000)
    KERMIT       = lambda: CRC16(0x1021, 0x0000,  True,  True, 0x0000)
    MODBUS       = lambda: CRC16(0x8005, 0xffff,  True,  True, 0x0000)
    NRSC_5       = lambda: CRC16(0x080b, 0xffff,  True,  True, 0x0000)
    X_25         = lambda: CRC16(0x1021, 0xffff,  True,  True, 0xffff)
    XMODEM       = lambda: CRC16(0x1021, 0x0000, False, False, 0x0000)

    def __init__(self, poly, init, refin, refout, xorout):
        self.poly = poly
        self.init = init
        self.refin = refin
        self.refout = refout
        self.xorout = xorout
        if self.refin:
            self.init = self.__reflect(init, 16)

    def __reflect(self, x, bits):
        r = 0
        for i in range(bits):
            r = (r << 1) | ((x >> i) & 1)
        return r

    def update(self, data):
        for x in data:
            self.init ^= (self.__reflect(x, 8) if self.refin else x) << 8
            for _ in range(8):
                if self.init & 0x8000:
                    self.init = ((self.init << 1) ^ self.poly) & 0xffff
                else:
                    self.init = (self.init << 1) & 0xffff

    def digest(self, data=b''):
        self.update(data)
        x = self.init
        if self.refout:
            x = self.__reflect(x, 16)
        return x ^ self.xorout
