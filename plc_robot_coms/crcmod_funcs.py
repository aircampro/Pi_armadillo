#
# library of common crc generation using the crcmod python library
# before installing crcmod set PYTHONUTF8 = "1"
#
import crcmod
from intelhex import IntelHex

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

def calculate_crc_from_param(data, a=0x16f63, rev=False, initCrc=0xFFFF, xorOut=0xFFFF):
    # default VÖV 04.05.1" CRC - 
    crc16_func = crcmod.mkCrcFun(a, rev=rev, initCrc=initCrc, xorOut=xorOut)
    return crc16_func(data)
    
def calculate_crc16_vov_bin(msg="00000000000000000000000010000000"):
    # VÖV 04.05.1" CRC - 
    crc16_func = crcmod.mkCrcFun(0x16f63, rev=False, initCrc=0xFFFF, xorOut=0xFFFF)
    #interpret string as binary, convert to bytes, calculate CRC
    crc_result = crc16_func(int(msg, 2).to_bytes((len(msg) + 7) // 8, 'big'))
    print(bin(crc_result))
    print(hex(crc_result))
    return crc_result

def calculate_crc16_modbus(data):
    #  CRC-Modbus 
    crc16_func = crcmod.predefined.mkPredefinedCrcFun("modbus")
    return crc16_func(data)
	
def calculate_crc_from_predefined(data, func_nm="modbus"):
    # default is CRC-Modbus 
    crc16_func = crcmod.predefined.mkPredefinedCrcFun(func_nm)
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
    
def load_hex(self, path):
    ih = IntelHex()
    ih.loadhex(path)
    flash_hex_data = []

    for start, end in ih.segments():
        data = ih.tobinstr(start=start, end=end - 1)
        crc32 = crcmod.predefined.Crc('crc-32')
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
