# FLIR EtherNet/IP Thermo Camera AX8
#
import pycomm3
import struct

o = pycomm3.CIPDriver.discover()
# Example output
# [{'encap_protocol_version': 1,
#  'ip_address': '192.168.0.180',
#  'vendor': 'FLIR Systems',
#  'product_type': 'Generic Device (keyable)',
#  'product_code': 321,
#  'revision': {'major': 2, 'minor': 40},
#  'status': b'\x00\x00',
#  'serial': '********',
#  'product_name': 'FLIR AX8',
#  'state': 255}]
ip_addr = o[0]['ip_address']

# get temperature
with pycomm3.CIPDriver(ip_addr) as driver:
    driver.open()
    res = driver.generic_message(
        service=pycomm3.Services.get_attribute_single,
        class_code=pycomm3.ClassCode.assembly,
        instance=0x64,
        attribute=3,
        )
    buffer = res[1]
    spot1_temp = struct.unpack_from('<f', buffer, 36)[0] - 273.15
    print(f"{spot1_temp:.1f}: spot1 temp degC")
    spot2_temp = struct.unpack_from('<f', buffer, 56)[0] - 273.15
    print(f"{spot2_temp:.1f}: spot2 temp degC")

# take shot
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x69,
        instance=1,
        attribute=1,
        request_data=bytearray([1]),
        data_type=pycomm3.BOOL
    )
  
# please refer to this document https://support.flir.com/Answers/A1239/EIP.pdf
#
# auto NUC
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x65,                                  # camera control
        instance=1,                                       # auto NUC
        attribute=1,
        request_data=bytearray([1]),                      # enable
        data_type=pycomm3.BOOL
    )

# set distance units
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x64,                                  # system command object
        instance=1,                                       # distance unit
        attribute=1,
        request_data=bytearray("meter".encode('utf-8')),  # meters
        data_type=pycomm3.STRING
    )

# set temp units
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x64,                                  # system command object
        instance=1,                                       # temp unit
        attribute=2,
        request_data=bytearray("Celcius".encode('utf-8')),  # meters
        data_type=pycomm3.STRING
    )

# set pallete
pal = [ "bw.pal", "iron.pal", "rainbox.pal" ]
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x67,                                  # image control object
        instance=1,                                       # pallete
        attribute=1,
        request_data=bytearray(pal[2].encode('utf-8')),   # rainbox
        data_type=pycomm3.STRING
    )
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x67,                                  # image control object
        instance=1,                                       # pallete invert 
        attribute=2,
        request_data=bytearray([0]),                      # off
        data_type=pycomm3.BOOL
    )
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x67,                                  # image control object
        instance=1,                                       # image adjust
        attribute=4,
        request_data=bytearray("Auto".encode('utf-8')),   # auto
        data_type=pycomm3.STRING
    )
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x67,                                  # image control object
        instance=1,                                       # image freeze
        attribute=11,
        request_data=bytearray([0]),                      # off
        data_type=pycomm3.BOOL
    )
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10,                                     # set attribute
        class_code=0x67,                                  # image control object
        instance=1,                                       # image live
        attribute=12,
        request_data=bytearray([1]),                      # on
        data_type=pycomm3.BOOL
    )

# get atmos temperature
with pycomm3.CIPDriver(ip_addr) as driver:
    driver.open()
    res = driver.generic_message(
        service=pycomm3.Services.get_attribute_single,
        class_code=0x6B,                                # instance
        instance=1,                                     # atmospheric temp
        attribute=1,
        )
    buffer = res[1]                                    # result should be 4 bytes long
    atmos_temp = struct.unpack_from('<f', buffer, 0)[0] - 273.15
    print(f"{atmos_temp:.1f}: atmos_temp degC")

# get emissivity
with pycomm3.CIPDriver(ip_addr) as driver:
    driver.open()
    res = driver.generic_message(
        service=pycomm3.Services.get_attribute_single,
        class_code=0x6B,                                # instance
        instance=1,                                     # emissivity
        attribute=2,
        )
    buffer = res[1]                                     # result should be 4 bytes long
    r = struct.unpack_from('<f', buffer, 0)[0]
    print(f"{r:.5f}: emissivity")

# get distance
with pycomm3.CIPDriver(ip_addr) as driver:
    driver.open()
    res = driver.generic_message(
        service=pycomm3.Services.get_attribute_single,
        class_code=0x6B,                                # instance
        instance=1,                                     # distance
        attribute=3,
        )
    buffer = res[1]                                     # result should be 4 bytes long
    r = struct.unpack_from('<f', buffer, 0)[0]
    print(f"{r:.1f}: distance meters")

# get reflected temperature
with pycomm3.CIPDriver(ip_addr) as driver:
    driver.open()
    res = driver.generic_message(
        service=pycomm3.Services.get_attribute_single,
        class_code=0x6B,                                # instance
        instance=1,                                     # reflected temp
        attribute=4,
        )
    buffer = res[1]                                    # result should be 4 bytes long
    ref_temp = struct.unpack_from('<f', buffer, 0)[0] - 273.15
    print(f"{ref_temp:.1f}: atmos_temp degC")

# get rel humidity
with pycomm3.CIPDriver(ip_addr) as driver:
    driver.open()
    res = driver.generic_message(
        service=pycomm3.Services.get_attribute_single,
        class_code=0x6B,                                # instance
        instance=1,                                     # humidity
        attribute=5,
        )
    buffer = res[1]                                    # result should be 4 bytes long
    r = struct.unpack_from('<f', buffer, 0)[0] 
    print(f"{r:.1f}: relative humidity")

# drive GPIO
REF_HIGH=200.0
if ref_temp > REF_HIGH:
   set_state = [1]
else:
   set_state = [0]
with pycomm3.CIPDriver(ip_addr) as driver:
    res = driver.generic_message(
        service=0x10, 
        class_code=0x6F,                                # pysical i/o
        instance=1,                                     # DO1
        attribute=101,
        request_data=bytearray(set_state),              # off
        data_type=pycomm3.BOOL
        )

# connection to Omron S8VK-X12024A-EIP power monitor
# from the discover command [{'encap_protocol_version': 1, 'ip_address': '192.168.xxx.xxx', 'vendor': 'Omron Corporation', 
# 'product_type': 'UNKNOWN', 'product_code': 1680, 'revision': {'major': 1, 'minor': 2}, 'status': b'\x00\x00', 'serial': 'xxxxxx', 
# 'product_name': 'S8VK-X12024A-EIP', 'state': 255}]
#
# therefore use hard coded ip_addr
PSU_IP="192.168.10.1"
# Since the S8VK-X series is a power supply unit, data such as output voltage and current can be obtained via EtherNet/IP
#
with pycomm3.CIPDriver(PSU_IP) as driver:
    driver.open()
    res = driver.generic_message(
        service=pycomm3.Services.get_attributes_all,
        class_code=0x0372,
        instance=0x01)
    buffer = res[1]
    print(buffer)

# unpack the reply message
uint1 = struct.unpack_from('<H', buffer, 0)[0]
uint2 = struct.unpack_from('<H', buffer, 2)[0]
uint3 = struct.unpack_from('<H', buffer, 4)[0]
uint4 = struct.unpack_from('<H', buffer, 6)[0]
uint5 = struct.unpack_from('<H', buffer, 8)[0]
uint6 = struct.unpack_from('<H', buffer, 10)[0]
dword1 = struct.unpack_from('<I', buffer, 12)[0]
dword2 = struct.unpack_from('<I', buffer, 16)[0]

data = {}
data['status_bits'] = bin(uint1)
data['voltage'] = uint2 / 100
data['current1'] = uint3 / 100
data['current2'] = uint4 / 100
data['lifetime1'] = uint5 / 10
data['lifetime2'] = uint6 / 10
data['total_operating_time'] = dword1
data['runtime'] = dword2

# print results
for key, value in data.items():
    if key == 'voltage':
        print(f"  {key}: {value:.2f} V")
    elif key == 'current1':
        print(f"  {key}: {value:.2f} A")
    elif key == 'current2':
        print(f"  {key}: {value:.2f} A")
    elif key == 'lifetime1':
        print(f"  {key}: {value:.1f} 年")
    elif key == 'lifetime2':
        print(f"  {key}: {value:.1f} %")
    elif key == 'total_operating_time':
        print(f"  {key}: {value}")
    elif key == 'runtime':
        print(f"  {key}: {value}")
    else:
        print(f"  {key}: {value}")