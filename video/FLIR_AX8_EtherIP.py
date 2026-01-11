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