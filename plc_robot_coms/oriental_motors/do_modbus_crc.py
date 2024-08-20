# modbus CRC and hex conversions
#
def get_modbus_crc(command, endi=0):
    # Set first CRC register value to FFFFFFh
    crc_register = 0xFFFF
    for data_byte in command:
        # XOR the CRC register and the data byte
        tmp = crc_register ^ data_byte
        # Store the number of shifts
        shift_num = 0
        # Repeat until 8 shifts
        while(shift_num < 8): 
            if(tmp&1 == 1):           # if overflow is 1
                tmp = tmp >> 1
                shift_num += 1
                tmp = 0xA001 ^ tmp
            else: 
                tmp = tmp >> 1
                shift_num += 1
        # Set calculation result to crc_register
        crc_register = tmp
    # Convert calculation result to bytes type
    if endi == 1:
        a = (crc_register&0xFF00)>>8
        b = (crc_register&0x00FF)<<8 
        crc_register = a | b     
    crc = crc_register.to_bytes(2, 'big')

    print("crc value = ",crc_register) 
    print(crc)  
    return crc

# alternative calculation
def get_modbus_crc2(command, endi=0):
    crc_register = 0xFFFF
    for data_byte in command:
        crc_register ^= data_byte
        for _ in range(8):
            overflow = crc_register & 1 == 1
            crc_register >>= 1
            if overflow:
                crc_register ^= 0xA001
    if endi == 1:
        a = (crc_register&0xFF00)>>8
        b = (crc_register&0x00FF)<<8 
        crc_register = a | b                 
    crc = crc_register.to_bytes(2, 'big')

    print("crc value = ",crc_register) 
    print(crc)  
    return crc

# another alternative
import crc16
def get_modbus_crc2(command, endi=0):
    a = crc16.CRC16.MODBUS()                    # choose the modbus crc from the crc16.py library
    a.update(command)
    crc_register = a.digest()
    if endi == 1:
        a = (crc_register&0xFF00)>>8
        b = (crc_register&0x00FF)<<8 
        crc_register = a | b      
    crc = crc_register.to_bytes(2, 'big')

    print("crc value = ",crc_register) 
    print(crc)  
    return crc

# convert an iteger to bytes to add to the bytestream    
def number_to_bytes( num ):
    return num.to_bytes(2, 'big')
    
def number_to_hexbytes( num ):
	n1 = (num & 0xFF00) >> 8
	n2 = (num & 0x00FF) 
    return str(hex(n1)).split('x')[1].encode('utf-8'), str(hex(n2)).split('x')[1].encode('utf-8')