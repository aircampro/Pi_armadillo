# TCS3472 light-to-digital converter on i2c
#		
import smbus
import time
bus = smbus.SMBus(1)
# DEVICE    ADDRESS PACKAGE − LEADS INTERFACE DESCRIPTION ORDERING NUMBER
# TCS34721† 0x39    FN−6 I2C Vbus = VDD Interface TCS34721FN
# TCS34723† 0x39    FN−6 I2C Vbus = 1.8 V Interface TCS34723FN
# TCS34725  0x29    FN−6 I2C Vbus = VDD Interface TCS34725FN
# TCS34727  0x29    FN−6 I2C Vbus = 1.8 V Interface TCS34727FN
addr = 0x39                                                         # e.g. TCS34721/3
gain = 0                                                            # no gain isd being applied to the outputs
while 1:
        bus.write_byte_data(addr , 0x80, 0x03)                      # PON=1 power on RGBC=enable=2 == 0x03
        bus.write_byte_data(addr , 0x81, 0xD5)                      # timing 101 ms
        bus.write_byte_data(addr , 0x82, 0xAB)                      # wait time 204 ms    
        if gain == 0:        
            bus.write_byte_data(addr , 0x8F, 0x00)                  # no gain 
        elif gain == 4:
            bus.write_byte_data(addr , 0x8F, 0x01)                  # 4x gain 
        elif gain == 16:
            bus.write_byte_data(addr , 0x8F, 0x02)                  # 16x gain 
        elif gain == 60:
            bus.write_byte_data(addr , 0x8F, 0x03)                  # 60x gain             
        time.sleep(0.5)
        data_status = bus.read_i2c_block_data(addr , 0x13, 2)
        data_rlow = bus.read_i2c_block_data(addr , 0x16, 2)
        data_rhigh = bus.read_i2c_block_data(addr , 0x17, 2)
        data_glow = bus.read_i2c_block_data(addr , 0x18, 2)
        data_ghigh = bus.read_i2c_block_data(addr , 0x19, 2)
        data_blow = bus.read_i2c_block_data(addr , 0x1A, 2)
        data_bhigh = bus.read_i2c_block_data(addr , 0x1B, 2)

        print("status : ", data_status)
        r = (data_rhigh<<8) | data_rlow
        g = (data_ghigh<<8) | data_glow
        b = (data_bhigh<<8) | data_blow
        print("red : %d green : %d blue : %d " % (r,g,b))        
        time.sleep(1)