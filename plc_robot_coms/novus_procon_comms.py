#!/usr/bin/env python
# using -- pip install pymodbus
#
# ===================================================================================================================================================
#
# simple read and write to Novus i/o set up to read pt100 thermocouples over serial - this is a multifunctional input on modbus rtu serial
# simple read and write to Procon TCP/IP PLC - this is a webserver and PLC on modbus tcp
#
# ===================================================================================================================================================
import sys

# =================================================
#           modbus serial client which is novus
# =================================================
from pymodbus.client.sync import ModbusSerialClient as ModbusClient_Novus

#---------- this is for the NOVUS DigiRail-2A universal Analog Input Module ----------#
NOVUS_SER_PRT="/dev/ttyS0"

# =================================================
#           modbus tcp client which is procon
# =================================================
from pymodbus.client.sync import ModbusTcpClient as ModbusClient_Procon

# =================================================
#           modbus mapping for the devices
# =================================================
import modbus_novus_procon

#---------- this is for the Procon PL101 PLC Webserver or Logic Interface ----------#
IP_ADDRESS_OF_PROCON_PLC = modbus_novus_procon.PROCON_TCP16DI_DEF_IP
PROCON_TCP_PORT=5020

# if set to 1 exit if re-connection unsucessful otherwise set to 0
DEF_CON_EXIT = 1

# =================================================
#           modbus functions
# =================================================

#---------- modbus parameters ----------#
# address   : modbus register to start with
# values    : value to set or list of values
# unit      : the slave number when on serial
# count     : number of registers to perform action upon 

def write_holding_reg(client, reg=1, value=0, method="tcp", unit=1):
    print("Write to a holding register and read back")
    if method == "tcp":
        rq = client.write_register(reg, value)
    else :
        rq = client.write_register(reg, value, unit=unit)	
    print(rq)
    return rq
	
def read_holding_reg(client, reg=1, method="tcp", unit=1):
    print("Read holding register")
    if method == "tcp":
        rr = client.read_holding_registers(address=reg, count=1)
    else :		
        rr = client.read_holding_registers(address=reg, count=1, unit=unit)
    print(rr.registers)
    return rr.registers
	
def write_holding_regs(client, reg=1, values=[0,2,3], method="tcp", unit=1):
    print("Write to multiple holding registers and read back")
    if method == "tcp":
        request = client.write_registers(address=reg, values=values1)
    else :
        request = client.write_registers(address=reg, values=values1, unit= unit)
    print(request)
    return request
	
def read_holding_regs(client, reg=1, method="tcp", cc=3, unit=1):
    print("Read holding registers")
    if method == "tcp":
        rr = client.read_holding_registers(address=reg, count=cc)
    else :		
        rr = client.read_holding_registers(address=reg, count=cc, unit=unit)
    print(rr.registers)	
    return rr.registers
	
def read_input_regs(client, reg=1, method="tcp", cc=3, unit=1):
    print("Read input registers")
    if method == "tcp":
        rr = client.read_input_registers(reg, cc)
    else :		
        rr = client.read_input_registers(address=reg, count=cc, unit=unit)
    print(rr.registers)		
    return rr.registers

def read_disc_inputs(client, reg=0, num=3, method="tcp", unit=1):
    print("Read discrete inputs")
    if method == "tcp":   
        rr = client.read_discrete_inputs(req, num)
    else:
        rr = client.read_discrete_inputs(req, num, unit)    
    print(rr.bits)
    return rr.bits
    
def write_coil(client, reg=0, v=True, method="tcp", unit=1):
    print("Write to a coil and read back")
    if method == "tcp":   
        rq = client.write_coil(reg, v)
    else:
        rq = client.write_coil(reg, v, unit) 
    print(rq)
    return rq

def write_coils(client, reg=0, v=[True]*9, method="tcp", unit=1):
    print("Write to multiple coils and read back")
    if method == "tcp":   
        rq = client.write_coils(reg, v)
    else:
        rq = client.write_coils(reg, v, unit) 
    print(rq)
    return rq

def read_coils(client, reg=0, cc=9, method="tcp", unit=1):
    print("Reading multiple/Single coils and read back")
    if method == "tcp":   
        req = client.read_coils(reg, cc)
    else:
        req = client.read_coils(reg, cc, unit) 
    print(req.bits)
    return req.bits


if __name__ == '__main__': 

    # set up the novus on serial
    client_novus = ModbusClient_Novus(method = "rtu", port=NOVUS_SER_PRT, stopbits = 1, bytesize = 8, parity = 'E', baudrate= 115200, timeout= 1)
    client_novus.connect()
    mthd = "ser"
    
    # set up the novus module -- write the channel type
    # NOVS_DR2A_TYPE1 = NDR2A_Pt100
    def_addr_novus_d2a = modbus_novus_procon.NOVS_DR2A_DEF_ADDR 
    values = [modbus_novus_procon.NDR2A_Pt100, modbus_novus_procon.NDR2A_Pt100]
   
    # writing as single registers
    w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_TYPE1, value=values[0], method=mthd, unit=def_addr_novus_d2a)
    w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_TYPE2, value=values[1], method=mthd, unit=def_addr_novus_d2a)

    # might work as multiple write as below - please try it       
    # w = write_holding_regs(client_novus, NOVS_DR2A_TYPE1, values, mthd, def_addr_novus_d2a)  
    values = [modbus_novus_procon.NDR2A_loPt100, modbus_novus_procon.NDR2A_ulPt100, modbus_novus_procon.NDR2A_loPt100, modbus_novus_procon.NDR2A_ulPt100]
    
    # writing as single registers
    w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH1_LOW, value=values[0], method=mthd, unit=def_addr_novus_d2a)
    w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH1_HIGH, value=values[1], method=mthd, unit=def_addr_novus_d2a)
    w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH2_LOW, value=values[2], method=mthd, unit=def_addr_novus_d2a)
    w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH2_HIGH, value=values[3], method=mthd, unit=def_addr_novus_d2a)

    # might work as multiple write as below - please try it    
    # w = write_holding_regs(client_novus, NOVS_DR2A_CH1_LOW, values, mthd, def_addr_novus_d2a)

    # set up the procon on tcp/ip    
    client_pro = ModbusClient_Procon(IP_ADDRESS_OF_PROCON_PLC, port=PROCON_TCP_PORT)
    client_pro.connect()    
    mthd_pro = "tcp"

    try: 
        while True:
            try:        
                while True:
                    # get the pt100 values from the novus module unscaled and scaled for each of the 2 channels         
                    results = read_input_regs(client_novus, modbus_novus_procon.NOVS_DR2A_PV_CH1, mthd, 4, def_addr_novus_d2a)
                    print(results) 
                    # write those values to the Battery Backed RAM in the PLC for frequent storage        
                    w = write_holding_regs(client_pro, modbus_novus_procon.PROCON_PL101_BBRAM_START, results, mthd_pro) 
                    # read them back         
                    r = read_holding_regs(client_pro, modbus_novus_procon.PROCON_PL101_BBRAM_START, mthd_pro, 4) 
            except (ValueError, Exception) as e:
                print(e)
                steps_in_error = 0
                # attempt re-connection once but if already closed just ignore that error
                while (steps_in_error < 2):
                    if (steps_in_error == 0):
                        try:
                            # close the connections
                            client_novus.close()
                            client_pro.close() 
                            steps_in_error = 1
                        except (ValueError, Exception) as e:
                            # tolerate this error and go into the next stage step which is to try to re-establish communication
                            print(e)   
                            steps_in_error = 1
                    elif (steps_in_error == 1):   
                        try:                
                            # try re-open connections
                            client_novus = ModbusClient_Novus(method = "rtu", port=NOVUS_SER_PRT,stopbits = 1, bytesize = 8, parity = 'E', baudrate= 115200, timeout= 1)
                            client_novus.connect() 
                            client_pro = ModbusClient_Procon(IP_ADDRESS_OF_PROCON_PLC, port=PROCON_TCP_PORT)
                            client_pro.connect() 
                            steps_in_error = 2
                        except (ValueError, Exception) as e:
                            # error should then drop out loop and close connections
                            print(e) 
                            if DEF_CON_EXIT == 1:
                                try:
                                    client_novus.close()
                                except (ValueError, Exception) as e:
                                    print(e) 
                                try:                                
                                    client_pro.close() 
                                except (ValueError, Exception) as e:
                                    print(e)
                                sys.exit(-1)                                
    except (KeyboardInterrupt) as k: 
        print(k) 
        try:        
            client_novus.close()
        except (ValueError, Exception) as e:
            print(e)
        try:
            client_pro.close()
        except (ValueError, Exception) as e:
            print(e)