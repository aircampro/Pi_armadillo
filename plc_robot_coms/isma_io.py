# https://icdocs.ismacontrolli.com/multiprotocol-io-modbus/V1.12/inputs-and-outputs
# https://icdocs.ismacontrolli.com/multiprotocol-io-modbus/V1.12/technical-specification
#
#
# please refer tp above to use various temp input modules etc
#
# here i show to pwm module configuration
#
# Holding Registers (4x)
# 40121-40126 120-125 ao write
# 40144-40150 143-149 start-up valur @ boot or reset
# 40168-40173 167-172 mode (as below)
#
# modes 
Voltage_out_0_10_V = 0
PWM_1_Hz = 1
PWM_10_Hz = 2
PWM_100_Hz = 3
PWM_0_1_Hz = 4
PWM_0_01_Hz = 5

from pymodbus.client.sync import ModbusTcpClient as ModbusClient_isma

IP_ADDRESS_OF_ISMA_IO = "10.0.2.1"
ISMA_TCP_PORT=5020

def conect_isma():
    client = ModbusClient_isma(IP_ADDRESS_OF_ISMA_IO, port=ISMA_TCP_PORT)
    client.connect() 
    return client

def write_holding_regs(client, reg=1, values=[0,2,3], method="tcp", unit=1):
    print("Write to multiple holding registers and read back")
    if method == "tcp":
        request = client.write_registers(address=reg, values=values1)
    else :
        request = client.write_registers(address=reg, values=values1, unit= unit)
    print(request)
    return request

def write_holding_reg(client, reg=1, value=0, method="tcp", unit=1):
    print("Write to a holding register and read back")
    if method == "tcp":
        rq = client.write_register(reg, value)
    else :
        rq = client.write_register(reg, value, unit=unit)	
    print(rq)
    return rq