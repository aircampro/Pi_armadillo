#!/usr/bin/env python3
# -*- coding: utf-8 -+-
# example of a client (master) requesting data from the modbus server (slave) using pymodbus
#
from pymodbus_client import ModbusClient                        # this is the class we wrote for modbus clients

def main():
    with ModbusClient() as client:
    
        # Read holding registers starting at address 0 with a count of 10
        holding_registers = client.read_holding_registers(0, 10)
        if holding_registers is not None:
            print(f"Holding registers: {holding_registers}")
        else:
            print("Error reading holding registers")

        # Read input registers starting at address 0 with a count of 10
        input_registers = client.read_input_registers(0, 10)
        if input_registers is not None:
            print(f"Input registers: {input_registers}")
        else:
            print("Error reading input registers")

        # Read coils starting at address 0 with a count of 10
        coils = client.read_coils(0, 10)
        if coils is not None:
            print(f"Coils: {coils}")
        else:
            print("Error reading coils")

        # Write coil starting at address 0 
        coil = client.write_coil(0, True)
        if coil is not None:
            print(f"Coils: {coils}")
        else:
            print("Error writing coil")

        # Write coils starting at address 3 
        list_v = [True, False, True]
        coils = client.write_coils(3, list_v)
        if coils is not None:
            print(f"Coils: {coils}")
        else:
            print("Error reading coils")

        # write holding registers starting at address 0 to 70
        holding_registers = client.write_holding_register(0, 70)
        if holding_registers is not None:
            print(f"Holding registers: {holding_registers}")
        else:
            print("Error reading holding register")

        # write holding registers starting at address 10 to hr_list
        hr_list = [ 12, 67, 76, 89 ]
        holding_registers = client.write_holding_registers(10, hr_list)
        if holding_registers is not None:
            print(f"Holding registers: {holding_registers}")
        else:
            print("Error reading holding register")
            
        # Read discrete inputs starting at address 0 with a count of 10
        discrete_inputs = client.read_discrete_inputs(0, 10)
        if discrete_inputs is not None:
            print(f"Discrete inputs: {discrete_inputs}")
        else:
            print("Error reading discrete inputs")
            
if __name__ == '__main__':
    main()