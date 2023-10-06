# read the tempewrature from the ember mug can be any BLE device
#
import asyncio
from bleak import BleakClient

EMBER_MANUFACTURER_CODE = 0xFFFF

# temperature decoder
def decode_temperature(value: bytearray) -> float:
    return int.from_bytes(value, byteorder='little') / 100  # bytearray of little endian 32-bit float values

# temperature encoder
def encode_temperature(value: float) -> bytearray:
    return bytearray(round(value*100).to_bytes(length=2, byteorder='little'))

# define the main thread    
async def main():
    devices = await discover()
    for d in devices:
        if EMBER_MANUFACTURER_CODE in d.metadata.get('manufacturer_data', {}):
            user_input = input('Device {!r} found. Is this ember mug? Y/N [Y]: '.format(d.name)) or 'y'
            if user_input.lower() == 'y':
                ember = d
                break
    else:
        print('Ember mug is not found. Exiting...')
        return

    #-#-#-# Now get and set the requested data #-#-#-#
    async with BleakClient(ember.address) as client:
        value = await client.read_gatt_char('FC540002-236C-4C94-8FA9-944A3E5353FA')  # Current temperature 002 read
        value = decode_temperature(value)
        print(f"mug temperature {value}")
        value = await client.read_gatt_char('FC540003-236C-4C94-8FA9-944A3E5353FA')  # Set temperature 003 read
        value = decode_temperature(value)
        print(f"set temperature {value}")
        value_spt = float(input('Please enter the setpoint '))
        value = await client.write_gatt_char('FC540003-236C-4C94-8FA9-944A3E5353FA',encode_temperature(value_spt))  # Set temperature 003 write
        value = decode_temperature(value)
        print(f"set temperature {value}")
        value = await client.read_gatt_char('FC540007-236C-4C94-8FA9-944A3E5353FA')  # Battery 007 read
        # not sure on the decode of it ? value = decode_temperature(value)
        print(f"battery {value}")
        value = await client.read_gatt_char('FC540014-236C-4C94-8FA9-944A3E5353FA')  # LED 014 read
        # not sure on the decode of it ? 
        print(f"LED {value}")
        await client.disconnect()                                                    # disconnect the central from the periferal ember mug

# run the main thread
if __name__ == '__main__':
    asyncio.run(main())
"""
output:
    20.0
"""