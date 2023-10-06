# read the tempewrature from the ember mug can be any BLE device this version has an exception handler for BLE errors
#
import asyncio
from bleak import BleakClient
from bleak.exc import BleakError
from functools import wraps
from warnings import warn

EMBER_MANUFACTURER_CODE = 0xFFFF

def ble_error_catch(func):
    if asyncio.iscoroutinefunction(func):
        @wraps(func)
        async def async_inner(self, *args, **kwargs):
            try:
                return await func(self, *args, **kwargs)
            except (RuntimeError, BleakError):
                warn(f"'{func.__name__}' was failed because of Bluetooth error.")
        return async_inner
    else:
        @wraps(func)
        def inner(self, *args, **kwargs):
            try:
                return func(self, *args, **kwargs)
            except (RuntimeError, BleakError):
                warn(f"'{func.__name__}' was failed because of Bluetooth error.")
        return inner

class BatteryState:
    def __init__(self, battery_charge: int, is_charging: bool):
        self.battery_charge = battery_charge
        self.is_charging = is_charging
    def __repr__(self):
        return 'BatteryState(battery_charge={!r}, is_charging={!r})'.format(self.battery_charge, self.is_charging)

def parse_battery(value: bytearray) -> BatteryState:
    battery = value[0]
    is_charging = bool(value[1])
    return BatteryState(battery, is_charging)
    
# temperature decoder
def decode_temperature(value: bytearray) -> float:
    return int.from_bytes(value, byteorder='little') / 100                          # bytearray of little endian 32-bit float values

# temperature encoder
def encode_temperature(value: float) -> bytearray:
    return bytearray(round(value*100).to_bytes(length=2, byteorder='little'))

@ble_error_catch
def fetch_current_temp() -> float:
    value = await client.read_gatt_char('FC540002-236C-4C94-8FA9-944A3E5353FA')  # Current temperature 002 read
    value = decode_temperature(value)
    print(f"mug temperature {value}")
    return value

@ble_error_catch
def fetch_set_temp() -> float:
    value = await client.read_gatt_char('FC540003-236C-4C94-8FA9-944A3E5353FA')  # Set temperature 003 read
    value = decode_temperature(value)
    print(f"set temperature {value}")
    value_spt = float(input('Please enter the setpoint '))
    value = await client.write_gatt_char('FC540003-236C-4C94-8FA9-944A3E5353FA',encode_temperature(value_spt))  # Set temperature 003 write
    value = decode_temperature(value) 
    print(f"set temperature {value}")
    return value

@ble_error_catch
def fetch_battery_data() -> BatteryState:
    value = await client.read_gatt_char('FC540007-236C-4C94-8FA9-944A3E5353FA')  # Battery 007 read
    c_b = BatteryState
    c_b = parse_battery(value)
    print(f"battery {c_b.battery_charge} charger = {c_b.is_charging}")
    return c_b

@ble_error_catch
def fetch_state_data() -> str:
    value = await client.read_gatt_char('FC540008-236C-4C94-8FA9-944A3E5353FA')  # State 008 read
    # not sure on the decode of it ? value = decode_temperature(value)
    if value == 0x00:  
        state = "accept"
    elif value == 0x01:
        state = "empty"
    elif value == 0x02:  
        state = "poured"
    elif value == 0x03:
        state = "off"
    elif value == 0x04:
        state = "cooling"
    elif value == 0x05:
        state = "heating"
    elif value == 0x06:
        state = "keeping"
    elif value == 0x07:  
        state = "finish drinking"
    else:
        state = "unknown"
    return state
    
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
        ct = fetch_current_temp()
        st = fetch_set_temp()
        cb = fetch_battery_data()
        sta = fetch_state_data()
        await client.disconnect()                                                    # disconnect the central from the periferal ember mug

# run the main thread
if __name__ == '__main__':
    asyncio.run(main())
