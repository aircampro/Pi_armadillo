# read the BLE devices and check to see if it matches the id specified
#
import asyncio
from bleak import discover

EMBER_MANUFACTURER_CODE = 0xFFFF  # 企業コード。これがキーにあれば、その企業の製品

async def main():
    devices = await discover()
    for d in devices:
        if EMBER_MANUFACTURER_CODE in d.metadata.get('manufacturer_data', {}):  # If the manufacturer matches the one specified
            user_input = input('Device {!r} found. Is this ember mug? Y/N [Y]: '.format(d.name)) or 'y'
            if user_input.lower() == 'y':
                ember = d                                                       # set the device name
                break
    else:
        print('Ember mug is not found. Exiting...')                             # we couldn't find the requested device
        return
    print(ember)
    print(ember.metadata)

if __name__ == '__main__':
    asyncio.run(main())

"""
output:
    Device 'Ember Ceramic Mug' found. Is this ember mug? Y/N [Y]: y
    FF:FF:FF:FF:FF:FF: Ember Ceramic Mug
    {'uuids': [], 'manufacturer_data': {65535: b'\x81'}}
"""