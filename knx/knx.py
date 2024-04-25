#!/usr/bin/env python3
# -*- coding: utf-8 -+-
""" Example for controlling lights, fans and reading sensors on knx protcol
    taken from examples shown here https://github.com/XKNX/xknx/blob/main/examples """
import asyncio

# light
from xknx import XKNX
from xknx.devices import Light
# fan
from xknx.devices import Fan
# colour lighting
from xknx.remote_value import RemoteValueColorRGBW
# proximity sensor
from xknx.devices import BinarySensor
# temperature sensor
from xknx.devices import Sensor
# weather device
from xknx.devices import Weather
# scene light
from xknx.devices import Scene
# switch (device turn on telly)
from xknx.devices import Switch

from Crypto.Random import get_random_bytes

import sys

async def main(ramp_iterval=1,fan_ramp=1):
    """Connect to KNX/IP bus, slowly dimm on light, set it off again afterwards."""
    xknx = XKNX()
    await xknx.start()

    # -------------- light -------------------------
    light = Light(
        xknx,
        name="TestLight2",
        group_address_switch="1/0/14",
        group_address_brightness="1/0/15",
    )

    // slowly dim this light at each of these settings per second
    for i in [0, 31, 63, 95, 127, 159, 191, 223, 255]:
        await light.set_brightness(i)
        await asyncio.sleep(ramp_iterval)

    await light.set_off()

    # ------------- fan ------------------------------
    fan = Fan(
        xknx,
        name="TestFan",
        group_address_switch="1/0/12",
        group_address_speed="1/0/11",
        max_step=3,
    )

    # Turn on the fan
    await fan.turn_on()

    # Set fan speed to different levels
    for speed in [0, 33, 66, 100]:
        await fan.set_speed(speed)
        await asyncio.sleep(1)

    # Turn off the fan
    await fan.turn_off()

    # set rgb ligting
    rgbw = RemoteValueColorRGBW(
        xknx,
        group_address="1/1/40",
        group_address_state="1/1/41",
        device_name="RGBWLight",
    )

    await rgbw.set([255, 255, 255, 0, 15])  # cold-white
    await asyncio.sleep(1)
    await rgbw.set([0, 0, 0, 255, 15])  # warm-white
    await asyncio.sleep(1)
    await rgbw.set([0, 0, 0, 0, 15])  # off
    await asyncio.sleep(1)

    await rgbw.set([255, 0, 0, 0])  # red
    await asyncio.sleep(1)
    await rgbw.set([0, 255, 0, 0])  # green
    await asyncio.sleep(1)
    await rgbw.set([0, 0, 255, 0])  # blue
    await asyncio.sleep(1)
    await rgbw.set([0, 0, 0, 0, 15])  # off
    await asyncio.sleep(1)

    await rgbw.set([255, 255, 0, 0, 15])
    await asyncio.sleep(1)
    await rgbw.set([0, 255, 255, 0, 15])
    await asyncio.sleep(1)
    await rgbw.set([255, 0, 255, 0, 15])
    await asyncio.sleep(1)
    await rgbw.set([0, 0, 0, 0, 15])  # off
    await asyncio.sleep(1)

    # sequence changing colours
    red1 = 255
    green1 = 127
    blue1 = 0
    for z in range(0, 1000):
        await rgbw.set(red1, green1, blue1, 0, 255])
        red1 = (red1+1) % 256
        green1 = (green1+1) % 256
        blue1 = (blue1+1) % 256

    # sequence random colors
    for z in range(0, 20):
        await rgbw.set(red1, green1, blue1, 0, 255])
        red1 = ((red1+1)*int(get_random_bytes(1))) % 256
        green1 = ((green1+1)*int(get_random_bytes(1))) % 256
        blue1 = ((blue1+1)*int(get_random_bytes(1))) % 256
        await asyncio.sleep(5)   
        
    # read sensors
    sensor1 = BinarySensor(
        xknx,
        "DiningRoom.Motion.Sensor",
        group_address_state="6/0/2",
    )
    await sensor1.sync(wait_for_result=True)
    print("DiningRoom.Motion.Sensor val=",sensor1)

    sensor2 = Sensor(
        xknx,
        "DiningRoom.Temperature.Sensor",
        group_address_state="6/2/1",
        value_type="temperature",
    )

    await sensor2.sync(wait_for_result=True)
    print("DiningRoom.Temperature.Sensor val=",sensor2)

    # weather device
    weather = Weather(
        xknx,
        "Home",
        group_address_temperature="7/0/1",
        group_address_brightness_south="7/0/5",
        group_address_brightness_east="7/0/4",
        group_address_brightness_west="7/0/3",
        group_address_wind_speed="7/0/2",
        group_address_wind_bearing="7/0/6",
        group_address_day_night="7/0/7",
        group_address_rain_alarm="7/0/0",
    )

    await weather.sync(wait_for_result=True)
    print(weather.max_brightness)
    print(weather.ha_current_state())
    print(weather)

    # scene
    scene = Scene(xknx, name="Romantic", group_address="7/0/9", scene_number=23)
    await scene.run()

    # switch
    switch = Switch(xknx, name="TestOutlet", group_address="1/1/11")
    await switch.set_on()
    await asyncio.sleep(2)
    await switch.set_off()
    
    await xknx.stop()

if __name__ == '__main__':

    argc = len(sys.argv)
    if argc >= 2:
        asyncio.run(main(int(sys.argv[1]),int(sys.argv[2]))
    else:
        asyncio.run(main())
