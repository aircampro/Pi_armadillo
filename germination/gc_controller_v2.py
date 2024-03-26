# germination system
import os
from datetime import datetime
import time
import json
import board

import busio
import RPi.GPIO as GPIO
import adafruit_sht31d
import I2C_LCD_driver

# GPIO pin scheme
GPIO.setmode(GPIO.BCM)  # BCM channel, ex GPIO#
# OUPTUT
coil_LED = 10; GPIO.setup(coil_LED, GPIO.OUT)
mister_LED = 9; GPIO.setup(mister_LED, GPIO.OUT)
sys_LED = 11; GPIO.setup(sys_LED, GPIO.OUT)

coil_relay = 14; GPIO.setup(coil_relay, GPIO.OUT)
mister_relay = 5; GPIO.setup(mister_relay, GPIO.OUT)
alarm_relay = 6; GPIO.setup(alarm_relay, GPIO.OUT)

# using relay now
fan_driver = 13; GPIO.setup(fan_driver, GPIO.OUT)  # NO relay
# fan_PWM = GPIO.PWM(fan_driver, 4000)  # set frequency
# fan_PWM.start(0)

# INPUT
coil_switch = 7; GPIO.setup(coil_switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
mister_switch = 8; GPIO.setup(mister_switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
sys_switch = 25; GPIO.setup(sys_switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# LDC SCREEN
lcd = 0x27  # I2C
lcd_cols = 16
lcd_rows = 2
mylcd = I2C_LCD_driver.lcd()

# TIME
now = datetime.now()

# --ENVIRONMENTAL VARS--
GC_TEMP_UPPER = 81  # f
GC_TEMP_LOWER = 79  # f
# heat coil
COIL_TEMP_LIMIT = 145  # f
COIL_TEMP_COOL = 110  # f
COIL_TIME_LIMIT = 3  # min
COIL_BREAK_TIME = 1 # min
# humidifier
GC_HUMD_UPPER = 98  # %RH
GC_HUMD_LOWER = 100  # %RH

# -- SYS VARS --
#FAN_PWM_LVL = 15  # software pwm duty cycle
data_out_path = '/home/pi/Documents/AG_GC/MDP_DATA'

# -- COM ADDR --
gc_sht31d_addr = 0x44  # I2C
ext_sht31d_addr = 0x45  # I2C
LCD_addr = 0x27  # I2C
coil_ds18_addr = '28-000006b03811'  # 1W

# --SENSORS INIT--
# 1W - DS18B20, high temp
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
# I2C - SHT31-D, temp & humidity
i2c = busio.I2C(board.SCL, board.SDA)
gc_temp_humd_i2c = adafruit_sht31d.SHT31D(i2c, address=gc_sht31d_addr)
ext_temp_humd_i2c = adafruit_sht31d.SHT31D(i2c, address=ext_sht31d_addr)

def get_time():
    return datetime.now().strftime("%H:%M:%S")

def convert_tempf(temp_c: float):
    """
    convert celcius to farh
    """
    return round((temp_c * 1.8) + 32, 1)

def LED_startup_display(LED_pin1: int, LED_pin2: int, LED_pin3: int):
    """
    This runs a LED lighting display to signify the system is running, part of UI
    """
    actuate_all(0)
    print('<*RUNNING*>')
    LED_delay_loop = 0.1
    LED_delay_flash = 0.3
    # start all OFF
    GPIO.output(LED_pin1, 0)
    GPIO.output(LED_pin2, 0)
    GPIO.output(LED_pin3, 0)
    # LED loop
    for i in range(10):
        GPIO.output(LED_pin1, 1)
        time.sleep(LED_delay_loop)
        GPIO.output(LED_pin1, 0); GPIO.output(LED_pin2, 1)
        time.sleep(LED_delay_loop)
        GPIO.output(LED_pin2, 0); GPIO.output(LED_pin3, 1)
        time.sleep(LED_delay_loop)
        GPIO.output(LED_pin3, 0)
    # LED flash
    for i in range(3):
        GPIO.output(LED_pin1, 1); GPIO.output(LED_pin2, 1); GPIO.output(LED_pin3, 1)
        time.sleep(LED_delay_flash)        
        GPIO.output(LED_pin1, 0); GPIO.output(LED_pin2, 0); GPIO.output(LED_pin3, 0)
        time.sleep(LED_delay_flash)


def sys_error_display(LED_pin1: int, LED_pin2: int, LED_pin3: int):
    """
    This runs a LED and alarm display to signify the system has hit an error that stopped the program, part of UI
    """
    LED_delay_flash = 0.25
    # start all OFF
    GPIO.output(LED_pin1, 0)
    GPIO.output(LED_pin2, 0)
    GPIO.output(LED_pin3, 0)
    # LED flash
    while True:
        # ON
        GPIO.output(LED_pin1, 1); GPIO.output(LED_pin2, 1); GPIO.output(LED_pin3, 1)
        GPIO.output(alarm_relay, 0)  # alarm
        time.sleep(LED_delay_flash)
        # OFF
        GPIO.output(LED_pin1, 0); GPIO.output(LED_pin2, 0); GPIO.output(LED_pin3, 0)
        GPIO.output(alarm_relay, 1)  # alarm
        time.sleep(LED_delay_flash)
        print('<System Error - Must Reset Program>')

def get_sht31_temp(i2c_sensor, temp_unit='f'):
    """
    return temp of sht31 i2c object, rounded
    """
    temp = i2c_sensor.temperature
    temp = convert_tempf(temp) if temp_unit == 'f' else temp
    return round(temp, 2)

def get_sht31_humd(i2c_sensor):
    """
    return relative humidity of sht31 i2c object, rounded
    """
    humd = i2c_sensor.relative_humidity
    return round(humd, 2)

def get_w1temp(w1_addr: str, temp_unit: str, prev_temp: int=0):
    """
    This grabs the W1 temp in either F or C units
    """
    try:
        w1_dir = '/sys/bus/w1/devices/' + w1_addr + '/w1_slave'
        w1slave_file = open(w1_dir)
        w1slave_text = w1slave_file.read()
        # parse data script
        text_2ndline = w1slave_text.split('\n')[1]
        temp_data = text_2ndline.split(' ')[9]  # 10th item
        temp_reading = float(temp_data[2:])  # ignore 't='
        temp_c = round(temp_reading / 1000, 1)
        if temp_unit is 'c':
            return temp_c
        elif temp_unit is 'f':
            temp_f = convert_tempf(temp_c)
            return temp_f
        else:
            print('ERROR - Temp Unit')
    except Exception:
        # if w1 break, (-) previous will be sent as signal
        return -1*prev_temp
        pass


def get_sensor_state(prev_w1temp=0):
    """
    retrieve and return ordered sensor values of system.
    also, print temp/humd values of sensors
    [Coil-temp, GC-temp, GC-humd, Ext-temp, Ext-humd]
    """
    # get sys state
    coil_temp = get_w1temp(coil_ds18_addr, 'f', prev_w1temp)
    gc_temp = get_sht31_temp(gc_temp_humd_i2c, 'c')
    gc_humd = get_sht31_humd(gc_temp_humd_i2c)
    ext_temp = get_sht31_temp(ext_temp_humd_i2c, 'c')
    ext_humd = get_sht31_humd(ext_temp_humd_i2c)
    w1_error_note = '*' if coil_temp < 0 else ''
    print(f'********** {get_time()} **********')
    print(f'Coil Temp: {w1_error_note*2}{coil_temp}f')
    print(f'GC Temp:   {gc_temp}f, GC Humidity: {gc_humd}%RH')
    print(f'EXT Temp:  {ext_temp}f, EXT Humidity: {ext_humd}%RH')
    return coil_temp, gc_temp, gc_humd, ext_temp, ext_humd


def get_switch_state():
    """
    read and return pin state of switches
    [sys-switch, mister-switch, coil-switch]
    """
    return GPIO.input(sys_switch), GPIO.input(mister_switch), GPIO.input(coil_switch)


def actuate_all(signal: int):
    """
    actuate all relays/drivers to same val
    """
    # LEDs
    GPIO.output(coil_LED, signal)
    GPIO.output(mister_LED, signal)
    GPIO.output(sys_LED, signal)
    # relays, driver
    GPIO.output(coil_relay, signal)
    GPIO.output(mister_relay, 1-signal)
    GPIO.output(alarm_relay, 1-signal)
    GPIO.output(fan_driver, 0)  # default always off
    report = 'All ON' if signal else 'All OFF'
    print(report)


def actuate_coil(cmd: int):
    """
    low-level control commands of coil system
    1-ON, 0-OFF
    """
    GPIO.output(fan_driver, 0)  # redundant turn off fan driver
    if cmd:
        print('<Coil ON>')
    else:
        print('<Coil OFF>')
    GPIO.output(coil_relay, cmd)
    GPIO.output(coil_LED, cmd)
    return cmd


def actuate_mister(cmd: int, coil_state=1):
    """
    low-level control commands of mister/fan system
    1-ON, 0-OFF
    """
    GPIO.output(mister_relay, 1-cmd)
    GPIO.output(mister_LED, cmd)
    # Coil ON, don't power fan
    if coil_state or not cmd:
        GPIO.output(fan_driver, 0)
        print('<Mister Fan OFF>')
    # Coil OFF, need to power fan externally
    elif not coil_state and cmd:
        GPIO.output(fan_driver, 1)
        print('<Mister Fan ON>')
    # output
    if cmd:
        print('<Mister ON>')
    else:
        print('<Mister OFF>')
    return cmd


def coil_overheat_op(cool_temp: int):
    """
    This runs a LED and alarm display to signify the coil has overheat - runs until temp safe. User must flip coil
    switch to acknowledge overheat to turn off alarm and resume normal operation
    ((DIRECTION: turn coil switch OFF until alarm is off, then turn back on for normal op to resume))
    [Coil-temp, GC-temp, GC-humd, Ext-temp, Ext-humd]
    [sys-switch, mister-switch, coil-switch]
    """
    print('<OVERHEAT Routine>')
    flash_time = 0.2
    switch_orig = get_switch_state()[2]
    noticed = False  # first switch flip
    resolved = False  # second switch flip
    cooled = False
    while True:
        coil_temp = get_sensor_state()[0]
        coil_switch = get_switch_state()[2]
        # fan cooling and reset
        if coil_temp < cool_temp:
            cooled = True
            print('<Coil Cooled>')
        if not cooled:
            GPIO.output(fan_driver, 1)  # cool using fan
            print('<Cooling Coil with Fan>')
        # RESET system and resume normal op
        elif resolved and cooled:
            actuate_all(0)  # all off
            print('<Coil Cooling Resolved - Return to Normal Op>')
            break  # exit subroutine
        else:
            GPIO.output(fan_driver, 0)  # cooling done
            print('<Cooling Fan Off>')
        # user awcknowledgement
        if coil_switch is not switch_orig:
            noticed = True
            GPIO.output(alarm_relay, 1)  # turn off alarm
            print('<Overheat Acknowledged>')
        elif coil_switch is switch_orig and noticed:  # acknowledged and switched turned back
            resolved = True
        # run alarm and flash LED
        if not noticed:
            GPIO.output(alarm_relay, 0); GPIO.output(coil_LED, 0)
            time.sleep(flash_time)
            GPIO.output(alarm_relay, 1); GPIO.output(coil_LED, 1)
            # w1 sensor acq gives off delay
        # run just LED while cooling
        elif noticed and not cooled:
            GPIO.output(coil_LED, 1)
            time.sleep(flash_time)
            GPIO.output(coil_LED, 0)


def run_coil(sensor_state: tuple, switch_state: tuple, coil_state: int, target_hit: int, time: float):
    """
    bang-bang control strategy for coil with conditions
    [Coil-temp, GC-temp, GC-humd, Ext-temp, Ext-humd]
    """
    # gather useful data
    coil_temp = sensor_state[0]  # coil temp
    gc_temp = sensor_state[1]
    coil_switch = switch_state[2]  # coil switch
    sys_switch = switch_state[0]

    # time limit for coil running continuously, manage elapsed time in main loop
    if time > COIL_TIME_LIMIT:
        overtime = True  # time limit reached, give coil break
        print('*Coil Overtime Limit*')
    else:
        overtime = False

    # run coil, redundant sys switch conditions
    if sys_switch and coil_switch and coil_temp < COIL_TEMP_LIMIT and gc_temp < GC_TEMP_UPPER and not target_hit and not overtime:
        coil_state = actuate_coil(1)
    # overheat
    elif coil_temp >= COIL_TEMP_LIMIT:
        coil_state = actuate_coil(0)
        # OVERHEAT procedure, grabs control of program until cooling satisfied
        coil_overheat_op(COIL_TEMP_COOL)
    elif overtime:
        coil_state = actuate_coil(0)
    # BANG BANG controller part
    elif gc_temp >= GC_TEMP_UPPER or not coil_switch:  # achieved heating goal or switches off
        coil_state = actuate_coil(0)
        target_hit = 1  # hit setpoint UB
        if not coil_switch:
            print('<Coil Switch OFF>')
        else:  
            print('*GC UB Temp Target Hit*')
    elif gc_temp < GC_TEMP_LOWER:
        target_hit = 0  # hit setpoint LB, reset
        print('*GC LB Temp Target Hit*')
    # fail safe default
    else:
        coil_state = actuate_coil(0)

    return coil_state, target_hit, overtime


def run_mister(sensor_state: tuple, switch_state: tuple, mister_state: int, target_hit: int, coil_state: int):
    """
    bang-bang control strategy for mister with conditions
    [Coil-temp, GC-temp, GC-humd, Ext-temp, Ext-humd]
    """
    humd = sensor_state[2]  # gc humd
    switch = switch_state[1]  # mister switch
    # power ON sys if below UB, haven't met target, and switch on
    if humd < GC_HUMD_UPPER and not target_hit and switch:
        mister_state = actuate_mister(1, coil_state)  # turn on mister sys
    # power OFF sys if met UB goal or switch off
    elif humd >= GC_HUMD_UPPER or not switch:
        mister_state = actuate_mister(0, coil_state)  # turn off mister sys
        target_hit = 1  # hit setpoint UB
        if not switch:
            print('<Humidity Switch OFF>')
        else:
            print('*Humidity UB Target Hit*')
    # once humidity falls below desirable window
    elif humd <= GC_HUMD_LOWER:
        target_hit = 0 # hit setpoint LB, reset
        print('*Humidity LB Target Hit*')
    return mister_state, target_hit


# --STARTUP ROUTINE--
actuate_all(0)  # turn all systems off
LED_startup_display(sys_LED, mister_LED, coil_LED)  # UI feature
actuate_all(0)  # turn all systems off
coil_show = False  # alternating display

# -- RUN LOOP --
try:
    while True:
        # default sys shutdown
        actuate_all(0)
        # sys init vars
        mister_state = 0; mister_target = 0
        coil_state = 0; coil_target = 0
        # timing init for coil limit
        start_coil_time = time.time()
        # get sys state
        sensor_state = get_sensor_state()  # [Coil-temp, GC-temp, GC-humd, Ext-temp, Ext-humd]
        switch_state = get_switch_state()  # [sys-switch, mister-switch, coil-switch]
        # SYS LOOP
        while switch_state[0] == 1:  # sys switch ON
            GPIO.output(sys_LED, 1)
            # for model data collection <sensor states, humidity satisfaction, temp satisfaction>
            prev_state = sensor_state
            # elapsed time
            d_time = (time.time() - start_coil_time) / 60  # min
            # actuate system
            coil_state, coil_target, overtime = run_coil(sensor_state, switch_state, coil_state, coil_target, d_time)
            mister_state, mister_target = run_mister(sensor_state, switch_state, mister_state, mister_target, coil_state)
            # handle coil cooling time
            if overtime:
                if d_time > COIL_TIME_LIMIT + COIL_BREAK_TIME :
                    start_coil_time = time.time()
                    print('*Coil Cool Limit Reached*')
                else:
                     print(f'<Cooling Coil for {COIL_BREAK_TIME} mins>')
            # non-continuous operation, interrupts overtime count
            elif not overtime and not coil_state:
                start_coil_time = time.time()  # coil not running, reset timer

            # get sys state
            sensor_state = get_sensor_state(prev_state[0])  # [Coil-temp, GC-temp, GC-humd, Ext-temp, Ext-humd]
            switch_state = get_switch_state()  # [sys-switch, mister-switch, coil-switch]
            # model tuple <prev_state, actions, next_state>
            state = sensor_state
            #model_instance = prev_state + (coil_state) + (mister_state) + state
            # save to file for day
            ct = int(sensor_state[0])
            gt = round(sensor_state[1], 1)
            gh = int(sensor_state[2])
            et = round(sensor_state[3], 1)
            eh = int(sensor_state[4])
            mylcd.clear()  # reset display
            mylcd.lcd_display_string(f"GT:{gt}F  GH:{gh}%", 1)
            if coil_show:
                w1_error_note = '*' if ct < 0 else ' '
                mylcd.lcd_display_string(f"ET:{et}F{w1_error_note}CT:{ct}F", 2)
                coil_show = False
            else:
                mylcd.lcd_display_string(f"ET:{et}F  EH:{eh}%", 2)
                coil_show = True
        # main switch off
        print('<Sys Switch OFF>')
        GPIO.output(sys_LED, 0)
        ct = int(sensor_state[0])
        gt = round(sensor_state[1], 1)
        gh = int(sensor_state[2])
        mylcd.clear()  # reset display
        mylcd.lcd_display_string("***SYSTEM OFF***", 1)
        if coil_show:
            w1_error_note = '*' if ct < 0 else ' '
            mylcd.lcd_display_string(f"GT:{gt}F{w1_error_note}CT:{ct}F", 2)
            coil_show = False
        else:
            mylcd.lcd_display_string(f"GT:{gt}F  GH:{gh}%", 2)
            coil_show = True

except KeyboardInterrupt:
    # end program from bash terminal, etc
    actuate_all(0)
    print("Program Terminated.")

except:
    # standby error notification
    print("-ERROR-")
    actuate_all(0)
    sys_error_display(sys_LED, mister_LED, coil_LED)  # UI feature
	
============================================

