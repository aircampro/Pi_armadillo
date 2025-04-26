#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Reading IR signal from Mitsubishi AC IR hand-held controller
# and sending this signal, every IR is different so you will need to check with the handheld 
#
# uses pigpio so you must install and have daemon running
# $ sudo apt update
# $ sudo apt install pigpio python-pigpio -y
# $ sudo systemctl start pigpiod
#
# $ curl http://abyz.me.uk/rpi/pigpio/code/irrp_py.zip | zcat > irrp.py
#
# use this to record each IR command e.g Temperture to 26, Temperture to 27 etc
# python3 irrp.py -r -g18 -f codes --no-confirm --post 50 ac:cool27
# the last argument is for example cool 27 deg C - what you command from handheld to the AC
# 
# -r Mode of recording the received signal
# -g18 Receiving signals using GPIO18
# -f codes	Specify the file name to output the received signal.
# --no-confirm	Don't ask for the same signal twice for confirmation
# --post 50	If there is no signal for 50 ms, it is judged that the signal is over.
# ac:cool27	The identifier of the signal. Give it a descriptive name
# result is a list of timed pulses
#
# decodes in AEHA or NEC format please check for SONY
#
# $ python3 decode.py -f codes ac:cool27
# c4d36480000418d06c12000000000000006b
# c4d36480000418d06c12000000000000006b
# remote control signal of the Mitsubishi MSZ-GV225-W remote control transmitter model name RH151
#
# IR transmitter info https://qiita.com/takjg/items/e6b8af53421be54b62c9#%E8%B5%A4%E5%A4%96%E7%B7%9A%E3%82%92%E5%8F%97%E4%BF%A1%E3%81%99%E3%82%8B%E5%9B%9E%E8%B7%AF
# example cmd : $ echo 'm 17 w   w 17 0   m 18 r   pud 18 u' > /dev/pigpio
#
# This could win you money on TV games or make use of your old telly handhelds
#
import json
import os
import pigpio
from subprocess import PIPE, Popen
from enum import Enum

list_of_manu = [ "RC5", "RC6", "NEC", "SONY", "PANASONIC", "JVC", "SAMSUNG", "WHYNTER", "AIWA_RC_T501", "LG", "BANDO",  "SANYO", "MITSI", "DISH", "SHARP", "DENON",  "PRONTO",  "LEGO_PF" ]

class IR_Freq(Enum):   
    BandO_KHZ      = 455                                                # some B&O are also 38 KHz (std)
    SONY_KHZ       = 40
    BOSEWAVE_KHZ   = 38
    DENON_KHZ      = 38
    JVC_KHZ        = 38
    LG_KHZ         = 38
    NEC_KHZ        = 38
    SAMSUNG_KHZ    = 38
    KASEIKYO_KHZ   = 37                                                 # panasonic
    RC5_RC6_KHZ    = 36
    
# run cmdline task
def cmdline(command):
    """
    shell command to run script and check result
    """
    return Popen(
        args=command,
        stdout=PIPE,
        stderr=PIPE,
        shell=True
    )
    
# set-up gpio for the door lock
#
def set_up_door_lock(gpiopin=23):
    """
    set-up gpio for the door lock
    """
    pi = pigpio.pi()
    pi.set_mode(gpiopin, pigpio.OUTPUT)  
    return pi
          
# create the hex id from the recorded data json works for AEHA and NEC
#
def create_hex_from_list(filenm, id0):
    """
    create the hex id from the recorded data json works for AEHA and NEC
    """
    with open(filenm, "r") as f:
        records = json.load(f)
    if id0 in records:
        code = records[id0]
        t = 0.0
        data = []
        for i in range(0, len(code) - 1, 2):
            if t * 7.0 < code[i]:
                t = (code[i] + code[i + 1]) / 12.0
                data.append("")
            elif code[i + 1] < t * 2.0:
                data[-1] += "0"
            elif code[i + 1] < t * 6.0:
                data[-1] += "1"
        rlist = []
        for d in data:
            print(format(int(d, 2), "x"))
            rlist.append(format(int(d, 2), "x"))
        return rlist
         
# read the signal from the IR controller and check against our recorded library
# pass gpio pin which IR receiver connected to
#
def read_ir_sent(pi_handle, gpio_pin=19, action_pin=23):
    """
    read the signal from the IR controller and perform action if mathching
    """
    os.remove(my_send)
    ir_sent = 0
    run_str = f'python3 irrp.py -r -g{gpio_pin} -f my_send --no-confirm --post 50 ac:request'      # record the ir handheld key
    cmd_printed = cmdline(run_str).stdout.readline()
    print(cmd_printed)    
    ir_sent = create_hex_from_list(my_send, "ac:request")
    for msgs_recorded in [ "ac:cool25", "ac:cool26", "ac:cool27", "open_door", "close_door", "tv_button" ]:     # msgs we sent and recorded
        ir_sig = create_hex_from_list(codes, msgs_recorded)   
        if ir_sig == ir_sent:
            print("matches ",msgs_recorded)                                                        # if it matched any that were recorded it prints it
            if ir_sig == "open_door":                                                              # check for door actions and open/close lock
                pi_handle.write(action_pin,1)  
            elif ir_sig == "close_door":                                                           # power fail state is closed (its safe)
                pi_handle.write(action_pin,0)     
            elif ir_sig == "tv_button":                                                            # tv button on control handheld
                send_ir_file("tv_data", "tv_button_1")                                             # send an IR signal to the TV using its protocol e.g. panasonic

# control door lock from IR handheld controller
#
def key_door():
    """
    open close door from IR controler
    """
    hPi = set_up_door_lock()
    while True:
        try:
            read_ir_sent(hPi)
        except Exception as e:
            print("Exception door lock control = ",e)
        
# mitsubishi AC control (e.g. set home heating on and off)
#  
def make_mitsiAC_file(filenm):
    # ON (22 ℃)
    mitsiAC_on_22 = [ 3200,1600, 400,400, 400,1200, 400,350, 450,350, 450,1150, 450,350, 400,1200, 400,350, 450,350, 450,1150, 450,1150, 400,1200, 400,350, 450,1150, 450,350, 450,1150, 450,1150, 400,1150, 450,350, 450,350, 450,350, 450,350, 400,1200, 400,1150, 450,350, 450,1150, 450,1150, 400,400, 400,350, 450,1150, 450,350, 450,350, 450,1150, 400,350, 450,400, 400,1150, 450,1150, 450,350, 400,1200, 400,1150, 450,1150, 450,1150, 450,1100, 450,1150, 450,1150, 450,1150, 450,1100, 450,1150, 450,350, 450,350, 450,350, 450,300, 450,350, 450,350, 450,350, 450,350, 450,1150, 450,1150, 400,1200, 400,1200, 400,1150, 450,1150, 400,1150, 450,1150, 450,350, 450,350, 450,350, 400,400, 400,350, 450,350, 450,350, 450,350, 450,1150, 450,1150, 400,350, 450,400, 400,350, 450,1150, 450,300, 500,1150, 400,400, 400,350, 450,1150, 450,1150, 450,1100, 450,350, 450,1150, 450,350, 450] # F288C8C8
    # OFF
    mitsiAC_off = [ 3200,1650, 400,350, 450,1150, 450,350, 400,400, 400,1200, 400,350, 450,1150, 450,350, 400,400, 450,1150, 450,1100, 450,1150, 450,350, 450,1150, 400,400, 400,1150, 450,1150, 400,1200, 450,350, 400,350, 450,350, 450,350, 450,1150, 450,1100, 500,350, 400,1150, 450,1150, 450,350, 450,350, 450,1150, 400,400, 400,400, 400,1150, 450,350, 400,400, 450,1150, 400,1150, 450,350, 450,1150, 450,1150, 450,1100, 450,1150, 450,1150, 450,1100, 500,1100, 450,1150, 450,1150, 450,1150, 450,350, 400,350, 450,350, 450,350, 450,350, 450,350, 450,350, 450,350, 400,1150, 450,1150, 450,1150, 450,1100, 500,1150, 400,1150, 450,1150, 450,1150, 450,300, 450,350, 450,350, 450,350, 450,350, 400,400, 400,400, 450,350, 400,1200, 400,1150, 450,350, 400,1200, 400,400, 450,1100, 450,350, 450,1150, 450,350, 450,350, 400,1150, 450,350, 450,1150, 450,350, 400,1200, 400,400, 450]  # 51E87A6C
    with open(filenm, "w") as f:
        rec = { 'AC_22' : mitsiAC_on_22 }
        records = json.dumps(rec)
        f.write(records)
        rec = { 'AC_OFF' : mitsiAC_off }
        records = json.dumps(rec)
        f.write(records)
                        
# send id if it has been already recorded into the file with pre-post-ambles as sepcified and frequency, short code length as specified  
#                                    
def send_ir_file_id(gpio_pin=18, send_fle, id0, pre=50, post=200, f=38, s=10):
    """
    sends the data recorded in the file as IR pulses
    """
    run_str = f'python3 irrp.py -r -g{gpio_pin} -f {send_fle} -pre {pre} -post {post} -freq {f} -short {s} --no-confirm --post 50 {id0}'   
    cmd_printed = cmdline(run_str).stdout.readline()
    print(cmd_printed)


