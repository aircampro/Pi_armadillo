# read a card and control the electric door
#
# sudo apt-get update
# sudo apt-get install -y python-dev
# sudo apt-get install -y python-pip
# sudo pip install RPi.GPIO
#
# https://monomonotech.jp/kurage/raspberrypi/nfc.html
# sudo pip3 install nfcpy
#
# git clone https://github.com/nfcpy/nfcpy.git

# https://www.amazon.co.jp/dp/B00948CGAG?th=1
# sudo nano /etc/udev/rules.d/nfcdev.rules
# SUBSYSTEM=="usb", ACTION=="add", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="06c3", GROUP="plugdev" # Sony RC-S380/P
#
#
# door switch https://akizukidenshi.com/catalog/g/gP-13371/
# output is 3.3v and drives a relay to a motorised lock
#
import binascii
import nfc
import os

import time
import RPi.GPIO as GPIO

# for voice if required
VOICE_ON=1
if VOICE_ON == 1:
    import subprocess

GPIO.setmode(GPIO.BCM)
# GPIO18pin is connected to the door switch
DOOR_SW = 18
GPIO.setup(DOOR_SW,GPIO.IN,pull_up_down=GPIO.PUD_UP)
# GPIO21pin is connected to the lock
OPERATE_LOCK = 21
GPIO.setup(OPERATE_LOCK, GPIO.OUT)
ALARM_OUT = 19
GPIO.setup(ALARM_OUT, GPIO.OUT)

# timer setpoints in seconds
TOO_LONG=30
RE_TRIGR=5

class MyCardReader(object):
    def on_connect(self, tag):
        print("【 Touched 】")
        print(tag)

        self.idm = binascii.hexlify(tag._nfcid)
        print("IDm : " + str(self.idm))
        self.open_door = 0
        
        # check the idm
        if self.idm == "00000000000000":
            print("【 found the ID 】")
            self.open_door = 1
            if VOICE_ON == 1:
                subprocess.Popen(['../sound/voice/say_this.sh', 'mai', 'card valid.. open. door'])            
        return True

    def read_id(self):
        clf = nfc.ContactlessFrontend('usb')
        self.open_door = 0
        try:
            clf.connect(rdwr={'on-connect': self.on_connect})
        finally:
            clf.close()

if __name__ == '__main__':
    cr = MyCardReader()
    # ensure lock is closed
    GPIO.output(OPERATE_LOCK, 0) 
    GPIO.output(ALARM_OUT, 0) 
    msg_latch = 0
    
    while True:
        # read the card
        print("Please Touch")
        door_state = 0

        # read the card        
        cr.read_id()

        # card removed
        print("【 Released 】")

        if cr.open_door == 1:                     # valid id
            # operate 3.3v relay for the door lock
            GPIO.output(OPERATE_LOCK, 1) 
            door_state = 1 
            start_tm = time.time()
            st2 = time.time()
            
        while door_state == 1:                    # valid id wait for open
            try:
                sw_status = GPIO.input(DOOR_SW)
                if sw_status == 0:                # door closed
                    #print("Close")
                    cur_tm = time.time()
                    if (cur_tm - start_tm) > TOO_LONG:
                        print("took too long please re-swipe")
                        GPIO.output(OPERATE_LOCK, 0) 
                        door_state = 0                        
                else:
                    #print("Open!")                # door open
                    door_state = 2                 # wait for close
                    start_tm = time.time()
                time.sleep(0.03)
            except:
                break 
        while door_state == 2:                      # now wait for the door closed
            try:
                sw_status = GPIO.input(DOOR_SW)
                if sw_status == 0:                  # door closed
                    #print("Close")
                    door_state = 0 
                    GPIO.output(OPERATE_LOCK, 0)    # disable lock
                    GPIO.output(ALARM_OUT, 0)       # clear alarm                  
                else:
                    #print("Open!")                 # door open
                    cur_tm = time.time()
                    if (cur_tm - start_tm) > TOO_LONG:
                        print("open too long.... set alarm")
                        GPIO.output(ALARM_OUT, 1) 
                        if msg_latch == 0:
                            if VOICE_ON == 1:
                                subprocess.Popen(['../sound/voice/say_this.sh', 'Please.. cloae. door']) 
                            msg_latch = 1
                            st2 = time.time()
                        if ((cur_tm - st2) > RE_TRIGR) and (msg_latch == 1):
                            msg_latch = 0
                time.sleep(0.03)
            except:
                break   