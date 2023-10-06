#!/usr/bin/env python
 
"""
This simple switch input on the raspberry pi using an interrupt call back with a tk gui to show the status of the digital input
"""

import tkinter
import RPi.GPIO as GPIO
import time

DEFAULT_SWITCH_PORT = 23

class SwitchGUI(object):
    
    def __init__(self, port_num=DEFAULT_SWITCH_PORT):
        # GPIO
        self._port_num = port_num
        GPIO.setmode(GPIO.BCM)  
        GPIO.setup(self._port_num, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self._port_num, GPIO.BOTH,
                              callback=self.cb_switch)

    def run(self):
        # GUI
        self._root = tkinter.Tk()
        self._root.wm_title('Raspberry Pi Digital Input Pin Monitor')
        self._root.geometry('200x100')
        # Frame
        frame = tkinter.Frame(self._root)
        frame.pack()
        # Message
        self._message = tkinter.StringVar()
        self._message.set('Ready')
        label = tkinter.Label(frame,
                              textvariable=self._message, font=('', 30))
        label.pack()
        # Quit button
        button = tkinter.Button(frame, text='Quit', command=exit)
        button.pack()
        
        self._root.mainloop()

    def cb_switch(self, ch):
        self._state = GPIO.input(ch)
        if self._state:
            self._message.set('OFF')
        else:
            self._message.set('Pushed')

    
if __name__ == '__main__':
    app = SwitchGUI()
    app.run()