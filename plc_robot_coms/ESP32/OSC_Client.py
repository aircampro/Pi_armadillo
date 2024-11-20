#!/usr/bin/env python
#
# example OSC client to Stack Chan ESP32 OSC server
#
from pythonosc import osc_message_builder
from pythonosc import udp_client
PORT_NO = 10000
OSC_SERVER_IPADDR="192.168.1.1"

client = udp_client.SimpleUDPClient(OSC_SERVER_IPADDR, PORT_NO)

def say_text_message(text_to_say):
    text = text_to_say
    cmd = 'talk ' + text
    print(cmd)
    client.send_message("/msg", cmd)

def change_face(face_to_show):
    text = face_to_show
    cmd = 'face ' + text
    print(cmd)
    client.send_message("/msg", cmd)

def change_expression():
    cmd = 'expr ' 
    print(cmd)
    client.send_message("/msg", cmd)

def change_color():
    cmd = 'colr ' 
    print(cmd)
    client.send_message("/msg", cmd)

def toggle_blink():
    cmd = 'blin ' 
    print(cmd)
    client.send_message("/msg", cmd)
	
if __name__ == "__main__":

    say_text_message("Welcome to the demo for Stack Chan")
    change_face("dog")
    say_text_message("changing the face again")
    change_face("pink")
    say_text_message("now the expression changes")
    change_expression()
    say_text_message("also the color changes")
    change_color()
    say_text_message("now toggle the blink")
    toggle_blink()
    change_face("custom")