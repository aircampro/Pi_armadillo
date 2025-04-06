#!/usr/bin/env python
#
# Impulsive protocol for coin receptors
#
# This should read the vending machine coin readers such as :-
# Münzprüfer Dietmar Trenner GmbH
# ELECTRONIC COIN SELECTOR
# EMP 500.xx v4 - which can be programmed to match the programming defined as below
#
# Also CH-926 Coin Acceptor https://www.arcadexpress.com/documentos/Manual%20CH926.pdf?srsltid=AfmBOortuUcHCGMSzkB4_8iVLFC0jf4MdXU4KDVFq6JISWEvlxLy0yrX
#
# Please note that the number of coin pulses per coin can be freely programmed
#
# When only one impuls is detected it was a 2EUR coin
# If it was two impulses it was 1 EUR coin
# 3 impulses mean 50 eurocent coin
# 4 impulses mean 20 cent coin
# 5 impulses mean 10 eurocents coin
#
# Taken from a raspi example for pulse to mqtt (mode=0) and adapted for the coin receptors as shown above mode=1 
# from https://github.com/balena-labs-projects/pulse/blob/main/server.py
#
import RPi.GPIO as GPIO
import time
import datetime
import os
#from balena import Balena
import sys
import paho.mqtt.client as mqtt
import json
import threading
import signal
import socket

# use if webserver option is true
SERVER_HOST = '10.0.0.1'
SERVER_PORT = 7575
		
pulse_per_second= 0
pulse_count = 0
pulse_trip = 0
pulse_trip_spt = 0
refund_pin = 23
accept_pin = 24

pulse_output = {
    "uuid": os.environ.get('BALENA_DEVICE_UUID'),
    "gpio": 0,
    "pulse_per_second": 0,
    "pulse_per_minute": 0,
    "pulse_per_hour": 0,
    "pulse_count": 0,
    "pulse_trip": 0,
    "pps_mult": 0,
    "ppm_mult": 0,
    "1Euro" : 0,
    "50Cent" : 0,
    "20Cent" : 0,
    "10Cent" : 0,
    "total_coin" : 0,
    "triggers" : 0,
    "pph_mult": 0
}
env_vars = {
    "pulse_multiplier": 1,
    "gpio_pin": 37,
    "bounce_time": 200,
    "mqtt_address": "none",
    "gpio_reset_pin": 38,
    "gpio_refund_pin": 23,
    "gpio_accept_pin": 24,
    "enable_webserver": 0,
    "pulse_trip_spt": 50,
    "mode": 1,
    "pull_up_down": "down"
}
sum_queue = []
pulse_multiplier = 0
client = mqtt.Client()

# 1 not used
# If it was two impulses it was 1 EUR coin
# 3 impulses mean 50 eurocent coin
# 4 impulses mean 20 cent coin
# 5 impulses mean 10 eurocents coin
#
# you can edit this class to suit your configuration
#
PULSE_WIDTH_VENDING = 1.0                                                                    # mark space greater than the expected pulse rate (denotes end of pulse train)
class CoinSelections(object):                                                                # this object contains all the possible coin permutations
    def __init__(self, reset_spt=-1):                                                        # default is no reset
        not_used1 = 0
        euro1 = 0
        cent50 = 0
        cent20 = 0
        cent10 = 0
        total_sum = 0
        reset_val = reset_spt
        triggers = 0
		
coin = CoinSelections()                                                                      # global coin object which is updated by pulses
listofcoins = [ coin.not_used1, coin.euro1, coin.cent50, coin.cent20, coin.cent10 ]          # list of coins in order 1,2,3,4,5

def total_coins():
    global coin
    coin.total_sum = coin.euro1 + (coin.cent50 * 0.5) + (coin.cent20 * 0.2) + (coin.cent10 * 0.1)
    if (not coin.reset_val == -1) and coin.total_sum > coin.reset_val:   
        coin.euro1 = 0
        coin.cent50 = 0
        coin.cent20 = 0
        coin.cent10 = 0
        coin.total_sum = 0
        coin.triggers += 1
        
def check_coin(counter):                                                                     # check the pulse counter once there has been more than PULSE_WIDTH_VENDING at zero & update relevant counter
    global coin, listofcoins
    if counter >0 and counter <= len(listofcoins):
        coin_found = listofcoins[counter-1]
        coin_found += 1  
    total_coins()
    
# Use the sdk to get services (eventually)
def mqtt_detect():
    
    return False

# Simple webserver
def background_web(server_socket):
    global pulse_output

    while True:
        # Wait for client connections
        client_connection, client_address = server_socket.accept()

        # Get the client request
        request = client_connection.recv(1024).decode()
        print(request)

        # Send HTTP response
        response = 'HTTP/1.0 200 OK\n\n'+ json.dumps(pulse_output)
        print(pulse_output)
        client_connection.sendall(response.encode())
        client_connection.close()

class ProgramKilled(Exception):
    """
    An instance of this custom exception class will be thrown every time we get an SIGTERM or SIGINT
    """
    pass

# Raise the custom exception whenever SIGINT or SIGTERM is triggered
def signal_handler(signum, frame):
    raise ProgramKilled

# This method fires on edge detection from a reset button
def on_reset(channel):
    global pulse_count
    global coin, refund_pin
    print("pulse reset detected")
    pulse_count = 0
    print("coin reset detected")
    coin.euro1 = 0
    coin.cent50 = 0
    coin.cent20 = 0
    coin.cent10 = 0
    coin.total_sum = 0
    GPIO.output(refund_pin,1)                              # open hatch for refund of coins
    time.sleep(1)
    GPIO.output(refund_pin,0)

# This function serves as the callback triggered on every run of our IntervalThread
def action() :
    global pulse_per_second, sum_queue, client, env_vars, pulse_output
    global coin, pulse_trip
    pulse_per_minute = 0
    pulse_per_hour = 0
    pulse_multiplier = env_vars["pulse_multiplier"]
    sum_queue.append(pulse_per_second)
    if len(sum_queue) > 1:
        pulse_per_minute = sum(sum_queue[-60:])
        pulse_per_hour = sum(sum_queue[-3600:])
    if len(sum_queue) > 3600:
        sum_queue.pop(0)
    pulse_output["gpio"] = env_vars["gpio_pin"]
    pulse_output["pulse_per_second"] = pulse_per_second
    pulse_output["pulse_per_minute"] = pulse_per_minute
    pulse_output["pulse_per_hour"] = pulse_per_hour
    pulse_output["pulse_count"] = pulse_count
    pulse_output["pps_mult"] = pulse_per_second * pulse_multiplier
    pulse_output["ppm_mult"] = pulse_per_minute * pulse_multiplier
    pulse_output["pph_mult"] = pulse_per_hour * pulse_multiplier
    pulse_output["1Euro"] = coin.euro1
    pulse_output["50Cent"] = coin.cent50 
    pulse_output["20Cent"] = coin.cent20
    pulse_output["10Cent"] = coin.cent10
    pulse_output["triggers"] = coin.triggers
    pulse_output["total_coin"] = coin.total_sum
    pulse_output["pulse_trip"] = pulse_trip
    #print(pulse_output)
    if env_vars["mqtt_address"] != "none":
        client.publish('pulse_data', json.dumps(pulse_output))
    pulse_per_second = 0

# See https://stackoverflow.com/questions/2697039/python-equivalent-of-setinterval
class IntervalThread(threading.Thread) :
    def __init__(self,interval,action, *args, **kwargs) :
        super(IntervalThread, self).__init__()
        self.interval=interval
        self.action=action
        self.stopEvent=threading.Event()
        self.start()

    def run(self) :
        nextTime=time.time()+self.interval
        while not self.stopEvent.wait(nextTime-time.time()) :
            nextTime+=self.interval
            self.action()

    def cancel(self) :
        self.stopEvent.set()

def main():

    global pulse_per_second, pulse_count, env_vars, client, coin
    global pulse_trip, pulse_trip_spt, refund_pin, accept_pin

    # device variables
    env_vars["pulse_multiplier"] = float(os.getenv('PULSE_MULTIPLIER', '1'))
    env_vars["gpio_pin"] = os.getenv('GPIO_PIN', 37)
    env_vars["bounce_time"]  = os.getenv('BOUNCE_TIME', 0)
    env_vars["mqtt_address"] = os.getenv('MQTT_ADDRESS', 'none')
    env_vars["gpio_reset_pin"] = os.getenv('GPIO_RESET_PIN', 38)
    env_vars["gpio_refund_pin"] = os.getenv('GPIO_REFUND_PIN', 23)
    env_vars["gpio_accept_pin"] = os.getenv('GPIO_ACCEPT_PIN', 24)
    env_vars["enable_webserver"] = os.getenv('ALWAYS_USE_HTTPSERVER', 0)
    env_vars["pull_up_down"] = os.getenv('PULL_UP_DOWN', 'NONE')
    env_vars["mode"] = os.getenv('PULSE_MODE', 1)
    env_vars["pulse_trip_spt"] = os.getenv('PULSE_TRIP_SPT', 50)
    mode = env_vars["mode"]
    pulse_trip_spt = env_vars["pulse_trip_spt"]
    refund_pin = env_vars["gpio_refund_pin"]
    accept_pin = env_vars["gpio_accept_pin"]
 	
    if env_vars["enable_webserver"] == "1":
        env_vars["enable_webserver"] = "True"
    else:
        env_vars["enable_webserver"] = "False"
        
    GPIO.setmode(GPIO.BOARD)
    gpio_pin = int(env_vars["gpio_pin"])

    # Set the pin for the incoming pulse
    if env_vars["pull_up_down"] == "UP":
        GPIO.setup(gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        test_cond = GPIO.LOW
    elif env_vars["pull_up_down"] == "DOWN":
        GPIO.setup(gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        test_cond = GPIO.HIGH
    else:
        GPIO.setup(gpio_pin, GPIO.IN)
        test_cond = GPIO.HIGH

    GPIO.setup(refund_pin, GPIO.OUT)
    GPIO.setup(accept_pin, GPIO.OUT)

    if env_vars["bounce_time"] == 0:
        bounce_time = 0
    else:
        if str(env_vars["bounce_time"]).isnumeric():
            bounce_time = int(env_vars["bounce_time"])/1000
            print("Bounce time set to {0} second(s)".format(bounce_time))
        else:
            bounce_time = 0

    # Set the pin for pulse count reset
    GPIO.setup(int(env_vars["gpio_reset_pin"]), GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(int(env_vars["gpio_reset_pin"]), GPIO.FALLING, callback=on_reset, bouncetime=200)

    if mqtt_detect() and env_vars["mqtt_address"] == "none":
        env_vars["mqtt_address"] = "mqtt"

    if env_vars["mqtt_address"] != "none":
        print("Starting mqtt client, publishing to {0}:1883".format(env_vars["mqtt_address"]))
        try:
            client.connect(env_vars["mqtt_address"], 1883, 60)
        except Exception as e:
            print("Error connecting to mqtt. ({0})".format(str(e)))
            env_vars["mqtt_address"] = "none"
            env_vars["enable_webserver"] = "True"
        else:
            client.loop_start()
    else:
        env_vars["enable_webserver"] = "True"

    if env_vars["enable_webserver"] == "True":

        # Create socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((SERVER_HOST, SERVER_PORT))
        server_socket.listen(1)
        print("HTTP server listening on port {0}...".format(SERVER_PORT))

        t = threading.Thread(target=background_web, args=(server_socket,))
        t.start()

    # Handle SIGINT and SIFTERM with the help of the callback function
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    # start action every 1s
    inter=IntervalThread(1,action)

    vendingcnt1 = 0
    vendingtm_start = time.time()
    vendingstate = 0
	
    # See https://www.g-loaded.eu/2016/11/24/how-to-terminate-running-python-threads-using-signals/
    while True:
        env_vars["pulse_trip_spt"] = os.getenv('PULSE_TRIP_SPT', 50)                      # allow change of set-point all the time
        pulse_trip_spt = env_vars["pulse_trip_spt"]
        try:
            if mode == 0:                                                                 # std pulse counter e.g. flowmeter
                GPIO.wait_for_edge(gpio_pin, GPIO.FALLING)
                if bounce_time > 0:
                    time.sleep(bounce_time)
                    if GPIO.input(gpio_pin) == test_cond:
                        pulse_per_second = pulse_per_second + 1
                        pulse_count = pulse_count + 1
                        if pulse_count >= pulse_trip_spt:                                 # have we reached the desired number of pulses
                            pulse_trip = 1
                        else:
                            pulse_trip = 0
                else:
                    pulse_per_second = pulse_per_second + 1
                    pulse_count = pulse_count + 1
                    if pulse_count >= pulse_trip_spt:                                      # have we reached the desired number of pulses
                        pulse_trip = 1
                    else:
                        pulse_trip = 0
            elif mode == 1:                                                                # coin recptor e.g. CH-926 or EMP-500
                if GPIO.input(gpio_pin) == 1 and vendingstate == 0:		           # got a pulse
                    vendingstate == 1
                elif GPIO.input(gpio_pin) == 0 and vendingstate == 1:	                   # it musr go to zero state to count	
                    vendingstate == 2
                    vendingcnt1 += 1
                    vendingtm_start = time.time()
                elif GPIO.input(gpio_pin) == 1 and vendingstate == 2:                      # got another one		
                    vendingstate == 3	
                elif GPIO.input(gpio_pin) == 0 and vendingstate == 2:	                   # didnt get another yet look for a timeout to count and return	
                    vendingtm_stop = time.time()
                    mark_space = vendingtm_stop - vendingtm_start
                    if mark_space >= PULSE_WIDTH_VENDING:
                        check_coin(vendingcnt1)                                            # update the right counter
                        vendingcnt1 = 0                                                    # reset counter
                        vendingstate = 0		                                   # return to look for new coin
                        if coin.total_sum >= pulse_trip_spt:                               # coins have reached the desired quantity
                            pulse_trip = 1
                            GPIO.output(accept_pin,1)                                      # open hatch to accept coins to main storage (start process)
                            time.sleep(1)
                            GPIO.output(accept_pin,0)                                      # close hatch and reset the coin total
                            coin.total_sum = 0
                        else:
                            pulse_trip = 0				
                elif GPIO.input(gpio_pin) == 0 and vendingstate == 3:	                   # zero state from second pulse
                    vendingcnt1 += 1                                                       # count again
                    vendingtm_stop = time.time()                                           # collect stop time
                    mark_space = vendingtm_stop - vendingtm_start
                    if mark_space >= PULSE_WIDTH_VENDING:                                  # also if this time too long update and reset                               
                        check_coin(vendingcnt1)
                        vendingcnt1 = 0
                        vendingstate = 0
                    else:
                        vendingtm_start = vendingtm_stop                                   # make stop start
                        vendingstate = 2                                                   # look for the next rising edge
						
        except ProgramKilled:
            print("Program killed: running cleanup code")
            inter.cancel()
            break

if __name__ == "__main__":
    main()
