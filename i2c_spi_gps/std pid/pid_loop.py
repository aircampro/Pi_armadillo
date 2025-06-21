#
# standard PID controller example (temperature controller)
#
# for DHT11 input git clone https://github.com/szazo/DHT11_Python.git
#
# using ups from alchemy power with output connected to a GPIO pin
#
import RPi.GPIO as GPIO
import dht11       
import time
import datetime
import numpy as np
import pickle
from std_pid import Controller

# define inputs normal and hold and trip levels as you wish (check board layout)
nml_gpio = 23
hld_gpio = 24
out_gpio = 25
trip_high = 40.0
trip_low = 5.0
ups_gpio = 26                                                       # e.g pin 37
rmpu_gpio = 27
rmpd_gpio = 28
pickle_file = "pickle_pid.pkl"

if __name__ == "__main__":

    # read the last known integral value if we can
    try:
        with open(pickle_file, 'rb') as f:
            ei = pickle.load(f) 
    except:
        ei = 0.0
		
    # initialize GPIO
    GPIO.setwarnings(False)       
    GPIO.setmode(GPIO.BCM)                                          # we are using GPIO numbering scheme
    # GPIO.setmode(GPIO,BOARD)                                      # to use board pin numbers instead
    GPIO.setup(out_gpio, GPIO.OUT)
    GPIO.setup(nml_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # controller is normal state
    GPIO.setup(hld_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # controller is hold state
    GPIO.setup(ups_gpio, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)       # set when it goes to zero
    GPIO.setup(rmpu_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # controller is ramp up state
    GPIO.setup(rmpd_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # controller is ramp down state
    
    # create controller instance with last known output value	
    tic101=Controller()  
    tic101.error_integral = ei                                      # set integral from what was last saved in the pickle file   
    tic101spt = 30.0                                                # set-point we will read this from a LSTM prediction in future
    instance = dht11.DHT11(pin=14)
    tt101value = 0.0
    rv = 0.05
    scan_t = 0.1
    
    # read io and calculate a controller output
    while True:
        temp_humid = instance.read()                               # get temp and humid
        if temp_humid.is_valid():
            tt101value = temp_humid.temperature
            at101value = temp_humid.humidity
        if GPIO.input(nml_gpio) == True:
            o = tic101.update(tic101spt, tt101value, 0, 100.0)     # update pid with data and set object output
            with open(pickle_file, 'wb') as f:                     # save the integral in a pickle file
                pickle.dump(tic101.error_integral, f)
        elif GPIO.input(hld_gpio) == True:
            o = tic101.hold()                                      # hold the pid 
        elif GPIO.input(rmpu_gpio) == True:
            o = tic101.ramp_up(rv, 100.0)                          # ramp-up 
        elif GPIO.input(rmpd_gpio) == True:
            o = tic101.ramp_down(rv, 100.0)                        # ramp-down
        elif GPIO.input(ups_gpio) == True:                         # we got a power failure signal from the ups battery pack
            print("power failed")
            with open(pickle_file, 'wb') as f:                     # save the integral in a pickle file
                pickle.dump(tic101.error_integral, f)
        else:        
            o = tic101.reset()                                     # stop pid immediately	
        print(" pid output is ",o)                                 # you can send this where you want to
        if tt101value > trip_high:		                           # trip amp set on high low value
            GPIO.output(out_gpio, GPIO.HIGH)
        elif tt101value > trip_low:		
            GPIO.output(out_gpio, GPIO.LOW)
        time.sleep(scan_t)
        
    GPIO.cleanup()
