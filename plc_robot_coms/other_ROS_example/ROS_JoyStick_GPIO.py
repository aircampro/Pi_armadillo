# https://www.biccamera.com/bc/item/5103296/
#
# ROS Joystick interface to Folo Robot via GPIO
# sudo apt install pigpio
# sudo pigpiod
#
import rospy
from sensor_msgs.msg import Joy
import time
import signal

# define the gpio pins
#
# pigpiof set true if you are using pigpio libs and daemon pigpiod
pigpiof = None
if pigpiof == True:
    import pigpio
else:
    import RPi.GPIO as GPIO

class Folo:

    # initialise ROS for joystick and i/o harness to Folo Robot
    #
    def __init__(self):
        rospy.init_node('folo', anonymous=True)
        self.subscriber = rospy.Subscriber("/joy",Joy,self.callback)

        if pigpiof == True:
            self.pi = pigpio.pi()
    	    self.pi.set_mode(14,pigpio.OUTPUT)
    	    self.pi.set_mode(15,pigpio.OUTPUT)
    	    self.pi.set_mode(17,pigpio.OUTPUT)
    	    self.pi.set_mode(18,pigpio.OUTPUT)
        else:
            GPIO.setmode(GPIO.BCM)
            for gpio_pin in [14, 15, 17, 18]:
                GPIO.setup(gpio_pin, GPIO.OUT)
        self.con=0
        rospy.spin()

    # stop and clean-up if you close the app
    #
    def __del__(self):
       if pigpiof == True:
           self.pi.write(14,0)
           self.pi.write(15,0)
           self.pi.write(17,0)
           self.pi.write(18,0)
       else:
           GPIO.output(14,0)
           GPIO.output(15,0)
           GPIO.output(17,0)
           GPIO.output(18,0)
           GPIO.cleanup()
       self.subscriber = None

    # callback when joystick movement detected on ROS subscriber
    #
    def callback(self,ros_data):
        if(ros_data.buttons[0]==1):
            if pigpiof == True:
                self.pi.write(14,1)
                self.pi.write(15,0)
            else:
                GPIO.output(14,1)
                GPIO.output(15,0)
        elif(ros_data.buttons[1]==1):
            if pigpiof == True:
                self.pi.write(14,0)
                self.pi.write(15,1)
            else:
                GPIO.output(14,0)
                GPIO.output(15,1)
        if(ros_data.buttons[2]==1):
            if pigpiof == True:
                self.pi.write(17,1)
                self.pi.write(18,0)
            else:
                GPIO.output(17,1)
                GPIO.output(18,0)
        elif(ros_data.buttons[3]==1):
            if pigpiof == True:
                self.pi.write(17,0)
                self.pi.write(18,1)
            else:
                GPIO.output(17,0)
                GPIO.output(18,1)
	if(ros_data.buttons[4]==1):
            if pigpiof == True:
	        self.pi.write(14,0)
	        self.pi.write(15,0)
	        self.pi.write(17,0)
	        self.pi.write(18,0)
            else:
	        GPIO.output(14,0)
	        GPIO.output(15,0)
	        GPIO.output(17,0)
	        GPIO.output(18,0)
        time.sleep(0.1)

# it doesnt need it but we have a global handler flag to exit cleanly should we do other things
#
MODE = 1
def handler():
    global MODE
    MODE = 0

if __name__=='__main__':
    print("start joystick reader")
    signal.signal(signal.SIGUSR1, handler)                             
    signal.signal(signal.SIGUSR2, handler)                             
    signal.signal(signal.SIGTERM, handler)                             
    try:
        while MODE == 1:
            folo = Folo()
    except KeyboardInterrupt:
        pass
    del folo 