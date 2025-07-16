#!/usr/bin/env python3
# coding: utf-8
# Example of moving GWS S03T servo motor for angle control
#
import RPi.GPIO as GPIO
import time
import atexit

# GP Pin number of servo input
DEF_SERVO_GPIO = 23

# Servo drive Hz
DEF_HZ = 50

# Controllable range of DC
DC_MIN = 2
DC_MAX = 12

class Servo():

    def __init__(self, SERVO_GPIO=DEF_SERVO_GPIO, HZ=DEF_HZ):
        super(Servo, self).__init__("servo", 1)
        # Specify pin number as General Purpose Pin number
        GPIO.setmode(GPIO.BCM)
        # Set servo pin
        GPIO.setup(SERVO_GPIO, GPIO.OUT)
        # Initialize servo
        self.__init_servo()
        # Register cleanup handler at exit
        atexit.register(self.__cleanup)

    # Clean up GPIO on exit
    def __cleanup(self):
        self.servo.ChangeDutyCycle(DC_MIN)
        time.sleep(0.5)
        self.servo.stop()
        GPIO.cleanup(SERVO_GPIO)

    """
    Initialize GPIO servo.
    """
    def __init_servo(self):
        self.servo = GPIO.PWM(SERVO_GPIO, HZ)
        self.servo.start(0.0)

    # Concete methods of super class
    def operate(self, args):
        angle = float(args[0])
        self.rotate_servo(angle=angle)
    
    def check_args(self, args):
        try:
            angle = float(args[0])
            return self.__check_angle(angle)
        except:
            return False

    """
    Convert angle to duty cycle.
    @param angle : angle to convert
    """
    @classmethod
    def __angle2dc(self, angle):
        return DC_MIN + (DC_MAX - DC_MIN) * angle / 180.0

    """
    Check angle arguments.
    @param angle : angle to check
    """
    def __check_angle(self, angle):
        return 0.0 <= angle < 180.0

    """
    Rotate servo with specified angle.
    @param angle : target angle
    """
    def rotate_servo(self, angle=90.0):
        if self.__check_angle(angle):
            self.angle = angle
            dc = Servo.__angle2dc(angle)
            print("dc = {0}".format(dc))
            # Start servo
            #self.servo.start(0.0)
            self.servo.ChangeDutyCycle(dc)
            time.sleep(0.5)
            # Stop servo to prevent vibration
            #self.servo.stop()
            
if __name__ == "__main__":

    ss = Servo()
    ss.rotate_servo(45.0)
    time.sleep(10)
    ss.rotate_servo()
    