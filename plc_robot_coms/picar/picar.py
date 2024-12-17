#  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
# / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
#| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
# \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
#                  (____/ 
# Osoyoo Raspberry Pi Obstacle Avoidance auto driving
# tutorial url: https://osoyoo.com/?p=33554
# modified by ACP 17/12/2024
#
import time
import RPi.GPIO as GPIO

# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pwm =  PCA9685(i2c_bus)

# Set frequency to 50hz, good for servos.
pwm.frequency = 50
high_speed = 0xAFFF                                                     # Max pulse length out of 4096
mid_speed = 0x8FFF                                                      # Middle pulse length out of 4096
low_speed = 0x6FFF                                                      # low pulse length out of 4096
short_delay = 0.2
long_delay = 0.25
extra_long_delay = 0.6
ob_range = 30
ob_close = 10

# Define GPIO to use on Pi
GPIO_TRIGGER = 20
GPIO_ECHO    = 21
#define L298N(Model-Pi motor drive board) GPIO pins
IN1 = 23                                                                #left motor direction pin
IN2 = 24                                                                #left motor direction pin
IN3 = 27                                                                #right motor direction pin
IN4 = 22                                                                #right motor direction pin
ENA = 0                                                                 #left motor speed PCA9685 port 0
ENB = 1                                                                 #right motor speed PCA9685 port 1
sensor1= 5                                                              # No.1 sensor from far left
sensor2= 6                                                              # No.2 sensor from left
sensor3= 13                                                             # middle sensor
sensor4= 19                                                             # No.2 sensor from right
sensor5= 26                                                             # No.1 sensor from far  right
sts1=0
sts2=0
sts3=0
sts4=0
sts5=0

# move ultrasonic to various positions to look for obstacles
servo_lft = 135                                                         # ultrasonic sensor facing 45 degree left
servo_ctr = 90                                                          # ultrasonic sensor facing front
servo_rgt = 45                                                          # ultrasonic sensor facing 135 degree right

# Set pins as output and input
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)                                       # Trigger
GPIO.setup(GPIO_ECHO,GPIO.IN)                                           # Echo
GPIO.setup(IN1, GPIO.OUT)   
GPIO.setup(IN2, GPIO.OUT) 
GPIO.setup(IN3, GPIO.OUT)   
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(sensor1, GPIO.IN)   
GPIO.setup(sensor2, GPIO.IN)
GPIO.setup(sensor3, GPIO.IN)   
GPIO.setup(sensor4, GPIO.IN)
GPIO.setup(sensor5, GPIO.IN) 

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, False)

def changespeed(left_speed, right_speed):
    pwm.channels[ENA].duty_cycle = left_speed
    pwm.channels[ENB].duty_cycle = right_speed

def stopcar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(0,0)

def backward(speed_left, speed_right):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(speed_left, speed_right)
 
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
	
def forward(speed_left, speed_right):
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    changespeed(speed_left, speed_right)
    #following two lines can be removed if you want car make continuous movement without pause
    #time.sleep(short_delay) 
    #stopcar()
	
def turnRight(speed_left, speed_right):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(speed_left, speed_right)
    #following two lines can be removed if you want car make continuous movement without pause
    #time.sleep(short_delay) 
    #stopcar()
	
def turnLeft(speed_left, speed_right):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    changespeed(speed_left, speed_right)	
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()

def measure():
    # This function measures a distance using the ultrasonic sensor
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO)==0:
        start = time.time()
    while GPIO.input(GPIO_ECHO)==1:
        stop = time.time()
    elapsed = stop-start
    distance = (elapsed * 34300)/2
    return distance

print('Moving servo on channel 15, press Ctrl-C to quit...')
sts1=0
sts2=0
sts3=0
print("facing left")
kit.servo[15].angle = servo_lft
time.sleep(1)
print("facing right")
kit.servo[15].angle = servo_rgt
time.sleep(1)
print("facing center")
kit.servo[15].angle = servo_ctr
time.sleep(3)

sharp_r=0                                                               # memorise how much we are going sharp right
sharp_l=0                                                               # same for sharp left

try:
    print("begin obstacle avoidance while follwoing the track...")

    while True:
        print("STEP1 : read the ultrasonic at various angles and avoid any obstacle")
        kit.servo[15].angle = servo_lft
        time.sleep(0.3)
        distance = measure()
        print(distance)
        sts1 =  0 if distance>ob_range else 1
        if sts1 == 1 and distance<ob_close:
           sts1 = 2
           
        kit.servo[15].angle = servo_ctr
        time.sleep(0.3)
        distance = measure()
        sts2 =  0 if distance>ob_range else 1

        kit.servo[15].angle = servo_rgt
        time.sleep(0.3)
        distance = measure()
        sts3 =  0 if distance>ob_range else 1
        if sts3 == 1 and distance<ob_close:
           sts3 = 2
                   
        sensorval = ''.join([str(sts1), str(sts2), str(sts3)])

        if sensorval=="100":
            print(sensorval+" slight right")
            forward(high_speed,0)                                       # slight right turn
            time.sleep(long_delay)  
            sharp_r += 1
            stopcar()
            time.sleep(short_delay)	
            
        elif sensorval=="001":
            print(sensorval+" slight left")
            forward(0,high_speed)                                       # slight left turn
            time.sleep(long_delay)  
            sharp_l += 1 
            stopcar()
            time.sleep(short_delay)	

        elif sensorval=="110" or sensorval=="210" or sensorval=="200":
            print(sensorval+" sharp right")
            turnRight(high_speed, low_speed)                            # sharp right turn
            time.sleep(long_delay) 
            sharp_r += 2 
            stopcar()
            time.sleep(short_delay) 
            
        elif sensorval=="011" or sensorval=="012" or sensorval=="002":	
            print(sensorval+" sharp left")
            turnLeft(low_speed,high_speed)                              # sharp left turn
            time.sleep(long_delay) 
            sharp_l += 2  
            stopcar()
            time.sleep(short_delay) 
            
        elif sensorval=="111" or sensorval=="101" or or sensorval=="202" or or sensorval=="212":	
            print(sensorval+" back to left")
            backward(low_speed,low_speed)                               # go backward
            time.sleep(short_delay) 
            turnRight(high_speed, high_speed)                           # back to left side 
            sharp_r += 3
            time.sleep(extra_long_delay)  
            stopcar()
            time.sleep(short_delay) 
            
        elif sensorval=="010":
            print(sensorval+" backward")
            backward(mid_speed,mid_speed)                               # go backward
            time.sleep(long_delay)  
            sharp_l = 0
            sharp_r = 0
            stopcar()
            time.sleep(short_delay)	
            
        elif sensorval=="000":                                         # no obstacles
            print(sensorval+" forward")
            if sharp_r >= 2:                                            # we went a lot right turn back a bit
                turnLeft(low_speed, low_speed)	
                sharp_r -= 1
            if sharp_l >= 2:                                            # we went a lot left turn back a bit
                turnRight(low_speed, low_speed)	
                sharp_l -= 1                			
            time.sleep(short_delay)	
            forward(mid_speed,mid_speed)                                # go forward
            time.sleep(long_delay)  
            stopcar()
            time.sleep(short_delay)	

        # now look at track sensor and try to keep on it
        #
        print("STEP2 : read the track and try to keep on it")
		sts1 =  0 if GPIO.input(sensor1) else 1
		sts2 =  0 if GPIO.input(sensor2) else 1
		sts3 =  0 if GPIO.input(sensor3) else 1
		sts4 =  0 if GPIO.input(sensor4) else 1
		sts5 =  0 if GPIO.input(sensor5) else 1

		sensorval = ''.join([str(sts1), str(sts2), str(sts3), str(sts4), str(sts5)])
		print(sensorval)

		if sensorval=="10000"  or sensorval=="01000" or sensorval=="11000":
			turnLeft(low_speed,mid_speed)                               # The black line left, sharp left turn
			time.sleep(long_delay)  
			stopcar()
			#time.sleep(short_delay) 
		
		elif sensorval=="01100"  or sensorval=="11100"  or sensorval=="11110" :
			turnLeft(0,high_speed)                                      # The black line left,  left turn
			time.sleep(long_delay)  
			stopcar()
			#time.sleep(short_delay) 	
			
		elif sensorval=="00001"  or sensorval=="00010" or sensorval=="00011":
			turnRight(mid_speed,low_speed)                              # The black line is  on the Left of the car, need  Left turn 
			print("right turn")
			time.sleep(long_delay)  
			stopcar()
			#time.sleep(short_delay)	
		
		elif sensorval=="00110" or  sensorval=="00111" or sensorval=="01111":
			forward(mid_speed,low_speed)                                # slight right turn
			time.sleep(long_delay)  
			stopcar()
			#time.sleep(short_delay)	

		elif sensorval=="00100"  or sensorval=="01110":
			forward(mid_speed,mid_speed)                                # right turn
			time.sleep(long_delay)  
			stopcar()
			#time.sleep(short_delay) 
		
		elif sensorval=="11111" :
			stopcar()                                                   #The car front touch stop line, need stop
			                        
except KeyboardInterrupt:
  # User pressed CTRL-C
  # Reset GPIO settings
  GPIO.cleanup()
