#!/usr/bin/env python3
# codint: utf-8
# Example of a simple car
#
# to remove tab space $> expand -i -t 4 file_name > new_file
#
import time
import RPi.GPIO as GPIO

#default GPIO pin set-up
DEF_L293D_EN1 = 17
DEF_L293D_IN1 = 27
DEF_L293D_IN2 = 22

# wheeled traction
#
class wheeled_traction():
    """  Raspi wheeled traction bot
    """
    def __init__(self, hz=50, GPIO_BCM_L293D_EN1=DEF_L293D_EN1, GPIO_BCM_L293D_IN1=DEF_L293D_IN1, GPIO_BCM_L293D_IN2=DEF_L293D_IN2):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_BCM_L293D_EN1, GPIO.OUT)
        GPIO.setup(GPIO_BCM_L293D_IN1, GPIO.OUT)
        GPIO.setup(GPIO_BCM_L293D_IN2, GPIO.OUT)
        GPIO.output(GPIO_BCM_L293D_EN1, GPIO.LOW)
        GPIO.output(GPIO_BCM_L293D_IN1, GPIO.LOW)
        GPIO.output(GPIO_BCM_L293D_IN2, GPIO.LOW)
        self.pwm_l293d_in1 = GPIO.PWM(GPIO_BCM_L293D_IN1, hz)
        self.pwm_l293d_in2 = GPIO.PWM(GPIO_BCM_L293D_IN2, hz)
        self.pwm_l293d_in1.start(0)
        self.pwm_l293d_in2.start(0)
        self.on = False
        print('using pins {}, {}, {}'.format(GPIO_BCM_L293D_EN1, GPIO_BCM_L293D_IN1, GPIO_BCM_L293D_IN2))
        print('pulse width modulation {} Hz'.format(hz))

    def destroy(self):
        """destroy
        """
        self.stop()
        GPIO.cleanup()

    def start(self, button_type, button_option):
        """start
        """
        if self.on is True:
            print('skip button event')
            return
        if button_type == 'BUTTON_TYPE_UP':
            GPIO.output(GPIO_BCM_L293D_EN1, GPIO.HIGH)
            self.pwm_l293d_in1.ChangeDutyCycle(button_option)
            self.pwm_l293d_in2.ChangeDutyCycle(0)
            self.on = True
        elif button_type == 'BUTTON_TYPE_DOWN':
            GPIO.output(GPIO_BCM_L293D_EN1, GPIO.HIGH)
            self.pwm_l293d_in1.ChangeDutyCycle(0)
            self.pwm_l293d_in2.ChangeDutyCycle(button_option)
            self.on = True
        else:
            self.on = False

    def stop(self):
        """stop
        """
        GPIO.output(GPIO_BCM_L293D_EN1, GPIO.LOW)
        self.pwm_l293d_in1.ChangeDutyCycle(0)
        self.pwm_l293d_in2.ChangeDutyCycle(0)
        self.on = False

    def loop(self):
        """loop
        """
        while True:
            self.start('BUTTON_TYPE_UP', 50)
            print('>>> BUTTON_TYPE_UP')
            time.sleep(3)
            self.stop()
            print('>>> STOP')
            time.sleep(3)
            self.start('BUTTON_TYPE_DOWN', 50)
            print('>>> BUTTON_TYPE_DOWN')
            time.sleep(3)
            self.stop()
            print('>>> STOP')
            time.sleep(3)

# gpio pin for steering angle
DEF_SERVO = 18
# Controllable range of DC
DC_MIN = 2.5
DC_MAX = 12.0

# steering servo
class steering():
    """Raspi steering using angle servo
    """

    def __init__(self, GPIO_BCM_SERVO=DEF_SERVO, hz=50):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_BCM_SERVO, GPIO.OUT)
        GPIO.output(GPIO_BCM_SERVO, GPIO.LOW)
        self.pwm_servo = GPIO.PWM(GPIO_BCM_SERVO, hz)
        self.pwm_servo.start(0)
        self.on = False
        print('using pin {}'.format(GPIO_BCM_SERVO))
        print('pulse width modulation {} Hz'.format(hz))

    def destroy(self):
        """destroy
        """
        self.pwm_servo.stop()
        GPIO.cleanup()

    def angle(self, angle):
        """angle
        """
        duty = DC_MIN + (DC_MAX - DC_MIN) * (angle + 90) / 180
        self.pwm_servo.ChangeDutyCycle(duty)

    def start(self, angle):
        """start
        """
        if self.on is True:
            print('skip button event')
            return
        self.angle(angle)
        self.on = True

    def stop(self, angle):
        """stop
        """
        self.angle(angle)
        self.on = False

    def loop(self):
        """loop
        """
        while True:
            self.start(-30)
            print('>>> BUTTON_TYPE_RIGHT')
            time.sleep(3)
            self.stop(0)
            print('>>> STOP')
            time.sleep(3)
            self.start(35)
            print('>>> BUTTON_TYPE_LEFT')
            time.sleep(3)
            self.stop(0)
            print('>>> STOP')
            time.sleep(3)

if __name__ == '__main__':
    raspi_car = wheeled_traction()
    raspi_steer = steering()
    try:
        print('starting demo......')
        raspi_car.loop()
        raspi_steer.loop()
    except KeyboardInterrupt:
        raspi_car.destroy()
        raspi_steer.destroy()
        print('stoping demo......')