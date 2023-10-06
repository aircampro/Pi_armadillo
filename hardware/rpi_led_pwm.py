#!/usr/bin/env python
 
"""
LED Pwm hardware driver 
"""

import wiringpi

LED_PORT = 18                   # GPIO 18
PWM_RANGE = 1024
PWM_FREQUENCY = 500             # Hz
CLOCK_BASE = int(18750 / PWM_FREQUENCY)

# Hardware PWM
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(LED_PORT, wiringpi.GPIO.PWM_OUTPUT)
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
wiringpi.pwmSetRange(PWM_RANGE)
wiringpi.pwmSetClock(CLOCK_BASE)

for duty_cycle in (5, 20, 50, 80, 100):
    print(duty_cycle)
    wiringpi.pwmWrite(LED_PORT, int(PWM_RANGE * duty_cycle / 100))
    wiringpi.delay(2000)


wiringpi.pwmWrite(LED_PORT, 0)

print('done')