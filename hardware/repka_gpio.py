#  Controls the i/o of the Repka pi ref:- https://repka-pi.ru/
#
#  sudo apt-get update
#  sudo apt-get install python3-dev python3-setuptools git
#  git clone https://gitflic.ru/project/repka_pi/repkapigpiofs.git
#  cd repkapigpiofs
#  sudo python3 setup.py install
#
#  ref :- https://gitflic.ru/project/repka_pi/repkapigpiofs
#
import RepkaPi.GPIO as GPIO
import RepkaPi.PWM_A as PWM_A

if __name__ == "__main__":
    GPIO.setboard(GPIO.REPKAPI3) # REPKAPI3 or REPKAPI4.
    GPIO.setmode(GPIO.BOARD)
    GPIO.output(5, 1)            # drive output 5 to true
    time.sleep(4)
    GPIO.output(5, 0)            # drive output 5 to false
	
	# pwm for motor control
    chip=1 
    pin=2 
    freq=10
    duty_cyc_pcent=50
    invert_polarity=False
    pwm1 = PWM_A(chip, pin, freq, duty_cyc_pcent, invert_polarity) # create pulse width modulator
    pwm1.start_pwm()            # start pwm1
    time.sleep(5)
    pwm1.stop_pwm()
    pwm1.start_pwm()
    time.sleep(2)
    pwm1.change_frequency(5)    # change freq
    time.sleep(2)
    pwm1.duty_cycle(70)
    time.sleep(2)
    pwm1.pwm_polarity()         # invert polarity
    time.sleep(5)
    pwm1.stop_pwm()
    pwm1.pwm_close()