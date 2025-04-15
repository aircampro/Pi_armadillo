# lucid control driver example linux on raspberry pi please download the driver LucidIoCtrl from the manufacturer site
#
#
# Linux Examples for lucid linux driver https://www.lucid-control.com/
# the module is connected on usb to example /dev/ttyACM0 for the rtd

# set up rtd module
LucidIoCtrl –d/dev/ttyACM0 –c0 –sinRtMode=standard –p 
LucidIoCtrl –d/dev/ttyACM0 –c0 –sinRtSetupTime=25 –p

# Set number or oversampling cycles to a value of 8 for channel 0 and make the setting persistent.
LucidIoCtrl –d/dev/ttyACM0 –c0 –sinRtNrSamples=8 –p

# env temp comp set to 1 for chan 0
LucidIoCtrl –d/dev/ttyACM0 –c0 –sinRtTempComp=1 -p
# env temp comp set to 1.5 for chan 1
LucidIoCtrl –d/dev/ttyACM0 –c1 –sinRtTempComp=1.5 -p

# Reading the values of the first 4 input channels of a temperature module
temp_read=`LucidIoCtrl –d/dev/ttyACM0 –tT –c0,1,2,3 –r`
echo $temp_read 
# example -> CH0:25.000 CH1:25.000 CH2:25.000 CH3:25.000
echo "print first channel value"
first_chan_val=`echo $temp_read | awk '{ print $1 }' | awk '-F:' '{ print $2 }'`
echo $first_chan_val
echo "print second channel value"
sec_chan_val=`echo $temp_read | awk '{ print $2 }' | awk '-F:' '{ print $2 }'`
echo $sec_chan_val
echo "print third channel value"
third_chan_val=`echo $temp_read | awk '{ print $3 }' | awk '-F:' '{ print $2 }'`
echo $third_chan_val
echo "print fourth channel value"
fourth_chan_val=`echo $temp_read | awk '{ print $4 }' | awk '-F:' '{ print $2 }'`
echo $fourth_chan_val
# first and fourth averaged
avg14=`echo 'scale=4; ' $first_chan_val '* 0.1 +' $fourth_chan_val '* 0.1 / 2.0'  | bc`
echo $avg14

# read 8 channel rtd
temp8_read=`LucidIoCtrl –d/dev/ttyACM0 –c0,1,2,3,4,5,6,7 –tT –r`
echo "print eigth channel value"
eigth_chan_val=`echo $temp8_read | awk '{ print $8 }' | awk '-F:' '{ print $2 }'`
echo $eigth_chan_val

# voltage read on ACM2
volts4=`LucidIoCtrl -drs485:/dev/ttyACM2:11 -tV -c0,1,2,3 -r`
echo $volts4 
# CH00:5.000 CH01:5.000 CH02:5.000 CH03:5.000

# volts set
LucidIoCtrl –drs485:/dev/ttyACM0:11 –tV –c0,1,2,3 – w5.000,2.500,1.250,0.625

# for a mA unit on usb 1 Set output channel 0 to 10 mA:
LucidIoCtrl –drs485:/dev/ttyACM1:11 –c0 –tC –w10

# read back
ma4=`LucidIoCtrl –drs485:/dev/ttyACM1:11 –tL –c0,1,2,3 –r`
#-> CH0:00 CH1:00 CH2:00 CH3:00

# relay output module on ACM3
# Setting output channel number 0 to “1”
./LucidIoCtrl –d/dev/ttyACM3 –tL –c0 –w1
sleep 2
# Resetting output channel number 0 to “0”
./LucidIoCtrl –d/dev/ttyACM3 –tL –c0 –w0
# Reading the outputs of the first 4 channels back
do4_in=`./LucidIoCtrl –d/dev/ttyACM3 –tL –c0,1,2,3 –r`
echo $do4_in
#-> CH0:00 CH1:00 CH2:00 CH3:00

# now do pwm with the dot module

# Configure output channel 0 for Duty-Cycle mode
#
LucidIoCtrl –d/dev/ttyACM3 –c0 –soutDiMode=dutyCycle
# Start processing of PWM signal for output channel 0
LucidIoCtrl –d/dev/ttyACM3 –c0 –tL –w1
sleep 4
# By default, the module is configured with TCycle = 1 s and DutyCycle = 50%. The output
# channel is switched 500 ms to “1” and 500 ms to “0”.
# Changing TCycle to 2 s
LucidIoCtrl –d/dev/ttyACM3 –c0 –soutDiCycleTime=2000000
sleep 4
# The output is now 1 s switched on and 1 s switched of
# Change DutyCycle to 75%
LucidIoCtrl –d/dev/ttyACM3 –c0 –soutDiDutyCycle=750
sleep 4
# Disable processing of output channel 0
LucidIoCtrl –d/dev/ttyACM3 –c0 –tT –w0
sleep 2

# By using On-Off Mode time-controlled switching functions (e.g.
# used in timing relays) can be realized.

# Configure output channel 0 for On-Off mode
LucidIoCtrl –d/dev/ttyACM3 –c0 –soutDiMode=onoff

# By default, TOnDelay and TOnHold are set to 1s.
# After writing a “1” to the output value of channel 0 the output will be set after 1s to “1”
# returning to “0” after 1s more.
# Start processing of output channel 0
LucidIoCtrl –d/dev/ttyACM3 –c0 –tL –w1
sleep 8
# fire again
LucidIoCtrl –d/dev/ttyACM3 –c0 –tL –w0
sleep 2
LucidIoCtrl –d/dev/ttyACM3 –c0 –tL –w1


