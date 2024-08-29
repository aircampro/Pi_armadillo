# sudo pip3 install evdev
# sudo apt-get install mpg321
#
# use the bluebutton to change the volume
#
import evdev
import subprocess
import time

while True:
    try:
        # ls /dev/input this is the events shown with a list and the number is the number after installing i.e. 3
        device = evdev.InputDevice('/dev/input/event3')
        print(device)

        for event in device.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                if event.value == 1:                    # 0:KEYUP, 1:KEYDOWN
                    print(event.code)

                    if event.code == evdev.ecodes.KEY_VOLUMEUP:
                        subprocess.Popen(['mpg321', 'output.mp3'])

                    if event.code == evdev.ecodes.KEY_ENTER:
                        subprocess.Popen(['amixer', 'sset', 'PCM', '10%-', '-M'])
    except:
        print('Retry...')
        time.sleep(1)