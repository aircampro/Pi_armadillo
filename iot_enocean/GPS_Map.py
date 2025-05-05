#
# Razpi - GPS simple location map
#
# ref:- https://zenn.dev/kotaproj/books/raspberrypi-tips/viewer/370_kiso_gps
#
# (env) $ git clone https://github.com/inmcm/micropyGPS
# (env) $ cp micropyGPS/micropyGPS.py .
# connection to Akizuki Electronics GPS Kit
#
# https://github.com/kuri-megane/evaluate-gps-qzss/blob/develop/logger.py
# https://akizukidenshi.com/catalog/g/g109991/
# for connection look here https://kuri-megane.hatenablog.jp/entry/2019/09/02/200000 GPIO18 to 1PPS rx-Tx Tx-rx 5v-5v G-G
# $ sudo nano /boot/config.txt
# dtoverlay=pps-gpio,gpiopin=18
# $ sudo nano /etc/modules
# pps-gpio
# $ lsmod | grep pps
# pps_gpio 16384 0
# pps_core 20480 1 pps_gpio
# $ sudo nano /etc/default/gpsd
# START_DAEMON="true"
# USBAUTO="true"
# DEVICES="/dev/ttyS0 /dev/pps0"
# GPSD_OPTIONS="-n"
# $ sudo systemctl enable gpsd.socket
# $ systemctl status gpsd.socket
# ● gpsd.socket - GPS (Global Positioning System) Daemon Sockets
# Loaded: loaded (/lib/systemd/system/gpsd.socket; enabled; vendor preset: enabled)
# Active: active (listening) since Mon 2019-08-19 21:17:05 JST; 1 weeks 6 days ago
# Listen: /var/run/gpsd.sock (Stream)
# [::1]:2947 (Stream)
# 127.0.0.1:2947 (Stream)
# CGroup: /system.slice/gpsd.socket
#
import time
import serial
from micropyGPS import MicropyGPS
import folium

def main():
    # connection to the GPS module
    uart = serial.Serial('/dev/serial0', 9600, timeout = 10)
    # make gps object
    my_gps = MicropyGPS(9, 'dd')
    # you can also change the format like this... my_gps.coord_format = 'dd'
	
    tm_last = 0
    while True:
        sentence = uart.readline()
        if len(sentence) > 0:
            for x in sentence:
                if 10 <= x <= 126:
                    stat = my_gps.update(chr(x))
                    if stat:
                        tm = my_gps.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 100:
                            print('=' * 20)
                            print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                            print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])
                            h = my_gps.timestamp[0] if my_gps.timestamp[0] < 24 else my_gps.timestamp[0] - 24
                            print('%2d:%02d:%04.1f' % (h, my_gps.timestamp[1], my_gps.timestamp[2]))
                            print('altitude: %f' % my_gps.altitude)
                            print('speed: %f' % my_gps.speed[2])
                            print('satelites: %s' % my_gps.satellites_used)
                            for k, v in my_gps.satellite_data.items():
                                print('%d: %s' % (k, v))
                            print('')
                            tooltip="altitude "+my_gps.altitude
                            mymap = folium.Map(location=[my_gps.latitude[0],my_gps.longitude[0]], zoom_start=16, tiles="Stamen Terrain")
                            folium.Marker(
                                location=[my_gps.latitude[0],my_gps.longitude[0]],
                                popup="<i>current position</i>",  
                                tooltip=tooltip,  
                                icon=folium.Icon(color="red", icon="tower")  
                            ).add_to(mymap)
                            mymap.save('current_location.html')

if __name__ == "__main__":
    main()
