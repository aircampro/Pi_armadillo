#
# Example mult-threaded webserver app which puts location on a map ref material on setting up the gps module is shown below
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
# fastAPI webserver
import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from starlette.templating import _TemplateResponse
from fastapi.staticfiles import StaticFiles

import os
import threading
import time

import serial
from geopy.distance import geodesic

from micropy_gps import micropyGPS

import sys

# make global gps position object
#
gps = micropyGPS.MicropyGPS()
gps.coord_format = 'dd'

def run_gps():
    """
    get the GPS data in real time
    :return: None
    """

    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline()
    while True:

        sentence = s.readline().decode('utf-8')
        if sentence[0] != '$':                            # valid NMEA0183
            continue

        for x in sentence:
            gps.update(x)


# Class wrapper for threading 
# you can also kill task e.g. if op_task.is_alive(): op_task.raise_exception()
#
class twe(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')

# -------------- fastAPI webserver ---------------------------------
app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
templates = Jinja2Templates(directory='templates')

# get page showing data and a link to the position map
@app.get('/')
async def getItemDetail(request: Request) -> _TemplateResponse:
    return templates.TemplateResponse('item_detail.html', {'request': request, 'title': 'position detail', 'lat': str(gps.latitude[0]), 'lon' : str(gps.longitude[0]), 'alt' : str(gps.altitude), 'spd' : str(gps.speed[2])})

# get the html page directly showing the position
@app.get("/htmldirect", response_class=HTMLResponse)
async def ui_index(request: Request):
    return templates.TemplateResponse(request=request, name="pos.html" )

# make a webmap for the position data
#    
def make_web_map():
    output_dir = "./templates"
    os.makedirs(output_dir, exist_ok=True)
    tooltip = "Dist frm target "
    # this is the coordinate of the target you are looking for read from the command line or defaulted
    if (len(sys.argv[0]) > 1):
        try:
            target_coordinate = [float(sys.argv[1]), float(sys.argv[2])]
        except Exception as e:
            print("error : ", e)
            print("first two arguments must be float numbers (lat/lon) - using default for location")
            target_coordinate = [35.690921, 139.700258]                                                    # sapporo city        
    else:    
        target_coordinate = [35.690921, 139.700258]               

    while True:

        if gps.clean_sentences > 20:

            h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
            print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
            print('lat/lon: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
            print('alt: %f' % gps.altitude)
            print('speedド: %f' % gps.speed[2])
            print('sats: %s' % gps.satellites_used)
            current_coordinate = [gps.latitude[0], gps.longitude[0]]
            distance = geodesic(current_coordinate, target_coordinate).kilometers
            print('distance from target: %s (km)' % distance)
            print('衛星番号: (仰角, 方位角, SN比)')
            for k, v in gps.satellite_data.items():
                print('%d: %s' % (k, v))
            print('')

            time_str = (
                    '20%02d/%02d/%02d %02d:%02d:%02d' %
                    (
                        gps.date[2], gps.date[1], gps.date[0],
                        h, gps.timestamp[1], gps.timestamp[2]
                    )
            )

            ofile = output_dir + "/pos.html"
            popu="<i>position @ "+time_str+"</i>"
            # map with location as centre
    		# mymap = folium.Map(location=[gps.latitude[0],gps.longitude[0]], zoom_start=7, width=800, height=600, tiles='openstreetmap')
            # whole map
            mymap = folium.Map(location=[target_coordinate[0], target_coordinate[1]], zoom_start=7, tiles='openstreetmap')
            tooltip2 = tootip + str(distance) + " km"
            folium.Marker(
                location=[gps.latitude[0],gps.longitude[0]],
                popup=popu,  
                tooltip=tooltip2,  
                icon=folium.Icon(color="red", icon="home")  
            ).add_to(mymap)
    		mymap.save(ofile)

        time.sleep(1.0)

if __name__ == '__main__':

    gps_thread = twe(name = 'Thread GPS', target=run_gps, args=(), kwargs={})
    # start the thread to receive our position
    gps_thread.start()
    map_thread = twe(name = 'Thread MAP', target=make_web_map, args=(), kwargs={})
    # start the thread to make webmap
    map_thread.start()
    # start webserver
    uvicorn.run('web:app', host='127.0.0.1', port=8000, log_level='info', reload=True)	
    gps_thread.join()
    map_thread.join()

