#
# shows various methods for getting GPS & GNSS using the SIM868 module
# 
import serial
import time
import string
import pynmea2
import sys
# gnss
import json
from sim800l import SIM800L
# use of this library https://github.com/AlexandreFrolov/rover_connect2/tree/main
from rover_connect import RoverConnect

if len(sys.argv) >= 1:
    PORT_CONN=str(sys.argv[1])
else:
    PORT_CONN="/dev/ttyS0"

# returns list of links for maps to the co-ordinates specified
def get_maps(rover, lat, lng):
    map_links = []
	map_link=rover.generate_google_maps_link(lat, lng)
    print(map_link)
	map_links.append(map_link)
	map_link=rover.generate_openstreetmap_link(lat, lng)
    print(map_link)
	map_links.append(map_link)
	map_link=rover.generate_yandex_maps_link(lat, lng)
    print(map_link)
	map_links.append(map_link)
    return map_links

got_pos=False		
while True:
    port=PORT_CONN
    ser=serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline().decode('unicode_escape')
    if newdata[0:6] == "$GPRMC":
        newmsg=pynmea2.parse(newdata)
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps = "(latitude, longditude): (" + str(lat) + "," + str(lng) + ")"
        print(gps)
        got_pos=True
    ser.close()

    rover = RoverConnect(port, 'internet.mts.ru')	
    if got_pos:
        links = get_maps(rover, lat, lng)
    gnss_data = rover.get_cgns_data()
    gnss_parsed_data = rover.parse_cgns_info(gnss_data)
    # "gnss_status", "fix_status", "utc_datetime",
    # "latitude", "longitude", "msl_altitude",
    # "speed", "course", "fix_mode", "reserved_1",
    # "hdop", "pdop", "vdop", "reserved_2", "satellites_in_view",
    # "gnss_satellites_used", "glonass_satellites_used",
    # "reserved_3", "c_n0_max", "hpa", "vpa"
    gnss_parsed_data = json.loads(gnss_parsed_data)
    print(json.dumps(gnss_parsed_data, indent=4))
	dat = json.dumps(gnss_parsed_data)
	lat = dat['latitude']
	lng = dat['longitude']
    speed = dat['speed']
    alt = dat['msl_altitude']
    links2 = get_maps(rover, lat, lng)	
	gsm_gnss_data = rover.getGsmLocations()
	lat = gsm_gnss_data.latitude
    lng = gsm_gnss_data.longitude
    pa = gsm_gnss_data.positioning_accuracy
    links3 = get_maps(rover, lat, lng)
