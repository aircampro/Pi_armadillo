#!usr/bin/env python
# -*- coding: utf-8 -*-
#
# =============================================================================================================================
# $pip install speechRecognition
# $pip install pyttsx3
# $pip install pywhatkit
# $pip install wikipedia
#
# $sudo apt-get install python-pip python-dev
# $pip is then used to install dronekit and dronekit-sitl. Mac and Linux may require you to prefix these commands with sudo:
# $pip install dronekit
# $pip install dronekit-sitl
# =============================================================================================================================
#
"""
  Listen to speech and then move the drone using the dronekit api, speak back name from wiki or play a tune
"""
import speech_recognition as sr
import pyttsx3
import pywhatkit
import wikipedia
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil                                                              # Needed for command message definitions
import time
import math

# create globals for the speach recognizer and talk back actions 
recognizer = sr.Recognizer()
Bob = pyttsx3.init()
voices = Bob.getProperty('voices')
Bob.setProperty('voice', voices[1].id)
g_step = 0
g_actions = []

# Connect to the drone 
vehicle = connect('/dev/ttyACM0,57600', wait_ready=True)

"""
Functions to make it easy to convert between the different frames-of-reference. In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take 
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close 
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0                                                                   #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

The methods include:
* goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
* goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
    MAV_FRAME_BODY_NED frame
* goto - A convenience function that can use Vehicle.simple_goto (default) or 
    goto_position_target_global_int to travel to a specific position in metres 
    North and East from the current location. 
    This method reports distance to the destination.
"""

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,                                               # time_boot_ms (not used)
        0, 0,                                            # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000,                              # type_mask (only speeds enabled)
        aLocation.lat*1e7,                               # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7,                               # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt,                                   # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0,                                               # X velocity in NED frame in m/s
        0,                                               # Y velocity in NED frame in m/s
        0,                                               # Z velocity in NED frame in m/s
        0, 0, 0,                                         # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                            # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    vehicle.send_mavlink(msg)                            # send command to vehicle

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                                                # time_boot_ms (not used)
        0, 0,                                             # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,              # frame
        0b0000111111111000,                               # type_mask (only positions enabled)
        north, east, down,                                # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0,                                          # x, y, z velocity in m/s  (not used)
        0, 0, 0,                                          # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                             # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    vehicle.send_mavlink(msg)                             # send command to vehicle

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED":                                          #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01:                              #Just below target, in case of undershoot.
            print("Reached target")
            Bob.say("at target")
            break;
        time.sleep(2)

"""
Functions that move the vehicle by specifying the velocity components in each direction.
The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the vehicle
orientation.

The methods include:
* send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
* send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                                                        # time_boot_ms (not used)
        0, 0,                                                     # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,                      # frame
        0b0000111111000111,                                       # type_mask (only speeds enabled)
        0, 0, 0,                                                  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,                       # x, y, z velocity in m/s
        0, 0, 0,                                                  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                                     # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,                                                                # time_boot_ms (not used)
        0, 0,                                                             # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,                # frame
        0b0000111111000111,                                               # type_mask (only speeds enabled)
        0,                                                                # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,                                                                # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0,                                                                # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,                                                       # X velocity in NED frame in m/s
        velocity_y,                                                       # Y velocity in NED frame in m/s
        velocity_z,                                                       # Z velocity in NED frame in m/s
        0, 0, 0,                                                          # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                                             # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    for x in range(0,duration):                                           # send command to vehicle on 1 Hz cycle
        vehicle.send_mavlink(msg)
        time.sleep(1) 
''' talk says the text read in for confirmation '''
def talk(text):
    Bob.say(text)
    Bob.runAndWait()
'''take_command listens to the speach said to get the command and put it in the actions list for the drone motion'''
def take_command():
    try:
        with sr.Microphone() as source:
            print('Listening.....')
            voice = recognizer.listen(source)
            command = recognizer.recognize_google(voice)
            command = command.lower()    
    except:
        pass
    return command
'''listen2commands for the robots instructions or play a song or look up a person on wikipedia'''
def listen2commands():
    command = take_command()
    print(command)
    if 'play' in command:                                                # plays the song specified in the speech      
        song = command.replace('play', '')
        talk('playing' + song)
        pywhatkit.playonyt(song)                                         
    elif 'who' in command:                                               # describes the person
        person = command.replace('who', '')
        info = wikipedia.summary(person, 5)
        talk(info)
        print(info)
    elif 'move' in command:
        if 'north' in command:
            g_actions.append("north")
        elif 'south' in command:
            g_actions.append("south")
        if 'east' in command:
            g_actions.append("east")
        elif 'west' in command:
            g_actions.append("west")
    elif 'get' in command:
        if 'gps' in command:
            gps = f"GPS is {vehicle.gps_0}"
            talk(gps)
        if 'battery' in command: 
            bat = f"battery is {vehicle.battery}" 
            talk(bat)   
        if 'status' in command: 
            stat = f"status is {vehicle.system_status.state}" 
            talk(stat) 
        if 'mode' in command: 
            md = f"mode is {vehicle.mode.name}" 
            talk(md)            
    elif 'end' in command:
        g_step = 2	
    elif 'stop' in command:
        g_step = 3		
    else:
        talk('please say the command again.....')
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    talk("initializing") 
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    talk("arming motors") 
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    saym = f"taking off to {aTargetAltitude} meters"
    talk(saym)    
    vehicle.simple_takeoff(aTargetAltitude)                                   # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    # after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            talk("Reached target altitude") 
            break
        time.sleep(1)
'''armtakeoff arm and take-off'''
def armtakeoff(gs=5,ta=5):
    arm_and_takeoff(ta)                                                      # Arm and take of to altitude of ta meters
    gs_msg = f"set ground speed to {gs}"
    talk(gs_msg) 
    vehicle.groundspeed=gs
'''performcommands performs the sequence of actions requested to the drone'''
def performcommands(NORTH = 1, SOUTH = -1, EAST = 1, WEST = -1):
    for action in g_actions:                                        # do sequentially each action in the voice list
        if not action.find("north") == -1: 
            goto(NORTH,0)
        elif not action.find("south") == -1: 
            goto(SOUTH,0)		
        elif not action.find("east") == -1: 
            goto(0,EAST)		
        elif not action.find("west") == -1: 	
            goto(0,WEST)
    g_step = 1                                                       # go back to listening
    g_actions = []                                                   # reset the voice command list

if __name__ == "__main__":
    try:
        armtakeoff()                                                 # arm drone & takeoff
        while g_step < 3:                                            # while we didnt say stop
            g_step = 1
            while g_step == 1:                                       # listen to commands
                listen2commands()
            while g_step == 2:                                       # perform actions
                performcommands()
    except( KeyboardInterrupt, SystemExit):	
        print( "SIGINT" )
    print("Setting LAND mode...")
    talk("landing") 
    vehicle.mode = VehicleMode("LAND")                               # now land
    vehicle.close()	                                                 # close the connection to the drone	
