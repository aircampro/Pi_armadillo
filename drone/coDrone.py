# https://codrone.robolink.com/edu/python/
# https://github.com/RobolinkInc/codrone-edu-python-examples/tree/main
#
from codrone_edu.drone import *

drone = Drone()
drone.pair()

drone.takeoff()
# maintain a distance of 60cm from an object once detected for 10 seconds
drone.keep_distance(10, 60)
print(drone.get_angular_speed_x())
print(drone.get_accel_z())
print(drone.get_angle_y())
print(drone.get_flow_velocity_x())
battery = drone.get_battery()
print(battery)
drone.hover(3) #Recommended to have a hover() or time.sleep(1) before landing
print(drone.get_movement_state()) # "ModeMovement.Hovering" after takeoff
drone.emergency_stop()
drone.land()

drone.close()