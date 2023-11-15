#!/usr/bin/python3
#
# useful functions relating to vehicles
#
import math

# A function for computing a virtual light sensor input for treating the goal as a light source.
# The x_offset of the first argument is the x-coordinate of the virtual optical sensor from the base_link
# The y_offset of the second argument is the y-coordinate of the virtual optical sensor from the base_link
# The goal_point of the third argument is the coordinates of the goal point when viewed in base_link
#
def emulate_light_sensor( x_offset, y_offset, goal_point_x, goal_point_y ):
    # Determines whether the goal_point is in the field of view, and outputs 0 if it is not
    #
    if math.abs(math.atan2((goal_point_y - y_offset),(goal_point_x - x_offset))) >= (v_light_sensor_view_angle * 0.5):
	return 0.0
    
    # If you have a valid goal attitude
    # Calculate the square root of (goal_point.x - x_offset)^2 (goal_point.y - y_offset)^2 to find the distance between two points
    #
    distance = math.hypot((goal_point_x - x_offset), (goal_point_y - y_offset))

    # We want to maximize the output value of the sensor while avoiding zeroing, so we return 1
    #	
    if distance <= math.epsilon:
        return 1.0
    else:
	ls = (1.0 / (distance * distance))
        if (ls >= 1.0):
            return 1.0
        elif (ls <= 0.0):
            return 0.0
        else:
            return ls

# get speed ftom rotstion speed and wheel radius
#
def get_speed( rotational_speed, wheel_radius ):
    return rotational_speed / (2 * math.pi) * wheel_radius
    
# get twist from left and right wheel rotation speeds and wheel base
#	
def get_twist(left_wheel_rotational_speed, right_wheel_rotational_speed, wheel_base):
    twist_linear_x = (get_speed(right_wheel_rotational_speed) + get_speed(left_wheel_rotational_speed)) * 0.5;
    twist_angular_z = (get_speed(right_wheel_rotational_speed) - get_speed(left_wheel_rotational_speed)) * wheel_base;
    return twist_linear_x, twist_angular_z;
