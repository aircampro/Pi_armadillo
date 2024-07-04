#!/usr/bin/python3
#
# Use the gripper camera picture to find the center of the circle and pass this to spots gripper as the x,y vector
#
# adapted from example code by:-
# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Use the camera to find center of the circle and pass this to spots gripper
"""
import argparse
import sys
import time

import cv2
import numpy as np
import math

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import estop_pb2, geometry_pb2, image_pb2, manipulation_api_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient

g_image_click = None
g_image_display = None

from scipy import ndimage
# pass an argument from 0 to 4 to select the camera 
# name of camera
# rotation angle
# gripper rotation from the body to our desired grasp
CAM_ROTATION_ANGLE = [
    ('back_fisheye_image', 0, 45),
    ('frontleft_fisheye_image', -78, 45),
    ('frontright_fisheye_image', -102, 47),
    ('left_fisheye_image', 0, 0),
    ('right_fisheye_image', 180, -45)
]
# select your camera from the above list 0 to 4
g_camera=1                                            # 'frontleft_fisheye_image' is selected as the camera 

# A Robot object represents a single robot. Clients using the Boston
# Dynamics API can manage multiple robots, but this tutorial limits
# access to just one. The network address of the robot needs to be
# specified to reach it. This can be done with a DNS name
# (e.g. spot.intranet.example.com) or an IP literal (e.g. 10.0.63.1)
RBT_HOSTNAME="10.0.63.1"

def verify_estop(robot):
    """Verify the robot is not estopped"""

    client = robot.ensure_client(EstopClient.default_service_name)
    if client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
        error_message = 'Robot is estopped. Please use an external E-Stop client, such as the' \
                        ' estop SDK example, to configure E-Stop.'
        robot.logger.error(error_message)
        raise Exception(error_message)


def arm_object_grasp(config):
    """A simple example of using the Boston Dynamics API to command Spot's arm."""

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('ArmObjectGraspClient')
    #robot = sdk.create_robot(config.hostname)
    robot = sdk.create_robot(RBT_HOSTNAME)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), 'Robot requires an arm to run this example.'

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    verify_estop(robot)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    image_client = robot.ensure_client(ImageClient.default_service_name)

    manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info('Powering on robot... This may take a several seconds.')
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # RobotCommandBuilder for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info('Commanding robot to stand...')
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info('Robot standing.')

        # Take a picture with the chosen camera
        robot.logger.info('Getting an image from: %s', config.image_source)
        # image_responses = image_client.get_image_from_sources([config.image_source])        
        image_responses = image_client.get_image_from_sources(CAM_ROTATION_ANGLE[g_camera][0])

        if len(image_responses) != 1:
            print(f'Got invalid number of images: {len(image_responses)}')
            print(image_responses)
            assert False

        image = image_responses[0]                            # take the first picture frame
        num_bytes = 1                                         # Assume a default of 1 byte encodings.
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
            extension = '.png'                           # if we want to write it (not at present)
        else:
            if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                num_bytes = 3
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                num_bytes = 4
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                num_bytes = 1
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
                num_bytes = 2
            dtype = np.uint8
            extension = '.jpg'                          # if we want to write it (not at present)

        img = np.frombuffer(image.shot.image.data, dtype=dtype)
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            try:
                # Attempt to reshape array into an RGB rows X cols shape.
                img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_bytes))
            except ValueError:
                # Unable to reshape the image data, trying a regular decode.
                img = cv2.imdecode(img, -1)
        else:
            img = cv2.imdecode(img, -1)
                
        if options.auto_rotate:
            img = ndimage.rotate(img, CAM_ROTATION_ANGLE[g_camera][1])
                
        # Show the image to the user and wait for them to click on a pixel
        robot.logger.info('Click on an object to start grasping...if it doesnt automatically see it')
        image_title = 'Click to grasp'
        cv2.namedWindow(image_title)
        
        global g_image_click, g_image_display
        g_image_display = img
        cv2.imshow(image_title, g_image_display)
              
        # process the image to look for a circle within it.
        try:   
            img = cv2.cvtColor(g_image_display, cv2.COLOR_BGR2GRAY)
            img = cv2.medianBlur(img, 9)                                         # if not try 11, 9 worked best for me
            circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 10, np.array([]), 79, 23, 9, 26)
            if circles is not None:
                a, b, c = circles.shape
            else:
                print("no detection found")
                b = 0
        except:                                                                  # show exception hough and median can throw them adjust conatsnts if so
            import sys
            print("Error:", sys.exc_info()[0])
            print(sys.exc_info()[1])
            import traceback
            print(traceback.format_tb(sys.exc_info()[2]))
            b = 0

        if b >= 1:                                                                # at least one circle was seen if more pick the first one
            draw=cv2.circle(color,(int(circles[0][0][0]),int(circles[0][0][1])),radius=int(circles[0][0][2]),color=255, thickness=2)
            cv2.imwrite("/usr/spot/pics/pick_circle_found.jpg", draw)             # draw the grasp and write the image
            cv2.imshow("object found as circle", draw)                            # aslo show this
            x_cent = circles[0][0][0]                                             # x co-ordinate for center
            y_cent = circles[0][0][1]                                             # x co-ordinate for center      
            circ_radius = circles[0][0][2]                                        # not sure i can use it here ?
            pick_vec = geometry_pb2.Vec2(x=x_cent, y=y_cent)                      # pick vector sent to gripper
            robot.logger.info(f'Auto Picking object at image location ({circles[0][0][0}, {circles[0][0][1]})')
            robot.logger.info('Auto Picking object at image location (%s, %s)', circles[0][0][0], circles[0][0][1])
        else:
            # for manually clicking the co-ordinates use the mouse call back
            cv2.setMouseCallback(image_title, cv_mouse_callback)
            while g_image_click is None:
                key = cv2.waitKey(1) & 0xFF
                if key.upper() == ord('Q'):
                    # Quit
                    print('"q" pressed, exiting.')
                    exit(0) 
            robot.logger.info(f'Picking object at image location ({g_image_click[0]}, {g_image_click[1]})')
            robot.logger.info('Picking object at image location (%s, %s)', g_image_click[0], g_image_click[1])                    
            # pick vector as x,y co-ordinates       
            # from mouse
            pick_vec = geometry_pb2.Vec2(x=g_image_click[0], y=g_image_click[1])   # pick vector sent to gripper from mouse click on screen
            
        # Build the proto
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)

        # Optionally add a grasp constraint.  This lets you tell the robot you only want top-down grasps or side-on grasps.
        add_grasp_constraint(config, grasp, robot_state_client)

        # Ask the robot to pick up the object
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

        # Send the request
        cmd_response = manipulation_api_client.manipulation_api_command(manipulation_api_request=grasp_request)

        # Get feedback from the robot
        while True:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = manipulation_api_client.manipulation_api_feedback_command(manipulation_api_feedback_request=feedback_request)
            print(f'Current state: {manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state)}')
            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                break
            time.sleep(0.25)

        robot.logger.info('Finished grasp.')
        time.sleep(4.0)
        robot.logger.info('Sitting down and turning off.')

        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), 'Robot power off failed.'
        robot.logger.info('Robot safely powered off.')

def cv_mouse_callback(event, x, y, flags, param):
    global g_image_click, g_image_display
    clone = g_image_display.copy()
    if event == cv2.EVENT_LBUTTONUP:
        g_image_click = (x, y)
    else:
        # Draw some lines on the image.
        # print('mouse', x, y)
        color = (30, 30, 30)
        thickness = 2
        image_title = 'Click to grasp'
        height = clone.shape[0]
        width = clone.shape[1]
        cv2.line(clone, (0, y), (width, y), color, thickness)
        cv2.line(clone, (x, 0), (x, height), color, thickness)
        cv2.imshow(image_title, clone)


def add_grasp_constraint(config, grasp, robot_state_client):
    # There are 3 types of constraints:
    #   1. Vector alignment
    #   2. Full rotation
    #   3. Squeeze grasp
    #
    # You can specify more than one if you want and they will be OR'ed together.

    # For these options, we'll use a vector alignment constraint.
    use_vector_constraint = config.force_top_down_grasp or config.force_horizontal_grasp

    # Specify the frame we're using.
    grasp.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME

    if use_vector_constraint:
        if config.force_top_down_grasp:
            # Add a constraint that requests that the x-axis of the gripper is pointing in the
            # negative-z direction in the vision frame.

            # The axis on the gripper is the x-axis.
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

            # The axis in the vision frame is the negative z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)

        if config.force_horizontal_grasp:
            # Add a constraint that requests that the y-axis of the gripper is pointing in the
            # positive-z direction in the vision frame.  That means that the gripper is constrained to be rolled 90 degrees and pointed at the horizon.

            # The axis on the gripper is the y-axis.
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

            # The axis in the vision frame is the positive z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)

        # Add the vector constraint to our proto.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(axis_on_gripper_ewrt_gripper)
        constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(axis_to_align_with_ewrt_vo)

        # We'll take anything within about 10 degrees for top-down or horizontal grasps.
        constraint.vector_alignment_with_tolerance.threshold_radians = 0.17

    elif config.force_45_angle_grasp:
        # Demonstration of a RotationWithTolerance constraint.  This constraint allows you to
        # specify a full orientation you want the hand to be in, along with a threshold.
        #
        # You might want this feature when grasping an object with known geometry and you want to
        # make sure you grasp a specific part of it.
        #
        # Here, since we don't have anything in particular we want to grasp,  we'll specify an
        # orientation that will have the hand aligned with robot and rotated down 45 degrees as an
        # example.

        # First, get the robot's position in the world.
        robot_state = robot_state_client.get_robot_state()
        vision_T_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

        # Rotation from the body to our desired grasp.
        body_Q_grasp = math_helpers.Quat.from_pitch(math.radians(CAM_ROTATION_ANGLE[g_camera][2]))       # for camera 1 its 45 degrees
        vision_Q_grasp = vision_T_body.rotation * body_Q_grasp

        # Turn into a proto
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.rotation_with_tolerance.rotation_ewrt_frame.CopyFrom(vision_Q_grasp.to_proto())

        # We'll accept anything within +/- 10 degrees
        constraint.rotation_with_tolerance.threshold_radians = math.radians(10)

    elif config.force_squeeze_grasp:
        # Tell the robot to just squeeze on the ground at the given point.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.squeeze_grasp.SetInParent()

def main():
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    #parser.add_argument('-i', '--image-source', help='Get image from source', default='frontleft_fisheye_image')
    parser.add_argument('--auto-rotate', help='rotate right and front images to be upright', action='store_true')
    parser.add_argument('-t', '--force-top-down-grasp', help='Force the robot to use a top-down grasp (vector_alignment demo)', action='store_true')
    parser.add_argument('-f', '--force-horizontal-grasp', help='Force the robot to use a horizontal grasp (vector_alignment demo)', action='store_true')
    parser.add_argument('-r', '--force-45-angle-grasp', help='Force the robot to use a 45 degree angled down grasp (rotation_with_tolerance demo)', action='store_true')
    parser.add_argument('-s', '--force-squeeze-grasp', help='Force the robot to use a squeeze grasp', action='store_true')
    options = parser.parse_args()

    # prevent multiple options for grasp being passed
    num = 0
    if options.force_top_down_grasp:
        num += 1
    if options.force_horizontal_grasp:
        num += 1
    if options.force_45_angle_grasp:
        num += 1
    if options.force_squeeze_grasp:
        num += 1
    if num > 1:
        print('Error: cannot force more than one type of grasp.  Choose only one.')
        sys.exit(1)

    try:
        arm_object_grasp(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception('Threw an exception')
        return False

if __name__ == '__main__':
    if not main():
        sys.exit(1)