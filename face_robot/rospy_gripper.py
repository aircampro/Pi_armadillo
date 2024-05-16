#!/usr/bin/env python
'''
Original Author: Avishai Sintov
        https://github.com/avishais
'''

'''
Provides service to run random episode over ROS and collect that data.
ACP modified slightly to run different episodea dependant on length of the episode
'''
import rospy
import numpy as np
import time
import random
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import Empty, EmptyResponse
from rollout_node.srv import observation, IsDropped, TargetAngles
from transition_experience import *

class move_and_collect_data():

    gripper_closed = False
    #discrete_actions = True                                         # Discrete or continuous actions
    drop = True

    #num_episodes = 20000
    #episode_length = 10000

    # initialise the rospy class for controls  Publisher, services, service proxys, and subscribers 
    def __init__(self, episode_length = 1000, discrete_actions = True):
        self.episode_length = episode_length
        self.discrete_actions = discrete_actions
        rospy.init_node('collect_data', anonymous=True)

        self.pub_gripper_action = rospy.Publisher('/collect/gripper_action', Float32MultiArray, queue_size=10)
        rospy.Service('/collect/random_episode', Empty, self.run_random_episode)
        rospy.Service('/collect/save_data', Empty, self.save_data)
        
        self.obs_srv = rospy.ServiceProxy('/hand_control/observation', observation)
        self.drop_srv = rospy.ServiceProxy('/hand_control/IsObjDropped', IsDropped)
        self.move_srv = rospy.ServiceProxy('/hand_control/MoveGripper', TargetAngles)
        self.reset_srv = rospy.ServiceProxy('/hand_control/ResetGripper', Empty)
        rospy.Subscriber('/hand_control/cylinder_drop', Bool, self.callbackDrop)
        rospy.Subscriber('/hand_control/gripper_status', String, self.callbackGripperStatus)
        
        self.record_srv = rospy.ServiceProxy('/recorder/trigger', Empty)
        self.recorder_save_srv = rospy.ServiceProxy('/recorder/save', Empty)

        if rospy.has_param('~actions_mode'):
            self.discrete_actions = True if rospy.get_param('~actions_mode') == 'discrete' else False
        
        rospy.sleep(1.)

        print('[collect_data] Ready to collect...')

        self.rate = rospy.Rate(2) 
        rospy.spin()

    # call back sunscriber to detect if the hand closes
    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    # call back subscriber if the item is dropped
    def callbackDrop(self, msg):
        self.drop = msg.data

    # save the data
    def save_data(self, msg):
        print('[collect_data] Saving all data...')

        self.recorder_save_srv()
        
        return EmptyResponse()

    # run the record and action for the robot
    def run_random_episode(self, req):

        # Reset gripper
        self.reset_srv()
        while not self.gripper_closed:
            self.rate.sleep()

        print('[collect_data] Running random episode...')

        Done = False
        msg = Float32MultiArray()

        # Start episode
        n = 0
        action = np.array([0.,0.])                      # default the action array
        self.record_srv()
        for ep_step in range(self.episode_length):      # do this for steps == episode_length 

            if n == 0:                                  # choose a new random action
                if self.discrete_actions:
                    action, n = self.choose_action()
                else:
                    action = self.choose_action()
                    
            msg.data = action
            # publish the action to save it
            self.pub_gripper_action.publish(msg)
            # move the gripper by the action angle array which was randomnly chosen above
            suc = self.move_srv(action).success
            n -= 1                                     # decrement to zero until it chooses a new one

            if suc:
                fail = self.drop                       # Check if dropped - end of episode
            else:
                # End episode if overload or angle limits reached
                rospy.logerr('[collect_data] Failed to move gripper. Episode declared failed.')
                fail = True

            if not suc or fail:
                break

            self.rate.sleep()

        print('[collect_data] End of episode.')

        return EmptyResponse()

    # choose randomly the action for the robot to do
    def choose_action(self):
        if self.discrete_actions:
            # A is an array of possible action angles
            A = np.array([[1.,1.],[-1.,-1.],[-1.,1.],[1.,-1.],[1.,0.],[-1.,0.],[0.,-1.],[0.,1.]])
            # choose randomly the action from the above list
            a = A[np.random.randint(A.shape[0])]

            # now choose the number of steps for this action before choosing a new one
            # use a random to choose which one...
            if np.random.uniform() > 0.8:
                if np.random.uniform() > 0.5:
                    num_steps = np.random.randint(round(self.episode_length/2))
                else:
                    num_steps = np.random.randint(round(self.episode_length/4))
            else:
                num_steps = np.random.randint(round(self.episode_length/10),self.episode_length)

            return a, num_steps
        else:
            # set random motion array
            a = np.random.uniform(-1.,1.,2)
            if np.random.uniform(0,1,1) > 0.35:
                if np.random.uniform(0,1,1) > 0.5:
                    a[0] = np.random.uniform(-1.,-0.8,1)           # modify the data to this random
                    a[1] = np.random.uniform(-1.,-0.8,1)
                else:
                    a[0] = np.random.uniform(0.8,1.,1)             # modify the data to this random
                    a[1] = np.random.uniform(0.8,1.,1)

            return a

if __name__ == '__main__':
    # tests class when loaded
    try:
        move_and_collect_data()
    except rospy.ROSInterruptException:
        pass    