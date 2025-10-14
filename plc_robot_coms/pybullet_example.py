from enum import Enum
import pybullet as p    
import pybullet_data    
import time             
import numpy as np      
import sys

DIMENTION_NONE  = -1    
DIMENTION_2D    =  2    
DIMENTION_3D    =  3   
GRAVITY_VAL   = 9.81

class INTERPOLATION(Enum):
    """
    Interpolation Method
    """
    JOINT     = "joint"     # joint
    POSITION  = "pos"       # position

class MainPyBulletRobot:
"""
PyBullet Main Class

Properties
_robot_id(): Robot arm ID number
_n_joints(int): Number of joints in the robot arm
_interpolation(str): Search space (Cartesian space/joint space)

Methods
public

Main Processing
run(): Execution (obtains information from the sliders and moves the robot on the simulator)

protected

Preparation
_init_robot(): Initializes the robot
_init_environment(): Initializes the environment
_init_sliders(): Initializes the sliders
_get_joint_limit(): Obtains the joint limit value (min + max)
_get_init_thetas(): Obtains the initial angle

Main Processing
_get_slider_values(): Obtains the slider values
_set_joint(): Sets the joint angle
_set_text(): Sets text in the GUI
"""


    _IDX_MIN_JOINT  = 8                  # Element number where the minimum joint value is saved
    _IDX_MAX_JOINT  = 9                  # Element number where the maximum joint value is saved
    _JOINT_INIT     = 0.0                # Joint initial values
    
    _SLIDER_MAKE_WAIT_TIME = 0.2          # Wait time for creating slider [sec]
    _SIMULATION_SLEEP_TIME = 1. / 240.    # Simulation waiting time [sec]
    
    _DEBUG_TEXT_LIFE_TIME  = 0            # Text lifetime [sec] (0 is infinite time)
    _DEBUG_TEXT_SIZE       = 0.5          # text size
    
    
    def __init__(self, robot_urdf, plane_urdf, interpolation):
        """
            robot_urdf(str): 
			plane_urdf(str):
            interpolation(str): 
        """
        # PyBullet connects to the GUI (simulator)
        p.connect(p.GUI)
        # Adding a Path
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Initializing the Simulation
        p.resetSimulation()
        # Gravity setting (downward (-z axis) acceleration)
        p.setGravity(0, 0, -GRAVITY_VAL)

        # initialize the robot using the urdf file defining the length and mass of each element in the arm
        self._init_robot(robot_urdf)

        # load the plane urdf file and initialize the environment
        self._PLANE_URDF = plane_urdf
        self._init_environment()

        # init sliders on GUI
        self._init_sliders(interpolation)

        time.sleep(self._SLIDER_MAKE_WAIT_TIME)

    def _init_robot(self, robot_urdf):
        """
        initialize robot from loading urdf file
            robot_urdf(str):  (urdf)
        """
        self._robot_id = p.loadURDF(robot_urdf, basePosition=[0, 0, 0], useFixedBase=True)
        self._n_joints = p.getNumJoints(self._robot_id) - 1

        if not (self._n_joints == DIMENTION_2D or self._n_joints == DIMENTION_3D):
            raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

    def _init_environment(self):
        """
        load the environment plane
        """
        p.loadURDF(self._PLANE_URDF)

    def _init_sliders(self, interpolation):
        """
        Slider Initialization

        Parameters
        interpolation(str): Interpolation method (joint space/position space)
        """
        sliders = []

        # Adding a slider to the GUI
        if interpolation == INTERPOLATION.JOINT.value:  # Explore joint space
            # Get the minimum and maximum values ​​that can be set for a joint
            min_joints, max_joints = self._get_joint_limit()
            # Get the initial angle
            init_thetas = self._get_init_thetas()

            # Set the minimum, maximum and initial values ​​of the joints that can be set
            for joint_idx, (min_joint, max_joint, init_theta) in enumerate(zip(min_joints, max_joints, init_thetas)):
                # Get joint information and add a sliderGet joint information and add a slider
                slider = p.addUserDebugParameter(f"joiint {joint_idx + 1}", min_joint, max_joint, init_theta)
                sliders.append(slider)

        else:
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        self._interpolation = interpolation
        self._sliders = sliders

    def _get_joint_limit(self):
        """
        Gets the joint limit values ​​(min + max).

        Returns
        min_joints (list): [rad]
        max_joints (list): [rad]
        """
        min_joints = []
        max_joints = []

        # Get the minimum and maximum values ​​of all joints
        for i in range(self._n_joints):
            # Get information about joints
            joint_info = p.getJointInfo(self._robot_id, i)
            min_joint  = joint_info[self._IDX_MIN_JOINT]
            max_joint  = joint_info[self._IDX_MAX_JOINT]
            # Save the minimum and maximum values ​​in a list
            min_joints.append(min_joint)
            max_joints.append(max_joint)

        return min_joints, max_joints

    def _get_init_thetas(self):
        """
        get initial positions
            init_thetas(numpy.ndarray):  [rad]
        """
        init_thetas = np.ones(self._n_joints) * self._JOINT_INIT

        return init_thetas

    def run(self):
        """
        runs the simulation
        """
        # add text
        text_id = p.addUserDebugText("", textPosition=[0, 0, 0])

        p.setRealTimeSimulation(1)
        loop = True

        try:
            while loop == True:
                # get slider values from GUI
                slider_values = self._get_slider_values()

                # set joints to slider values
                self._set_joint(slider_values)

                # write text to GUI
                self._set_text(text_id)

                # wait the sleep time
                time.sleep(self._SIMULATION_SLEEP_TIME)
        except KeyboardInterrupt:
                loop = False
        p.disconnect()

    def _get_slider_values(self):
        """
        get the slider values
            slider_values(numpy.ndarray): 
        """
        slider_values = []

        for slider in self._sliders:
            slider_value = p.readUserDebugParameter(slider)
            slider_values.append(slider_value)

        return np.array(slider_values)

    def _set_joint(self, thetas):
        """
        set the joints to the values in thetas
            thetas(numpy.ndarray): 
        """
        for i in range(len(thetas)):
            p.setJointMotorControl2(
                bodyUniqueId=self._robot_id,    # ID of robot
                jointIndex=i,                   # joint number
                controlMode=p.POSITION_CONTROL, # mode is position
                targetPosition=thetas[i]        # set target
            )

    def _set_text(self, text_id):
        """
        GUI text writer
            text_id(): 
        """
        ee_pos = p.getLinkState(self._robot_id, self._n_joints)[0]
        text = f"end effecter pos:\nx={ee_pos[0]:.2f}, y={ee_pos[1]:.2f}, z={ee_pos[2]:.2f}"
        p.addUserDebugText(text, ee_pos, textColorRGB=[0, 0, 0], textSize=self._DEBUG_TEXT_SIZE, lifeTime=self._DEBUG_TEXT_LIFE_TIME, replaceItemUniqueId=text_id)

def main(urdfile="robot_3dof.urdf", planeurdfile="plane.urdf"):
    """
    run the simulation
    """
    robot_urdf = urdfile
    plane_urdf = planeurdfile
    interpolation = INTERPOLATION.JOINT.value
    my_robot = MainPyBulletRobot(robot_urdf, plane_urdf, interpolation)
    my_robot.run()

if __name__ == "__main__":
    if len(sys.argv[0]) == 0: 
        main()
    elif len(sys.argv[0]) == 1: 
        main(str(sys.argv[1]))
    elif len(sys.argv[0]) == 2: 
        main(str(sys.argv[1]), str(sys.argv[2]))	
    else:

        print("arg1=youturdf file arg2=your plane urdf file")    
