#!/usr/bin/python
#
# Universal Robots RTDE interface ref:- https://github.com/UniversalRobots/RTDE_Python_Client_Library/blob/main/examples/example_control_loop.py
#
import sys
sys.path.append("..")
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "10.0.1.6"                                                  # This is the robot connected over wifi to us for local control its "localhost"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"                       # configuration of read/write params
logging.getLogger().setLevel(logging.INFO)

# get the state and steps from the config file 
conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

# connect to the robot
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# Setpoints to move the robot to (these shall be read from the config file)
setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]
setp3 = [-0.12, -0.73, 0.44, 0, 3.11, 0.04]
setp4 = [-0.22, -0.61, 0.11, 0, 3.50, 0.07]
seq_steps = [ setp1, setp2, setp3, setp4 ]

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list

def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp

# start data synchronization
if not con.send_start():
    sys.exit()

# define the actions for the robot
def main():
    move_completed = True
    N_step_count = 0
    while N_step_count < len(seq_steps):
        # receive the current state
        state = con.receive()
        if state is None:
            break
        # do sequence of requested steps
        if move_completed and state.output_int_register_0 == 1:
            move_completed = False
            new_setp = seq_steps[N_step_count]                                   # read new angle set-points
            list_to_setp(setp, new_setp)
            print("New pose = " + str(new_setp))
            # send new setpoint
            con.send(setp)
            watchdog.input_int_register_0 = 1
        elif not move_completed and state.output_int_register_0 == 0:
            print("Move to confirmed pose = " + str(state.target_q))
            move_completed = True
            N_step_count += 1		
            watchdog.input_int_register_0 = 0
        # kick watchdog
        con.send(watchdog)
    con.send_pause()
    con.disconnect()

if __name__ == '__main__':
    main()