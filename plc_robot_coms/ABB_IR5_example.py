#!/usr/bin/env python
#
# ABB IR5 robot series driver examples
#
# ref:- https://github.com/rpiRobotics/abb_robot_client
#
from abb_robot_client import egm, rws
from contextlib import closing
import time
import numpy as np
import abb_motion_program_exec as abb
import copy

# change this to your robot ip and port
#
_robot_url = "http://localhost:80"
# _robot_url = "https://localhost:443"

def do_pose(pry1=[0,0,0],q1=[1,0,0,0],pry2=[0,0,0],q2=[1,0,0,0],pry3=[0,0,0],q3=[1,0,0,0],t1=[400,0,600],t2=[0.7071068, 0., 0.7071068, 0.],rp=[10, 0.05, 0.05]):
    mm = abb.egm_minmax(-1e-3,1e-3)
    corr_frame = abb.pose(pry1,q1)
    corr_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
    sense_frame = abb.pose(pry2,q2)
    sense_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
    egm_offset = abb.pose(pry3,q3)
    egm_config = abb.EGMPoseTargetConfig(corr_frame, corr_fr_type, sense_frame, sense_fr_type, mm, mm, mm, mm, mm ,mm, 1000, 1000)
    r1 = abb.robtarget(t1,t2,abb.confdata(0,0,0,1),[0]*6)                                         # create the pose
    mp = abb.MotionProgram(egm_config = egm_config)                                               # write the rapid code to move to the desired pose
    mp.MoveJ(r1,abb.v5000,abb.fine)                                                               # issue moveJ instruction
    mp.EGMRunPose(rp[0], rp[1], rp[2], egm_offset)
    client = abb.MotionProgramExecClient(base_url=_robot_url)                                     # connect to the client robot
    lognum = client.execute_motion_program(mp, wait=False)                                        # execute the program on the remote robot
    t1 = time.perf_counter()
    r2 = copy.copy(r1)
    egm = egm.EGM()
    t2 = t1
    while (t2 - t1) < 5 :
        t2 = time.perf_counter()
        res, feedback = egm.receive_from_robot(timeout=0.05)
        if res:
            r2.trans[1]=(t2-t1)*100.
            egm.send_to_robot_cart(r2.trans, r2.rot)
    client.stop_egm()
    while client.is_motion_program_running():
        time.sleep(0.05)

def code_example():
    # Fill motion program for T_ROB1
    t1 = abb.robtarget([575,-200,1280],[0,-.707,0,.707],abb.confdata(0,0,-1,1),[0]*6)
    t2 = abb.robtarget([575,200,1480],[0,-.707,0,.707],abb.confdata(-1,-1,0,1),[0]*6)
    t3 = abb.robtarget([575,0,1280],[0,-.707,0,.707],abb.confdata(-1,-1,0,1),[0]*6)

    # tool 1
    my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)) 

    # write your motion program here for tool 1
    mp = abb.MotionProgram(tool=my_tool)
    mp.SyncMoveOn()
    mp.MoveAbsJ(abb.jointtarget([5,-20,30,27,-11,-27],[0]*6),abb.v1000,abb.fine)
    mp.MoveL(t1,abb.v1000,abb.fine)
    mp.MoveJ(t2,abb.v5000,abb.fine)
    mp.MoveL(t3,abb.v500,abb.fine)
    mp.WaitTime(1)
    mp.MoveL(t1,abb.v5000,abb.z50)
    mp.MoveJ(t2,abb.v500,abb.z200)
    mp.MoveL(t3,abb.v5000,abb.fine)

    # Fill motion program for T_ROB2. Both programs must have
    # same number of commands
    t1_2 = abb.robtarget([250,-200,1280],[.707,0,.707,0],abb.confdata(-1,-1,0,1),[0]*6)
    t2_2 = abb.robtarget([250,200,1480],[.707,0,.707,0],abb.confdata(0,0,-1,1),[0]*6)
    t3_2 = abb.robtarget([250,0,1280],[.707,0,.707,0],abb.confdata(0,0,0,1),[0]*6)

    # tool 2
    my_tool2 = abb.tooldata(True,abb.pose([0,0,0.5],[1,0,0,0]),abb.loaddata(0.1,[0,0,0.1],[1,0,0,0],0,0,0)) 

    # write your motion program here for tool 1
    mp2 = abb.MotionProgram(tool=my_tool2)
    mp2.SyncMoveOn()
    mp2.MoveAbsJ(abb.jointtarget([1,1,40,2,-40,-2],[0]*6),abb.v1000,abb.fine)
    mp2.MoveJ(t1_2,abb.v1000,abb.fine)
    mp2.MoveL(t2_2,abb.v5000,abb.fine)
    mp2.MoveL(t3_2,abb.v500,abb.fine)
    mp2.WaitTime(1)
    mp2.MoveL(t1_2,abb.v5000,abb.z50)
    mp2.MoveL(t2_2,abb.v500,abb.z200)
    mp2.MoveL(t3_2,abb.v5000,abb.fine)

    # Execute the motion program on the robot
    client = abb.MotionProgramExecClient(base_url=_robot_url)
    # Execute both motion programs simultaneously
    log_results = client.execute_multimove_motion_program([mp,mp2])

def test_controller_state():
    c = rws.RWS(_robot_url)
    with closing(c):        
        c.get_execution_state()
        c.get_controller_state()
        c.get_operation_mode()

def test_rws_tasks():
    c = rws.RWS(_robot_url)
    with closing(c):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",0)
        c.resetpp()
        c.start("forever", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        assert c.get_execution_state().cycle == "forever"
        time.sleep(1)
        c.stop()
        assert c.get_execution_state().ctrlexecstate == "stopped"

        task_state = c.get_tasks()

        c.deactivate_task("T_ROB1")
        c.activate_task("T_ROB1")

def test_signal():
    c = rws.RWS(_robot_url)
    with closing(c):
        c.set_digital_io("test_digital_io1", 1)
        assert c.get_digital_io("test_digital_io1") == 1
        c.set_digital_io("test_digital_io1", 0)
        assert c.get_digital_io("test_digital_io1") == 0

        c.set_digital_io("test_digital_io2", 1,  "DeviceNet", "d651")
        assert c.get_digital_io("test_digital_io2", "DeviceNet", "d651") == 1
        c.set_digital_io("test_digital_io2", 0, "DeviceNet", "d651")
        assert c.get_digital_io("test_digital_io2", "DeviceNet", "d651") == 0

        c.set_analog_io("test_analog_io1", 14.285)
        assert abs(c.get_analog_io("test_analog_io1") - 14.285) < 0.001
        c.set_analog_io("test_analog_io1", 0)
        assert abs(c.get_analog_io("test_analog_io1") - 0) < 0.001

        c.set_analog_io("test_analog_io2", 8.562, "DeviceNet", "d651")
        assert abs(c.get_analog_io("test_analog_io2", "DeviceNet", "d651") - 8.562) < 0.001
        c.set_analog_io("test_analog_io2", 0, "DeviceNet", "d651")
        assert abs(c.get_analog_io("test_analog_io2", "DeviceNet", "d651") - 0) < 0.001

def test_rapid_vars_str_flt(v="test_var_str", sv="\"Hello World!\"", l1="T_ROB1", n="test_var_num", nv=123.456, l2="T_ROB1/Module1" ):
    c = rws.RWS(_robot_url)
    with closing(c):
        c.set_rapid_variable(v, sv, l1)
        assert c.get_rapid_variable(v, l1) == sv
        c.set_rapid_variable(n, str(nv), l2)
        assert abs(float(c.get_rapid_variable(n, l2)) - nv) < 0.001

def test_rapid_vars_str(v="test_var_str", sv="\"Hello World!\"", l1="T_ROB1"):
    c = rws.RWS(_robot_url)
    with closing(c):
        c.set_rapid_variable(v, sv, l1)
        assert c.get_rapid_variable(v, l1) == sv

def test_rapid_vars_flt(n="test_var_num", nv=123.456, l2="T_ROB1/Module1" ):
    c = rws.RWS(_robot_url)
    with closing(c):
        c.set_rapid_variable(n, str(nv), l2)
        assert abs(float(c.get_rapid_variable(n, l2)) - nv) < 0.001

def test_files(f="/test_file.txt", t="your message is here..!"):
    c = rws.RWS(_robot_url)
    with closing(c):
        ramdisk_path = c.get_ramdisk_path()
        data = t.encode("utf-8")
        c.upload_file(ramdisk_path + f, data)
        file_list = c.list_files(ramdisk_path)
        assert "test_file.txt" in file_list
        assert c.read_file(ramdisk_path + f) == data
        c.delete_file(ramdisk_path + f)
        file_list = c.list_files(ramdisk_path)
        assert "test_file.txt" not in file_list

def test_evtlog():
    c = rws.RWS(_robot_url)
    with closing(c):
        evts = c.read_event_log()
        print(evts)
        return evts

def test_current_targets():
    c = rws.RWS(_robot_url)
    with closing(c):
        c.get_jointtarget("ROB_1")
        c.get_robtarget("ROB_1")
        c.get_robtarget("ROB_1", "tool0", "wobj0", "Base")

def set_speed_ratio(s=100):
    c = rws.RWS(_robot_url)
    with closing(c):
        c.get_speedratio()
        c.set_speedratio(s)
        return (c.get_speedratio() == s)

def test_egm_position_command(joints=[21, -10, -20, 30, 40, 50], external_joints=[90,100], joints_speed=[10,10,10,10,10,10], external_joints_speed=[10,10], rapid_to_robot=[88,99]):
    c = rws.RWS(_robot_url)
    e = egm.EGM()
    with closing(c), closing(e):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",1)
        c.resetpp()
        c.start("once", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        time.sleep(1)

        recv_count = 0
        loop_count = 0
        while True:
            try:
                res, _ = e.receive_from_robot(0.1)
                if res:
                    recv_count += 1
                    if recv_count > 5:
                        break
            except:
                pass
            loop_count += 1
            assert loop_count < 10

        e.send_to_robot(joints, joints_speed, external_joints, external_joints_speed, rapid_to_robot)
        time.sleep(3)
        e.send_to_robot(joints)
        time.sleep(2)
        while True:
            res, _ = e.receive_from_robot()
            if not res:
                break
        res, egm_state = e.receive_from_robot(1)
        assert res
        np.testing.assert_allclose(egm_state.joint_angles, joints, atol=0.1)
        c.set_digital_io("stop_egm", 1)
        time.sleep(3)
        assert c.get_execution_state().ctrlexecstate == "stopped"

def test_egm_pose_command( pose_lin=[535,181,625], pose_quat=[0.704416,0.1227878,0.6963642,0.0616284], pose_speed=[0.1,0.11,0.12,0.13,0.14,0.15], external_joints=[90,100], external_joints_speed=[10,10], rapid_to_robot=[88,99]):
    c = rws.RWS(_robot_url)
    e = egm.EGM()
    with closing(c), closing(e):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",2)
        c.resetpp()
        c.start("once", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        time.sleep(1)

        recv_count = 0
        loop_count = 0
        while True:
            try:
                res, _ = e.receive_from_robot(0.1)
                if res:
                    recv_count += 1
                    if recv_count > 5:
                        break
            except:
                pass
            loop_count += 1
            assert loop_count < 10

        e.send_to_robot_cart(pose_lin, pose_quat, pose_speed, external_joints, external_joints_speed, rapid_to_robot)
        time.sleep(0.1)
        while True:
            res, _ = e.receive_from_robot()
            if not res:
                break
        e.send_to_robot_cart(pose_lin, pose_quat)
        time.sleep(2)
        while True:
            res, _ = e.receive_from_robot()
            if not res:
                break
        res, egm_state = e.receive_from_robot(0.5)
        assert res
        np.testing.assert_allclose(egm_state.cartesian[0], pose_lin, atol=0.1)
        np.testing.assert_allclose(egm_state.cartesian[1], pose_quat, atol=0.1)
        c.set_digital_io("stop_egm", 1)
        time.sleep(3)
        assert c.get_execution_state().ctrlexecstate == "stopped"

def test_egm_path_corr(p1=[10,20,0.0],p2=[-10,-20,0.0]):
    c = rws.RWS(_robot_url)
    e = egm.EGM()
    with closing(c), closing(e):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",3)
        c.resetpp()
        c.start("once", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        time.sleep(1)

        recv_count = 0
        loop_count = 0
        while True:
            try:
                res, _ = e.receive_from_robot(0.1)
                if res:
                    recv_count += 1
                    if recv_count > 5:
                        break
            except:
                pass
            loop_count += 1
            assert loop_count < 10

        for i in range(10):
            e.send_to_robot_path_corr(p1, 1)
            time.sleep(0.5)
            e.send_to_robot_path_corr(p2,1)
            time.sleep(0.5)
        
        time.sleep(0.1)
        c.stop()

# the robot sequence of actions is here.
def main():
    test_controller_state()
    test_files()
    e=test_evtlog()
    test_current_targets()
    set_speed_ratio()
    test_rapid_vars_str_flt()
    test_rapid_vars_str(sv="new string")
    test_rapid_vars_flt(nv=-60.76)
    test_signal()
    test_egm_position_command()
    test_egm_pose_command()
    test_egm_path_corr()
    do_pose()
    code_example()

if __name__ == '__main__':
    main()