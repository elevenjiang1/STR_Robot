import os
import time
import math
import queue
import threading
import numpy as np
from robot_lib.old_str400_485 import STR400Protocol as OLDSTR400Protocol
from asm_robot import ASMRobot


def float2int(input_list):
    return list(map(int,input_list))

def check_old_angle():
    oldRobot=OLDSTR400Protocol(port='/dev/ttyUSB1')
    while True:
        robot_joitns=oldRobot.read_joint_angle()
        print("robot_joints is:",list(map(int,robot_joitns)) )
        time.sleep(0.5)

def check_new_angle():
    newRobot=ASMRobot('/dev/ttyUSB0')
    newRobot.set_torque(flag=False)
    
    while True:
        newrobot_joints=newRobot.get_robot_joints()
        print("newrobot_joints:",float2int(newrobot_joints))
        time.sleep(0.5)

def old2new(oldrobot_joints):
    new_target_joints=[]
    for joint in oldrobot_joints:
        if joint>180:
            joint=joint-360
        new_target_joints.append(joint)
        
    return new_target_joints


def move_step_by_step():
    #1: init each joints
    oldRobot=OLDSTR400Protocol(port='/dev/ttyUSB1')
    newRobot=ASMRobot(port_name='/dev/ttyUSB0')
    newRobot.home()
    
    #2: ensure two robot in same begining joints
    while True:
        oldrobot_joints=oldRobot.read_joint_angle()
        print("Get oldrobot joints is:",oldrobot_joints)
        
        new_joints=old2new(oldrobot_joints)
        
        print("target_joints is:",new_joints)
        newRobot.move_joints(new_joints,whole_time=2000,flag_block=True,flag_debug=True)
        
        temp=input("Wait for new joints...")

import copy        
def read_and_map_realtime():
    #1: load old and new robot
    oldRobot=OLDSTR400Protocol(port='/dev/ttyUSB1')
    newRobot=ASMRobot(port_name='/dev/ttyUSB0')
    
    print("Robot is ready")
    
    #2: ensure two robot in same begining joints
    newRobot.home()
    newRobot.init_servoj()
    
    #3: move to target pose
    oldrobot_joints=oldRobot.read_joint_angle()
    new_joints=old2new(oldrobot_joints)
    current_joints=newRobot.get_robot_joints()
    target_joints=copy.deepcopy(new_joints)
    
    #servo to the first pose
    print("Begin to move init pose")
    interpolate_joints=np.linspace(current_joints,target_joints,100)
    for joints in interpolate_joints:
        newRobot.servo_joints(joints)
            
    #4: in a big loop, if the joints diff is too big, just interpolate
    while True:
        begin_time=time.time()
        oldrobot_joints=oldRobot.read_joint_angle()
        new_target_joints=old2new(oldrobot_joints)
        newRobot.servo_joints(new_target_joints)
        print("Movement time is: {:.3f}".format(time.time()-begin_time))
        
        
if __name__=="__main__":
    # check_old_angle()
    # check_new_angle()
    # move_step_by_step()
    read_and_map_realtime()
    