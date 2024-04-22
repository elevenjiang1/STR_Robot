import os
import sys
import time
import numpy as np
from asm_robot import ASMRobot
from asm_config import ASMConfig

def test_asm_robot_api():
    asmRobot=ASMRobot()
    asmRobot.set_torque(flag=False)
    
    while True:
        robot_joints=asmRobot.get_robot_joints()
        print("Get robot_joints is:")
        print(list(map(int,robot_joints)))
        
        time.sleep(0.5)
    
def test_group_read():
    asmRobot=ASMRobot()
    asmRobot.set_torque(flag=False)
    
    
    groupSyncRead=asmRobot.init_group_read(ASMConfig.ADDR_PRESENT_POSITION,byte_length=ASMConfig.LEN_PRESENT_POSITION)
    while True:
        print("*"*30)
        read_flag,result_list=asmRobot.common_group_read(groupSyncRead,ASMConfig.ADDR_PRESENT_POSITION,byte_length=ASMConfig.LEN_PRESENT_POSITION,flag_unsigned=False)
        temp_joints=asmRobot._value_to_angle(result_list)
        
        
        robot_joints=asmRobot.get_robot_joints()
        
        
        if np.max(np.abs(temp_joints-robot_joints))>10:
            print("Error")
            print("1 is:",list(map(int,temp_joints)))
            print("2 is:",list(map(int,robot_joints)))
            
        time.sleep(0.001)
    
def test_asm_robot_move_joints():
    asmRobot=ASMRobot()
    print("Moveing home")
    asmRobot.home()
    
    while True:
        print("First pose")
        asmRobot.move_joints([90,0,0,0,0,0],whole_time=3000)
        robot_joints=asmRobot.get_robot_joints()
        print("Get robot_joints is:")
        print(list(map(int,robot_joints)))
        
        time.sleep(1)
        
        asmRobot.home()
        robot_joints=asmRobot.get_robot_joints()
        print("Get robot_joints is:")
        print(list(map(int,robot_joints)))
        
    
    
if __name__ == '__main__':
    # test_asm_robot_api()
    # test_group_read()
    test_asm_robot_move_joints()