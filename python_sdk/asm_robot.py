import os
import sys
import copy
import time
import struct
import numpy as np


import asm_sdk
from asm_config import ASMConfig


# 定义常量
TWO_POW_31 = 2**31
TWO_POW_32 = 2**32

class ASMRobot:
    def __init__(self,port_name='/dev/ttyUSB0') -> None:
        #1: init Port and PacketHandler
        #1.1 portHandler
        self.portHandler = asm_sdk.PortHandler(port_name)
        if not self.portHandler.openPort():
            self._beautiful_print("Can Not Open {} Port, Maybe need to chmod it".format(port_name))
        
        if not self.portHandler.setBaudRate(ASMConfig.BAUDRATE):
            self._beautiful_print("Can Not Set BAUDRATE")
        
        #1.2 init packetHandler and set status
        self.packetHandler = asm_sdk.PacketHandler(protocol_version=2.0)
        
        #2： Setup basic setting
        self.set_torque(flag=True)#enable torque
        self.set_trajectory_profile(flag=True)#enable trajectory profile; servoj should close trajectory profile
        set_success,movement_duration=self.set_movement_duration(3000)
        
        
        #3: Finish Init robot
        self._beautiful_print("Port {} Open Success".format(port_name))
        
        
        ###Common setting###
        self._home_joints=np.array([0,0,0,0,0,0])
        
    
    ###Common Read and Write###
    def unsigned_to_signed(self,value):
        if value >= TWO_POW_31:
            return value - TWO_POW_32
        return value
    
    def signed_to_unsigned(self,value):
        if value < 0:
            return value + TWO_POW_32
        return value
    
    def common_write(self,address,data,byte_length):
        if byte_length==1:
            for i in range(len(ASMConfig.DXL_ID)):
                dxl_comm_result, dxl_error =self.packetHandler.write1ByteTxRx(self.portHandler, ASMConfig.DXL_ID[i], address, data)
                if dxl_comm_result != asm_sdk.COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    return False
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    return False
                
            return True
                
        elif byte_length==2:
            for i in range(len(ASMConfig.DXL_ID)):
                dxl_comm_result, dxl_error =self.packetHandler.write2ByteTxRx(self.portHandler, ASMConfig.DXL_ID[i], address, data)
                if dxl_comm_result != asm_sdk.COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    return False
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    return False
                
            return True
        
        elif byte_length==4:
            pass

    def common_read(self,address,byte_length,flag_unsigned=True):
        if byte_length==1:
            result_list=[]
            
            for i in range(len(ASMConfig.DXL_ID)):
                dxl_data,dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ASMConfig.DXL_ID[i], address)
                if dxl_comm_result != asm_sdk.COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    return False,result_list
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    return False,result_list
                    
                result_list.append(dxl_data)
                
            return True,result_list
        
        elif byte_length==2:
            result_list=[]
            
            for i in range(len(ASMConfig.DXL_ID)):
                dxl_data,dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ASMConfig.DXL_ID[i], address)
                if dxl_comm_result != asm_sdk.COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    return False,result_list
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    return False,result_list
                    
                result_list.append(dxl_data)
                
            return True,result_list
                
        elif byte_length==4:
            result_list=[]
            
            for i in range(len(ASMConfig.DXL_ID)):
                dxl_data,dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, ASMConfig.DXL_ID[i], address)
                if not flag_unsigned:
                    dxl_data=self.unsigned_to_signed(dxl_data)
                # dxl_data = struct.unpack('i', struct.pack('I', dxl_data))[0]
                
                if dxl_comm_result != asm_sdk.COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    return False,result_list
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    return False,result_list
                    
                result_list.append(dxl_data)
                
            return True,result_list
            
    def common_group_write(self,address,data_list,byte_length):
        #add data
        groupSyncWrite = asm_sdk.GroupSyncWrite(self.portHandler, self.packetHandler, address, byte_length)
        for i in range(len(ASMConfig.DXL_ID)):
            param_goal_position=[asm_sdk.DXL_LOBYTE(asm_sdk.DXL_LOWORD(data_list[i])), asm_sdk.DXL_HIBYTE(asm_sdk.DXL_LOWORD(data_list[i])), asm_sdk.DXL_LOBYTE(asm_sdk.DXL_HIWORD(data_list[i])), asm_sdk.DXL_HIBYTE(asm_sdk.DXL_HIWORD(data_list[i]))]
            dxl_addparam_result=groupSyncWrite.addParam(ASMConfig.DXL_ID[i],param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % ASMConfig.DXL_ID[i])
                return False
            
        #send data
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != asm_sdk.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        
        #clear data
        groupSyncWrite.clearParam()
                
        return True
    
    def init_group_read(self,address,byte_length):
        groupSyncRead = asm_sdk.GroupSyncRead(self.portHandler, self.packetHandler, address,byte_length)
        for i in range(len(ASMConfig.DXL_ID)):
            dxl_addparam_result = groupSyncRead.addParam(ASMConfig.DXL_ID[i])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % ASMConfig.DXL_ID[i])
                quit()
        return groupSyncRead
        
    def common_group_read(self,groupSyncRead,address,byte_length,flag_unsigned=True):
        all_result_list=[]
        
        #1: syncread data
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != asm_sdk.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False,all_result_list
        
        #2: check data available
        for i in range(len(ASMConfig.DXL_ID)):
            dxl_getdata_result = groupSyncRead.isAvailable(ASMConfig.DXL_ID[i], address, byte_length)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % ASMConfig.DXL_ID[i])
                return False,all_result_list
        
        #3: get the value
        for i in range(len(ASMConfig.DXL_ID)):
            # Get Dynamixel present position value
            dxl_data = groupSyncRead.getData(ASMConfig.DXL_ID[i],address,byte_length)
            if not flag_unsigned:#chang to signed
                dxl_data = self.unsigned_to_signed(dxl_data)
            all_result_list.append(int(dxl_data))
            
        return True,all_result_list

    ###Setting Funtion###
    def set_torque(self,flag=True):
        if flag:
            return self.common_write(ASMConfig.ADDR_TORQUE_ENABLE,ASMConfig.TORQUE_ENABLE,byte_length=1)
        else:
            return self.common_write(ASMConfig.ADDR_TORQUE_ENABLE,ASMConfig.TORQUE_DISABLE,byte_length=1)
    
    def set_trajectory_profile(self,flag=True):
        """
        Servoj can only be in disable trajectory profile mode

        Args:
            flag (bool, optional): _description_. Defaults to True.

        Returns:
            _type_: _description_
        """
        
        
        time.sleep(0.5)#Wait for a while
        if flag:
            #1: set profile
            self.common_write(ASMConfig.ADDR_DRIVE_MODE,ASMConfig.PROFILE_ENABLE,byte_length=1)
            
            #2: check profile
            flag_writeable,control_model_list=self.common_read(ASMConfig.ADDR_DRIVE_MODE,byte_length=1)
            if not flag_writeable:
                print("!!!Set profile enable failed, it return {}!!!".format(control_model_list))
                return False,control_model_list
            else:
                if all([x==ASMConfig.PROFILE_ENABLE for x in control_model_list]):
                    return True,control_model_list
                else:
                    print("!!!Set profile enable failed, it return {}!!!".format(control_model_list))
                    return False,control_model_list
                
                
        else:
            #1: disable profile
            self.common_write(ASMConfig.ADDR_DRIVE_MODE,ASMConfig.PROFILE_DISABLE,byte_length=1)
            
            #2: check profile
            flag_writeable,control_model_list=self.common_read(ASMConfig.ADDR_DRIVE_MODE,byte_length=1)
            if not flag_writeable:
                print("!!!Set profile enable failed, it return {}!!!".format(control_model_list))
                return False,control_model_list
            else:
                if all([x==ASMConfig.PROFILE_DISABLE for x in control_model_list]):
                    return True,control_model_list
                else:
                    print("!!!Set profile enable failed, it return {}!!!".format(control_model_list))
                    return False,control_model_list
            
    def set_movement_duration(self,movement_duration=None):
        """_summary_

        Args:
            movement_duration (_type_, optional): If None, just read movement_duration, If True, set the movement_duration(ms)
        """
        movement_duration_list=[]
        if movement_duration is not None:
            #Set data
            if not self.common_write(ASMConfig.ADDR_MOVEMENT_DURATION,movement_duration,byte_length=2):
                return False,movement_duration_list
            #Read and check
            return self.common_read(ASMConfig.ADDR_MOVEMENT_DURATION,byte_length=2)
        else:
            return self.common_read(ASMConfig.ADDR_MOVEMENT_DURATION,byte_length=2)
                  
    ###Common Function###
    def _angle_to_value(self,angle_list):
        # 将角度转换为值范围
        # -180° 对应 -16384, 180° 对应 16384
        value_list=[]
        for angle in angle_list:
            value = int((angle / 180.0) * 16384)
            value_list.append(value)
        return value_list
    
    def _value_to_angle(self,value_list):
        # 将值范围转换为角度
        # -16384 对应 -180°, 16384 对应 180°
        angle_list=[]
        for value in value_list:
            angle = value/91.0
            angle_list.append(angle)
        return angle_list
    
    def get_robot_joints(self):
        read_flag,result_list=self.common_read(ASMConfig.ADDR_PRESENT_POSITION,byte_length=4,flag_unsigned=False)
        
        if read_flag:
            robot_joints=self._value_to_angle(result_list)
            return np.array(robot_joints)
        else:
            return None
    
    def move_joints(self,target_joints,whole_time=2000,flag_block=True,flag_debug=False):
        #1: init movement setting
        result,movement_duration_list=self.set_movement_duration(whole_time)
        if not result:
            print("!!!Set movement duration failed!!!")
            print("!!!Please check the movement_duration_list is:",movement_duration_list)
            return False
        
        #2: send data
        value_list=self._angle_to_value(target_joints)
        if not self.common_group_write(ASMConfig.ADDR_GOAL_POSITION,value_list,byte_length=ASMConfig.LEN_GOAL_POSITION):
            return False
        
        #3: wait until arrive, must be True now
        ###Block by group read mode###
        # groupSyncRead=self.init_group_read(ASMConfig.ADDR_PRESENT_POSITION,byte_length=ASMConfig.LEN_PRESENT_POSITION)
        # if flag_block:
        #     while True:
        #         read_flag,result_list=self.common_group_read(groupSyncRead,ASMConfig.ADDR_PRESENT_POSITION,byte_length=ASMConfig.LEN_PRESENT_POSITION,flag_unsigned=False)
        #         print("Get result list is:",result_list)
        #         if read_flag:
        #             if np.max(np.abs(np.array(result_list)-np.array(value_list)))<ASMConfig.DXL_MOVING_STATUS_THRESHOLD:
        #                 groupSyncRead.clearParam()
        #                 break
        #         else:
        #             print("!!!Grounp read in move_joints failed!!!")
                    
        #         time.sleep(0.5)
                
                
        ###Block by each motor read mode###
        if flag_block:
            while True:
                current_robot_joints=self.get_robot_joints()
                if current_robot_joints is not None:
                    if np.max(np.abs(current_robot_joints-target_joints))<0.5:
                        break
                    else:
                        if flag_debug:
                            print("Current robot joints is:",list(map(int,current_robot_joints)))
                            print("Diff is:",np.max(np.abs(current_robot_joints-target_joints)))
                        time.sleep(0.5)
                        
                else:
                    print("Still error in robot")
                    time.sleep(0.5)
        
    def init_servoj(self,flag=True):
        if flag:
            result,control_mode_list=self.set_trajectory_profile(False)#Disable trajectory profile
            if not result:
                return False
            self.servoj_groupSyncWrite = asm_sdk.GroupSyncWrite(self.portHandler, self.packetHandler, ASMConfig.ADDR_GOAL_POSITION, ASMConfig.LEN_GOAL_POSITION)
            return True
            
        else:
            result,control_mode_list=self.set_trajectory_profile(True)#Disable trajectory profile
            if not result:
                return False
            self.servoj_groupSyncWrite = None
            self.servoj_groupSyncRead = None
            return True
            
    def servo_joints(self,target_joints,servo_time=0.01):
        #Add target_joints into SyncWrite
        begin_time=time.time()
        if isinstance(target_joints,np.ndarray):
            target_joints=target_joints.astype(np.int32).tolist()
        
        target_value=self._angle_to_value(target_joints)
        for i in range(len(ASMConfig.DXL_ID)):
            param_goal_position=[asm_sdk.DXL_LOBYTE(asm_sdk.DXL_LOWORD(target_value[i])), asm_sdk.DXL_HIBYTE(asm_sdk.DXL_LOWORD(target_value[i])), asm_sdk.DXL_LOBYTE(asm_sdk.DXL_HIWORD(target_value[i])), asm_sdk.DXL_HIBYTE(asm_sdk.DXL_HIWORD(target_value[i]))]
            dxl_addparam_result=self.servoj_groupSyncWrite.addParam(ASMConfig.DXL_ID[i],param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % ASMConfig.DXL_ID[i])
                quit()
                
        #Send data
        dxl_comm_result = self.servoj_groupSyncWrite.txPacket()
        if dxl_comm_result != asm_sdk.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            
        #Clear send data and sleep
        self.servoj_groupSyncWrite.clearParam()
        time.sleep(max(0,servo_time-(time.time()-begin_time)))
    
    ###Other Function###
    def home(self,t=2000):
        self.move_joints(self._home_joints,whole_time=t)
    
    def _beautiful_print(self,data):
        print("!"*(len(data)+6))
        print("!!!"+data+"!!!")
        print("!"*(len(data)+6))

def example_read_joints():
    asmRobot=ASMRobot()
    robot_joints=asmRobot.get_robot_joints()
    print("Get robot_joints is:")
    print(list(map(int,robot_joints)))
    
def example_move_joints():
    asmRobot=ASMRobot()
    for i in range(10):
        asmRobot.home()
        time.sleep(2)
        
        target_robot_joints=[30,0,0,0,0,0]
        asmRobot.move_joints(target_robot_joints,flag_block=True,whole_time=3000,flag_debug=True)
        time.sleep(2)
    
def example_servo_joints():
    asmRobot=ASMRobot()
    asmRobot.home()
    
    asmRobot.move_joints([30,0,0,0,0,0],flag_debug=True)
    
    #init serovj joints
    print("Begin to servo joint")
    result=asmRobot.init_servoj()
    if not result:
        return
    
    #interpolate joints
    robot_joints=asmRobot.get_robot_joints()
    target_robot_joints=copy.deepcopy(robot_joints)
    target_robot_joints[0]=robot_joints[0]+40
    target_robot_joints[1]=robot_joints[1]+40
    target_robot_joints[2]=robot_joints[2]+40
    all_interpolate_joints=np.linspace(robot_joints,target_robot_joints,num=100)
    
    #servo interpolate joints
    for joints in all_interpolate_joints:
        asmRobot.servo_joints(joints)
        
    #close servoj
    time.sleep(0.5)
    result=asmRobot.init_servoj(flag=False)
    if not result:
        return
    print("Finish servo joint")
    
    time.sleep(2)
    asmRobot.home()
        
    
if __name__=="__main__":
    example_read_joints()
    # example_move_joints()
    # example_servo_joints()