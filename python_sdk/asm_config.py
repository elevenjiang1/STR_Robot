class ASMConfig():
    # Control table address
    
    BAUDRATE                    = 2000000
    DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
    DXL_ID = [1, 2, 3, 4, 5, 6]
    DXL_MINIMUM_POSITION_VALUE  = -5000      # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 5000       # Refer to the Maximum Position Limit of product eManual
    
    
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    
    ADDR_GOAL_POSITION          = 564
    LEN_GOAL_POSITION           = 4          # Data Byte Length
    
    ADDR_MOVEMENT_DURATION        = 522
    LEN_MOVEMENT_DURATION         = 2          # Data Byte Length
    
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4          # Data Byte Length
    
    ADDR_DRIVE_MODE             = 10
    
    
    
    
    
    
    
    
    PROFILE_ENABLE              = 0x0;              # Value for enable trajectory profile
    PROFILE_DISABLE             = 0x02;             # Value for disable trajectory profile
    
    
    