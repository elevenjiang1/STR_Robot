# Usage
Tested on Ubuntu 20.04
中文版请查看 [this link](./README_ZH.md)


## 1. Setup
1. Enable USB Permissions
   Execute the following command in your terminal to grant USB access permissions:
   ```
   sudo chmod 777 /dev/ttyUSB0
   ```


2. Run the Code
    Navigate to the directory containing the Python SDK, and execute the `asm_robot.py` script using Python 3:
    ```
    cd /path/to/pyhtonSDK
    python3 asm_robot.py
    ```


## 2. Code

The `asm_sdk` is based on the DYNAMIXEL communication interface, which you can find detailed documentation for at the [DynamixelSDK Python repository](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python).
Currently, it supports basic functionalities like reading motor angle values, and joint movements (`movej` and `servoj`). Specific examples for each of these functions can be found in the `asm_robot.py` file, under `example_*_joints()` methods:
```
if __name__=="__main__":
    example_read_joints()
    # example_move_joints()
    # example_servo_joints()
```
