
# 使用说明
在Ubuntu20.04上进行测试并实现
## 1. 设置

1. 启用USB权限
   在终端执行以下命令，授予USB访问权限：

2. 运行代码
进入包含Python SDK的目录，使用Python 3执行`asm_robot.py`脚本：



# 使用说明

## 1. 设置
1. 使能USB权限
   ```
   sudo chomd 777 /dev/ttyUSB0
   ```

2. 运行代码
   ```
   cd /path/to/pyhtonSDK
   python3 asm_robot.py
   ```


## 2. 代码
`asm_sdk`基于DYNAMIXEL通信接口，你可以在[DynamixelSDK Python仓库](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python)找到详细文档。
目前支持基本功能，如读取电机角度值、关节运动(`movej`)和关节伺服(`servoj`)。每种功能的具体示例都可以在`asm_robot.py`文件中找到，位于`example_*_joints()`方法中：

```
if __name__=="__main__":
    example_read_joints()
    # example_move_joints()
    # example_servo_joints()
```