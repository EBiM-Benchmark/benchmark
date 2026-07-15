## 各功能包作用
* franka_gello_state_publisher - 读取Franka GELLO，发布左右臂和夹爪状态
* franka_gello_state_subscriber - 订阅 GELLO 状态，用于测试或后续接机器人控制
* keyboard_state_publisher   -   读取键盘wasdqe，发布键盘状态
* keyboard_state_subscriber  -   订阅键盘状态并打印对应按键
* pedal_state_publisher      -   读取USB三踏板状态，发布踏板状态
* pedal_state_subscriber     -   订阅踏板状态并打印对应动作
* franka_gripper_manager     -   Franka夹爪相关功能包
* franka_fr3_arm_controllers  -  FR3机械臂控制相关功能包

# 部署流程    
### 准备 Pixi ROS2 环境    

进入工作区：
```bash
cd ~/my_ros_ws
pixi shell
```

确认环境：
```bash
echo $CONDA_PREFIX
which ros2
python --version
echo $ROS_DISTRO
```

期望类似：
```bash
/home/demo/my_ros_ws/.pixi/envs/default
/home/demo/my_ros_ws/.pixi/envs/default/bin/ros2
Python 3.9.x
humble
```
如果没有 `pixi shell` 环境，需要先在新电脑上配置 Pixi 和 ROS2 Humble 环境。

### 安装 Python 依赖   
```bash
cd ~/my_ros_ws
pixi shell

pip install dynamixel-sdk tyro evdev
```

如果缺少 colcon：
```bash
pixi add colcon-core colcon-common-extensions
```

或者：

```bash
pip install colcon-core colcon-common-extensions
```

### 设置设备权限
GELLO 使用串口设备，需要 dialout 权限：
```bash
sudo usermod -aG dialout $USER
```
脚踏板使用 /dev/input/eventX，需要 input 权限：
```bash
sudo usermod -aG input $USER
```
执行后需要重新登录或重启。
确认：
```bash
groups
```
应包含：
```bash
dialout input
```

### 编译 ROS2 工作区
```bash
cd ~/my_ros_ws/gello_software/ros2
colcon build --symlink-install
source install/setup.bash
```

# GELLO

### 确认 GELLO USB 设备
插入 GELLO 后：
```bash
ls /dev/serial/by-id/
```
当前测试机器上的两个 GELLO 是：
```bash
usb-ROBOTIS_OpenRB-150_38F23AFA5157375037202020FF11170D-if00
usb-ROBOTIS_OpenRB-150_BDEDB3875157375037202020FF102618-if00
```
新电脑上 USB ID 可能不同，需要重新确认。

### GELLO Duo 配置文件
```bash
~/my_ros_ws/gello_software/ros2/src/franka_gello_state_publisher/config/franka_gello_duo.yaml
```
当前配置示例：
```YAML
LEFT:
  namespace: "left"
  com_port: "usb-ROBOTIS_OpenRB-150_38F23AFA5157375037202020FF11170D-if00"
  num_arm_joints: 7
  joint_signs: [1, 1, -1, 1, 1, 1, 1]
  gripper: true
  assembly_offsets: [4.712, 3.142, 4.712, 3.142, 4.712, 3.142, 4.712]
  gripper_range_rad: [2.521, 3.253]
  dynamixel_torque_enable: [0,0,0,0,0,0,0,0]
  dynamixel_goal_position: [0.0,0.0,0.0,-1.571,0.0,1.571,0.0,3.509]
  dynamixel_kp_p: [30,60,0,30,0,0,0,50]
  dynamixel_kp_i: [0,0,0,0,0,0,0,0]
  dynamixel_kp_d: [250,100,80,60,30,10,5,0]

RIGHT:
  namespace: "right"
  com_port: "usb-ROBOTIS_OpenRB-150_BDEDB3875157375037202020FF102618-if00"
  num_arm_joints: 7
  joint_signs: [1, 1, -1, 1, 1, 1, 1]
  gripper: true
  assembly_offsets: [1.571, 3.142, 1.571, 3.142, 1.571, 3.142, 0.000]
  gripper_range_rad: [2.570, 3.299]
  dynamixel_torque_enable: [0,0,0,0,0,0,0,0]
  dynamixel_goal_position: [0.0,0.0,0.0,-1.571,0.0,1.571,0.0,3.509]
  dynamixel_kp_p: [30,60,0,30,0,0,0,50]
  dynamixel_kp_i: [0,0,0,0,0,0,0,0]
  dynamixel_kp_d: [250,100,80,60,30,10,5,0]
```

注意：
```YAML
com_port
```
只写 USB ID，不写 `/dev/serial/by-id/`，launch 文件会自动拼接。

### 启动 GELLO publisher
终端 1：
```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 launch franka_gello_state_publisher main.launch.py \
  config_file:=franka_gello_duo.yaml
```

期望输出：
```bash
[left.gello_publisher]: Publishing GELLO joint states.
[right.gello_publisher]: Publishing GELLO joint states.
```

### 启动 GELLO subscriber
终端 2：
```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run franka_gello_state_subscriber franka_gello_state_subscriber
```

移动 GELLO 后，应能看到左右臂和夹爪状态变化。

# 键盘
### 启动 keyboard publisher
终端 1：
```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run keyboard_state_publisher keyboard_state_publisher
```

### 启动 keyboard subscriber
终端 2：
```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run keyboard_state_subscriber keyboard_state_subscriber
```

按下 w/a/s/d/q/e 后，subscriber 会打印对应按键或动作。

# USB 三踏板
### 硬件识别
插入脚踏板后：
```bash
ls -l /dev/input/by-id/
```

当前测试设备为：
```bash
usb-PCsensor_FootSwitch-event-kbd
usb-PCsensor_FootSwitch-event-mouse
usb-PCsensor_FootSwitch-event-if01
usb-PCsensor_FootSwitch-mouse
```

本项目使用 `/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd`

### 启动 pedal publisher
终端 1：
```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run pedal_state_publisher pedal_state_publisher
```

如果正常，会看到：
```bash
Using pedal device: /dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd PCsensor FootSwitch Keyboard
```

### 启动 pedal subscriber
终端 2：
```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run pedal_state_subscriber pedal_state_subscriber
```

# 常见问题
### 新电脑 USB ID 不同
GELLO 的 USB ID 在新电脑上可能不同。重新查看：
```bash
ls /dev/serial/by-id/
```
然后修改：
```bash
franka_gello_state_publisher/config/franka_gello_duo.yaml
```
脚踏板设备查看：
```bash
ls -l /dev/input/by-id/
```
如果名称不同，需要修改：
```bash
PEDAL_DEVICE = "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"
```






