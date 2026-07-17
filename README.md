# velocity_servo_tag

FR3 的 ROS 2 AprilTag 视觉伺服接口包框架。

```
velocity_servo_tag/
├── velocity_servo_tag/
│   ├── __init__.py
│   ├── velocity_mapper_node.py
│   ├── robot_kinematics.py
│   ├── safety.py
│   └── vision/
│       ├── __init__.py
│       ├── apriltag_detector.py
│       └── camera.py
├── config/
│   ├── velocity_servo_tag.yaml
│   └── urdf/
│       └── fr3.urdf
├── launch/
│   └── velocity_servo_tag.launch.py
├── resource/
│   └── velocity_servo_tag
├── test/
│   ├── test_safety.py
│   └── test_velocity_mapper.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 项目简介

`velocity_servo_tag` 是面向 Franka FR3 机械臂的 ROS 2 AprilTag
视觉伺服接口包，基于
[sunflower050105/franka_ros2](https://github.com/sunflower050105/franka_ros2)
运行。

本项目负责连接 USB 相机、Simulink 视觉控制律与 Franka 底层关节速度
控制器。AprilTag 检测节点从相机图像中提取目标中心坐标，并通过 ROS 2
Topic 发送给 Simulink；Simulink 根据视觉误差计算相机坐标系下的六维
速度指令；Python 速度映射节点完成手眼坐标变换、Jacobian 阻尼逆解、
关节速度限制、关节加速度限制和通信看门狗处理，最终向 Franka 底层
控制器发送七维关节速度命令。

系统控制链路如下：

```text
USB Camera
    → AprilTag Detector
    → Simulink Visual Servo Law
    → Camera Velocity
    → Coordinate Transformation
    → Jacobian Velocity Mapping
    → Safety Limiter and Watchdog
    → Franka Joint Velocity Controller
    → FR3 Robot
```

## ROS 接口

检测器发送目标位置的Topic（simulink接受）

- `/apriltag_detector/target_position` (`Float64MultiArray`):
  `[valid, u, v]`  

simulink发送相机坐标系下速度的Topic

- `/simulink/camera_velocity` (`Float64MultiArray`):
  `[vx, vy, vz, wx, wy, wz]`，在相机坐标系表达

Franka 发送关节速度，位置的Topic（最大1000hz）（simulink接收）

- `/franka/joint_states` (`JointState`): FR3 当前关节状态

Python代码发送关节速度的Topic（python里面发送）

- `/velocity_command_node/target_velocities` (`Float64MultiArray`):
  joint1 到 joint7 的速度命令

## 关键参数

所有参数在 `config/velocity_servo_tag.yaml`：

- `T_end_effector_camera`: 手眼标定变换；当前单位阵只是占位值
- `max_joint_velocities`: 7 个关节各自最大速度，rad/s
- `max_joint_accelerations`: 7 个关节各自最大加速度，rad/s^2
- `visual_velocity_timeout_sec`: Simulink 输入看门狗，默认 0.2 s
- `joint_state_timeout_sec`: 关节状态看门狗，默认 0.1 s
- `dry_run`: 默认 `true`，不向真机发布关节速度

关节速度超限时，节点对完整 7 维向量同比例缩放，避免逐关节裁剪改变
末端速度方向。输入超时、数据无效或 Jacobian 计算失败时，目标速度立即
置零，实际命令按每关节最大加速度平滑减速，并持续发布。

本节点的 0.2 s 看门狗用于防止 Simulink 数据中断。底层
`franka_velocity_ctrl` 仍应保留独立的命令超时保护，因为本 Python
进程本身崩溃时无法继续发布零速度；两层看门狗不能互相替代。



## 依赖项目

本项目需要配合以下 Franka ROS 2 工程使用：

- [sunflower050105/franka_ros2](https://github.com/sunflower050105/franka_ros2)

该工程负责提供 Franka ROS 2 驱动、机器人状态以及底层关节速度控制接口。
请先按照该工程的 README 完成依赖安装、编译和机器人连接配置，再编译并运行本项目。

推荐将本项目放入同一个 ROS 2 工作区，放到 `~/franka_ros2_ws/src` 下。

## 构建与运行

### 先检查依赖：

```bash
python3 -c "import pinocchio; print('pinocchio: OK')"
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
python3 -c "from pupil_apriltags import Detector; print('pupil_apriltags: OK')"
```

如果缺少 pupil-apriltags，可安装到当前用户环境：

```bash
python3 -m pip install --user --break-system-packages pupil-apriltags
```

不建议使用 sudo pip install，以免影响 Ubuntu 和 ROS 2 的系统 Python 环境。

### 构建

将本项目放入 ROS 2 工作区的 src 目录：

```bash
cd ~/franka_ros2_ws/src
git clone https://github.com/charliehu329/visual_servo_tag.git
```

如果仓库已经存在，不需要重复克隆。


安装 ROS 依赖并构建：

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select velocity_servo_tag \
  --symlink-install
```

构建完成后加载工作区：

```bash
source ~/franka_ros2_ws/install/setup.bash
```

每次打开新终端都需要重新执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

先用 dry-run 验证整条链路：

```bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py dry_run:=true
```

不启动相机检测器，只测试 Simulink 到机械臂的后半段：

```bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py \
  start_detector:=false dry_run:=true
```

确认手眼矩阵、方向、topic、限速和关门狗全部正确后，才使用：

```bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py dry_run:=false
```
