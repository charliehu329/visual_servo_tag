# velocity_servo_tag

面向 Franka FR3 的 ROS 2 AprilTag 单目视觉伺服包。

本项目集成了：

- AprilTag 检测
- Simulink 视觉控制接口
- 相机速度到关节速度映射
- 关节速度与加速度限制
- 通信看门狗
- Franka 底层速度命令转发
- FR3 硬件和控制器启动

不再依赖单独的 `franka_velocity_ctrl` 包。

## 项目结构

```text
velocity_servo_tag/
├── velocity_servo_tag/
│   ├── __init__.py
│   ├── velocity_command_node.py
│   ├── velocity_mapper_node.py
│   ├── robot_kinematics.py
│   ├── safety.py
│   └── vision/
│       ├── __init__.py
│       ├── apriltag_detector.py
│       └── camera.py
├── config/
│   ├── controllers.yaml
│   ├── velocity_servo_tag.yaml
│   └── urdf/
│       └── fr3.urdf
├── launch/
│   ├── fr3_hardware.launch.py
│   ├── full_system.launch.py
│   └── velocity_servo_tag.launch.py
├── resource/
│   └── velocity_servo_tag
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 系统功能

`velocity_servo_tag` 连接 USB 相机、Simulink 视觉控制器和 Franka FR3。

AprilTag 棟测节点读取相机图像并发布目标位置。Simulink 根据视觉误差计算相机坐标系下的六维速度。Python 节点完成：

1. 相机坐标系到机械臂末端坐标系的速度变换；
2. Jacobian 阻尼伪逆；
3. 六维末端速度到七维关节速度的映射；
4. 关节速度限制；
5. 关节加速度限制；
6. 输入与关节状态看门狗；
7. 向 Franka 速度控制器发送命令。

控制链路如下：

```text
USB Camera
    → AprilTag Detector
    → Simulink Visual Servo Controller
    → /simulink/camera_velocity
    → Velocity Mapper
    → /velocity_command_node/target_velocities
    → Velocity Command Node
    → /joint_velocity_example_controller/commands
    → Franka FR3
```

## ROS 2 接口

### AprilTag 检测结果

```text
/apriltag_detector/target_position
```

消息类型：

```text
std_msgs/msg/Float64MultiArray
```

数据格式：

```text
[valid, tag_id, u, v]
```

其中：

- `valid`：是否检测到目标，检测到为 `1`
- `tag_id`：AprilTag ID
- `u`：目标中心横向像素坐标
- `v`：目标中心纵向像素坐标

`layout.dim: []` 和 `data_offset: 0` 是 `Float64MultiArray` 的默认数组描述，不属于控制数据。

### Simulink 相机速度

```text
/simulink/camera_velocity
```

消息类型：

```text
std_msgs/msg/Float64MultiArray
```

数据格式：

```text
[vx, vy, vz, wx, wy, wz]
```

速度在相机坐标系中表达：

- `vx, vy, vz`：线速度，单位 `m/s`
- `wx, wy, wz`：角速度，单位 `rad/s`

### Franka 关节状态

```text
/franka/joint_states
```

消息类型：

```text
sensor_msgs/msg/JointState
```

包含 FR3 七个关节的位置和速度。实际话题名称可以在参数文件中修改。

### 关节速度映射结果

```text
/velocity_command_node/target_velocities
```

消息类型：

```text
std_msgs/msg/Float64MultiArray
```

数据格式：

```text
[dq1, dq2, dq3, dq4, dq5, dq6, dq7]
```

单位为 `rad/s`。

### 底层控制器命令

```text
/joint_velocity_example_controller/commands
```

消息类型：

```text
std_msgs/msg/Float64MultiArray
```

该话题由本包中的 `velocity_command_node` 发布，直接连接 Franka 关节速度控制器。

## 安全机制

系统包含两层速度安全处理。

### Velocity Mapper

`velocity_mapper_node` 负责：

- 检查相机速度是否有效
- 检查关节状态是否超时
- Jacobian 异常保护
- 关节速度同比例缩放
- 关节加速度限制
- 输入超时后平滑减速到零
- `dry_run` 模式保护

关节速度超限时，对完整七维向量同比例缩放，避免逐关节裁剪改变末端运动方向。

### Velocity Command Node

`velocity_command_node` 负责：

- 检查输入是否为七维
- 检查输入是否包含 `NaN` 或 `Inf`
- 再次限制关节速度
- 命令超时后发送零速度
- 按固定频率向底层控制器发布
- 节点退出前发送零速度

即使上层映射节点停止发布，底层命令节点也会在超时后主动发送零速度。

> 软件保护不能替代 Franka 自身的碰撞保护、速度限制和急停装置。

## 关键参数

主要参数位于：

```text
config/velocity_servo_tag.yaml
```

关键参数包括：

- `T_end_effector_camera`：末端到相机的手眼标定变换
- `max_joint_velocities`：七个关节的最大速度，单位 `rad/s`
- `max_joint_accelerations`：七个关节的最大加速度，单位 `rad/s²`
- `visual_velocity_timeout_sec`：Simulink 速度输入看门狗
- `joint_state_timeout_sec`：关节状态看门狗
- `dry_run`：是否禁止向底层控制节点发送非零速度

底层控制器配置位于：

```text
config/controllers.yaml
```

手眼矩阵当前如果仍为单位阵，则只能用于软件链路测试，不能直接进行真机视觉伺服。

## 依赖

### 系统环境

推荐环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Franka FR3
- libfranka
- franka_ros2
- Python 3.12
- MATLAB/Simulink ROS Toolbox

Franka ROS 2 驱动使用：

[https://github.com/sunflower050105/franka_ros2](https://github.com/sunflower050105/franka_ros2)

本项目仍依赖 `franka_ros2` 提供硬件接口和控制器插件，但不再依赖原来的 `franka_velocity_ctrl` Python 包。

### Python 依赖

检查依赖：

```bash
python3 -c "import pinocchio; print('pinocchio: OK')"
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
python3 -c "from pupil_apriltags import Detector; print('pupil_apriltags: OK')"
```

推荐在虚拟环境中安装 `pupil-apriltags`：

```bash
cd ~/franka_ros2_ws

python3 -m venv --system-site-packages .venv
source .venv/bin/activate

python3 -m pip install --upgrade pip
python3 -m pip install pupil-apriltags
```

使用 `--system-site-packages` 可以让虚拟环境访问 ROS 2 已安装的 `rclpy`。

不建议使用：

```bash
sudo pip install
```

如果不使用虚拟环境，也可以安装到当前用户环境：

```bash
python3 -m pip install \
  --user \
  --break-system-packages \
  pupil-apriltags
```

## 构建

将项目放到 ROS 2 工作区：

```bash
cd ~/franka_ros2_ws/src

git clone https://github.com/charliehu329/visual_servo_tag.git
```

如果仓库已经存在，不要重复克隆。

安装 ROS 依赖：

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

构建：

```bash
colcon build \
  --packages-select velocity_servo_tag \
  --symlink-install
```

加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

每次打开新终端都需要重新加载环境。如果使用虚拟环境，还需要执行：

```bash
source ~/franka_ros2_ws/.venv/bin/activate
```

## 分阶段验证

### 1. 检查节点是否安装成功

```bash
ros2 pkg executables velocity_servo_tag
```

应至少能够看到：

```text
velocity_servo_tag apriltag_detector
velocity_servo_tag velocity_mapper_node
velocity_servo_tag velocity_command_node
```

### 2. 只运行 AprilTag 检测器

```bash
ros2 run velocity_servo_tag apriltag_detector \
  --ros-args \
  -p camera_fps:=60.0 \
  -p quad_decimate:=1.0
```

检查检测结果：

```bash
ros2 topic echo /apriltag_detector/target_position
```

### 3. Dry-run 验证视觉伺服链路

启动视觉伺服节点，但不向真机发送关节速度：

```bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py \
  dry_run:=true
```

不启动检测器，只测试 Simulink 到速度映射节点的后半段：

```bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py \
  start_detector:=false \
  dry_run:=true
```

检查 Simulink 输出：

```bash
ros2 topic echo /simulink/camera_velocity
```

检查映射节点状态：

```bash
ros2 node info /velocity_mapper_node
```

### 4. 手动发送相机速度测试

必须保持：

```text
dry_run:=true
```

然后在另一个终端发送很小的测试速度：

```bash
ros2 topic pub --rate 20 \
  /simulink/camera_velocity \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.005, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

这一步只用于确认：

- Simulink 速度话题能够被接收
- 坐标变换能够运行
- Jacobian 能够计算
- 七维关节速度能够生成
- 限速和看门狗能够工作

`dry_run:=true` 时机械臂不会执行该速度。

## 真机运行

首次真机运行前必须确认：

- Franka 急停可用
- 机械臂周围没有人员和障碍物
- 手眼矩阵已经标定
- 相机速度方向已经验证
- Jacobian 使用的末端坐标系正确
- 关节顺序为 `fr3_joint1` 到 `fr3_joint7`
- 速度和加速度限制足够小
- Simulink 使用实时节拍运行
- 所有看门狗均已验证

### 一体化启动

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

使用本包的一体化启动文件：

```bash
ros2 launch velocity_servo_tag full_system.launch.py \
  robot_ip:=172.16.0.2 \
  dry_run:=true \
  start_detector:=true \
  use_rviz:=false
```

该命令启动：

- Franka 硬件接口
- 关节状态广播器
- Franka 状态广播器
- 关节速度控制器
- `velocity_command_node`
- `velocity_mapper_node`
- AprilTag 检测器

先在 `dry_run:=true` 下确认所有话题和状态正常。

完成所有检查后，才允许使用：

```bash
ros2 launch velocity_servo_tag full_system.launch.py \
  robot_ip:=172.16.0.2 \
  dry_run:=false \
  start_detector:=true \
  use_rviz:=false
```

然后启动 Simulink 控制器。

## 运行状态检查

检查节点：

```bash
ros2 node list
```

检查控制器：

```bash
ros2 control list_controllers
```

需要确认速度控制器和状态广播器为：

```text
active
```

检查关键话题：

```bash
ros2 topic list
```

检查 AprilTag 数据：

```bash
ros2 topic echo /apriltag_detector/target_position
```

检查 Simulink 相机速度：

```bash
ros2 topic echo /simulink/camera_velocity
```

检查关节速度目标：

```bash
ros2 topic echo /velocity_command_node/target_velocities
```

检查底层控制器命令：

```bash
ros2 topic echo /joint_velocity_example_controller/commands
```

检查发布频率：

```bash
ros2 topic hz /simulink/camera_velocity
ros2 topic hz /velocity_command_node/target_velocities
ros2 topic hz /joint_velocity_example_controller/commands
```

检查话题发布者，确保关键速度话题只有预期的一个发布者：

```bash
ros2 topic info -v /velocity_command_node/target_velocities
ros2 topic info -v /joint_velocity_example_controller/commands
```

## 停止系统

优先使用 `Ctrl+C` 正常停止 launch。

`velocity_command_node` 在正常退出前会发送零速度。停止后检查控制器状态：

```bash
ros2 control list_controllers
```

如果发生异常运动、方向错误或 Franka reflex，应立即停止系统，不要直接提高速度限制或关闭安全保护。