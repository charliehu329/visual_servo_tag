# velocity_servo_tag

面向 Franka FR3 的 ROS 2 AprilTag 视觉伺服包。

仓库当前包含两部分：已经验证可用的单目 Python/Simulink 链路，以及正在接入的主动变焦双目 Simulink v2 控制器。Simulink v2 直接输出左相机坐标系下的六维速度，Python 继续负责相机速度到七维关节速度的映射和真机安全转发。

本项目集成了：

- AprilTag 检测
- Simulink 视觉控制接口
- 主动变焦双目 IBVS 与 EKF 控制模型
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
├── simulinkv2/
│   ├── arm_stereo_ibvs_ekf_v1.slx
│   ├── init_arm_stereo_ibvs_ekf_v1.m
│   ├── build_arm_stereo_ibvs_ekf_v1.m
│   └── START_HERE.m
├── resource/
│   └── velocity_servo_tag
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 系统功能

`velocity_servo_tag` 连接相机、Simulink 视觉控制器和 Franka FR3。

AprilTag 检测节点读取相机图像并发布目标位置。Simulink 根据视觉误差计算相机坐标系下的六维速度。Python 节点完成：

1. 相机坐标系到机械臂末端坐标系的速度变换；
2. Jacobian 阻尼伪逆；
3. 六维末端速度到七维关节速度的映射；
4. 关节速度限制；
5. 关节加速度限制；
6. 输入与关节状态看门狗；
7. 向 Franka 速度控制器发送命令。

已验证的单目链路如下：

```text
USB Camera
    → AprilTag Detector
    → Simulink Visual Servo Controller
    → /simulink/camera_velocity
    → Velocity Mapper
    → /velocity_mapper_node/target_joints_velocities
    → Velocity Command Node
    → /joint_velocity_example_controller/commands
    → Franka FR3
```

Simulink v2 的目标链路如下：

```text
Stereo Camera
    → vision_double
    → /vision_double/target_features
                                      \
/franka/joint_states -----------------→ Simulink Stereo IBVS + EKF
/vision_double/focal_lengths_mm ------/          │
                                                  ├→ /simulink/camera_velocity
                                                  │      → Velocity Mapper
                                                  │      → Velocity Command Node
                                                  │      → Franka FR3
                                                  └→ /simulink/zoom_velocities_mm_s
                                                         → Zoom Driver
                                                         → Left/Right Lens
```

这里不再让 Simulink 先算 `qDot`、再经 Jacobian 还原相机速度。Simulink 原生输出左相机六维速度，关节速度只在 Python Mapper 中计算一次。

## ROS 2 接口

### 现有单目 AprilTag 检测结果

```text
/apriltag_detector/target_position
```

消息类型：

```text
std_msgs/msg/Float64MultiArray
```

数据格式：

```text
[valid, u, v]
```

通信关系：

发布者：apriltag_detector
订阅者：Simulink 视觉控制器

其中：

- `valid`：是否检测到目标，检测到为 `1`
- `u`：目标中心横向像素坐标
- `v`：目标中心纵向像素坐标

`layout.dim: []` 和 `data_offset: 0` 是 `Float64MultiArray` 的默认数组描述，不属于控制数据。

该接口由当前 `apriltag_detector` 使用，是已验证单目链路的一部分；Simulink v2 改用下面的双目接口。

### Simulink v2 双目特征

```text
/vision_double/target_features
```

消息类型：`std_msgs/msg/Float64MultiArray`

```text
[validL, validR, uL, vL, uR, vR, scaleL, scaleR]
```

- `validL`、`validR`：左右相机目标有效标志，检测到为 `1`
- `uL, vL, uR, vR`：左右图像中的 AprilTag 中心像素坐标
- `scaleL`、`scaleR`：左右图像中 AprilTag 的像素边长

发布者将由待实现的 `vision_double` 节点提供，订阅者为 Simulink v2。

### 双目焦距反馈

```text
/vision_double/focal_lengths_mm
```

消息类型：`std_msgs/msg/Float64MultiArray`

```text
[fL_mm, fR_mm]
```

表示左右镜头的实际焦距，单位为 `mm`。该数据是 Simulink v2 有效计算相机速度和闭环变焦的必要输入，不能只发送一次初值后停止更新。

### Simulink v2 变焦速度

```text
/simulink/zoom_velocities_mm_s
```

消息类型：`std_msgs/msg/Float64MultiArray`

```text
[fDotL, fDotR]
```

单位为 `mm/s`，由 Simulink v2 发布，待变焦驱动节点订阅并转换为镜头硬件命令。

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

通信关系：

发布者：Simulink 视觉控制器
订阅者：velocity_mapper_node

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

通信关系：

发布者：joint_state_broadcaster
订阅者：velocity_mapper_node、Simulink及其他状态监控节点

包含 FR3 七个关节的位置和速度。实际话题名称可以在参数文件中修改。

Python Mapper 会按 `JointState.name` 映射 `fr3_joint1` 到 `fr3_joint7`。当前 Simulink v2 暂时直接使用 `position(1:7)`，真机运行前必须确认 `/franka/joint_states` 的实际顺序，或把模型改为按名称映射。

### 关节速度映射结果

```text
/velocity_mapper_node/target_joints_velocities
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

通信关系：

发布者：velocity_mapper_node
订阅者：velocity_command_node

### 底层控制器命令

```text
/joint_velocity_example_controller/commands
```

消息类型：

```text
std_msgs/msg/Float64MultiArray
```

通信关系：

发布者：velocity_command_node
订阅者：joint_velocity_example_controller

该话题由本包中的 `velocity_command_node` 发布，直接连接 Franka 关节速度控制器。

## Simulink v2

主要文件：

- `simulinkv2/arm_stereo_ibvs_ekf_v1.slx`：当前双目视觉伺服模型
- `simulinkv2/init_arm_stereo_ibvs_ekf_v1.m`：从统一 YAML 读取接口并初始化模型参数
- `simulinkv2/build_arm_stereo_ibvs_ekf_v1.m`：构建并连接 ROS 2 收发块
- `simulinkv2/START_HERE.m`：加载初始化参数并打开模型

模型使用左相机中心保持作为主要任务，结合双目尺度/逆深度估计、右相机可见性保护和双镜头变焦控制，最终输出左相机坐标系下的六维速度：

```text
[vx, vy, vz, wx, wy, wz]
```

### 构建与打开

在 MATLAB 中执行：

```matlab
cd('<仓库路径>/simulinkv2')
init_arm_stereo_ibvs_ekf_v1
build_arm_stereo_ibvs_ekf_v1
START_HERE
```

ROS 2 topic 统一配置在 `config/velocity_servo_tag.yaml` 的 `simulink_ros2` 段。当前构建脚本会把 topic 字符串写入 ROS 2 Block，因此修改 topic 后需要重新运行 `build_arm_stereo_ibvs_ekf_v1`；仅运行 `START_HERE` 不会重写这些 Block。

### 实时运行要求

模型固定步长为 `1/60 s`，但普通 Simulink 仿真的默认设置不等于墙钟实时运行。桌面联机控制时必须启用 **Simulation Pacing = 1x**，或使用经过验证的实时/部署执行方式，否则 ROS 发布频率和 `0.20 s` 输入看门狗都可能失真。

### 当前验证范围

目前已经确认：

- 模型可构建，Update Diagram 通过
- ROS 2 topic 名称已正确写入 Block
- `Float64MultiArray` 发布总线和有效长度正确
- 相机速度输出为原生六维左相机速度
- Python Mapper 到 Franka 控制器的消息维度和单位一致

目前尚未确认：

- `vision_double`、焦距反馈和变焦驱动尚未实现，真实双目闭环未跑通
- 真机 `/franka/joint_states` 的数组顺序尚未核对
- 手眼、双目外参、主点、像素尺寸与焦距换算仍需实际标定
- 按墙钟 60 Hz 的完整 ROS 在线闭环尚未验证

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
- `simulink_ros2`：集中记录 Simulink v2 的五个 topic、消息长度、采样周期和输入超时；当前 topic 修改后仍需重建 SLX

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
ros2 launch velocity_servo_tag full_system.launch.py \
  robot_ip:=172.16.0.2 \
  start_hardware:=true \
  dry_run:=true \
  command_mode:=zero \
  start_detector:=true \
  use_rviz:=false
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

### 5. Dry-run 验证 Simulink v2 接口

在不连接真机、Python Mapper 保持 `dry_run:=true` 的条件下，依次确认：

```bash
ros2 topic echo /vision_double/target_features
ros2 topic echo /vision_double/focal_lengths_mm
ros2 topic echo /franka/joint_states
ros2 topic echo /simulink/camera_velocity
ros2 topic echo /simulink/zoom_velocities_mm_s
```

同时检查消息维度、有限数值、坐标系、单位和发布频率。断开视觉、焦距或关节状态的任意一路时，相机速度与变焦速度都应在看门狗超时后安全归零。

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

### 一体化真机启动参数说明

```bash
ros2 launch velocity_servo_tag full_system.launch.py \
  robot_ip:=172.16.0.2 \
  start_hardware:=true \
  dry_run:=false \
  command_mode:=topic \
  start_detector:=true \
  use_rviz:=false
```

各参数含义：

* `robot_ip:=172.16.0.2`
  Franka FR3 控制器的 IP 地址。

* `start_hardware:=true`
  启动 Franka 硬件驱动、状态广播器和关节速度控制器。设为 `false` 时不会连接真机。

* `dry_run:=false`
  允许 `velocity_mapper_node` 发布计算得到的七维关节速度。设为 `true` 时只计算，不向下游发布。

* `command_mode:=topic`
  让 `velocity_command_node` 接收并转发上层关节速度。设为 `zero` 时忽略上层速度，持续向机器人发送七维零速度。

* `start_detector:=true`
  启动 USB 相机和 AprilTag 检测节点。设为 `false` 时不启动检测器，适合单独测试 Simulink 后半段。

* `use_rviz:=false`
  不启动 RViz。设为 `true` 时同时打开 RViz 显示机器人模型。

> `start_hardware:=true + dry_run:=false + command_mode:=topic` 会允许非零速度到达真机，仅在手眼标定、方向、限速和看门狗均验证完成后使用。

> 当前 `full_system.launch.py` 启动的是旧版单目 `apriltag_detector`，不会提供 Simulink v2 所需的双目特征、焦距反馈和变焦驱动。完成 `vision_double` 后还需要更新 launch，才能一体化启动双目链路。




### 一体化启动

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

使用本包的一体化启动文件：

```bash
ros2 launch velocity_servo_tag full_system.launch.py \
  robot_ip:=172.16.0.2 \
  start_hardware:=true \
  dry_run:=true \
  command_mode:=zero \
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
  start_hardware:=true \
  dry_run:=false \
  command_mode:=topic \
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

检查 Simulink v2 输入：

```bash
ros2 topic echo /vision_double/target_features
ros2 topic echo /vision_double/focal_lengths_mm
ros2 topic echo /franka/joint_states
```

检查 Simulink 相机速度：

```bash
ros2 topic echo /simulink/camera_velocity
```

检查变焦速度：

```bash
ros2 topic echo /simulink/zoom_velocities_mm_s
```

检查关节速度目标：

```bash
ros2 topic echo /velocity_mapper_node/target_joints_velocities
```

检查底层控制器命令：

```bash
ros2 topic echo /joint_velocity_example_controller/commands
```

检查发布频率：

```bash
ros2 topic hz /simulink/camera_velocity
ros2 topic hz /simulink/zoom_velocities_mm_s
ros2 topic hz /velocity_mapper_node/target_joints_velocities
ros2 topic hz /joint_velocity_example_controller/commands
```

检查话题发布者，确保关键速度话题只有预期的一个发布者：

```bash
ros2 topic info -v /velocity_mapper_node/target_joints_velocities
ros2 topic info -v /joint_velocity_example_controller/commands
```

## 停止系统

优先使用 `Ctrl+C` 正常停止 launch。

`velocity_command_node` 在正常退出前会发送零速度。停止后检查控制器状态：

```bash
ros2 control list_controllers
```

如果发生异常运动、方向错误或 Franka reflex，应立即停止系统，不要直接提高速度限制或关闭安全保护。

## 下一步接入接口

建议按下面顺序完成，前三项是 Simulink v2 真正运行所缺的接口：

| 顺序 | 接口 | 方向 | 数据格式 | 下一步工作 |
|---|---|---|---|---|
| 1 | `/vision_double/target_features` | Python → Simulink | `[validL, validR, uL, vL, uR, vR, scaleL, scaleR]` | 实现双相机同步采集和 AprilTag 检测 |
| 2 | `/vision_double/focal_lengths_mm` | 镜头/相机节点 → Simulink | `[fL_mm, fR_mm]` | 发布左右镜头实际焦距反馈 |
| 3 | `/simulink/zoom_velocities_mm_s` | Simulink → 变焦驱动 | `[fDotL, fDotR]` | 实现限位、限速、超时归零和硬件单位换算 |
| 4 | `/franka/joint_states` | Franka → Simulink/Python | `sensor_msgs/JointState` | 采集真实消息，核对 `name`、`position` 顺序和长度 |
| 5 | `/simulink/camera_velocity` | Simulink → Python | `[vx, vy, vz, wx, wy, wz]` | 已接通；标定后复核方向、单位和限幅 |
| 6 | `/velocity_mapper_node/target_joints_velocities` | Mapper → Command | 七维 `rad/s` | 已接通，保持为唯一关节速度映射出口 |
| 7 | `/joint_velocity_example_controller/commands` | Command → FR3 | 七维 `rad/s` | 已接通，继续保留第二层看门狗和限速 |

## 待办清单

### P0：真实闭环前必须完成

- [ ] 实现 `vision_double`，稳定发布 8 维双目特征并校验实际数组长度
- [ ] 接入左右焦距反馈；实现变焦驱动及硬件限位/限速
- [ ] 获取一条真实 `/franka/joint_states` 完整消息，确认前七个 `position` 是否严格对应 `fr3_joint1` 到 `fr3_joint7`；不一致时把 Simulink 改为按 `name` 映射
- [ ] 完成 `T_end_effector_camera`、`T_link8_CL`、双目外参、主点、像素尺寸和焦距到像素的标定，并确保 Python 与 Simulink 使用同一套手眼关系
- [ ] 桌面运行时启用 Simulation Pacing 1x，实测所有关键 topic 的墙钟频率
- [ ] 修复 Simulink 输入安全逻辑：连续非法/NaN 消息也必须累计超时；视觉、焦距或关节状态超时后，相机速度和变焦速度都必须归零
- [ ] 用 `dry_run:=true` 完成全 ROS 在线端到端测试，再用极低速度和急停保护进行真机测试

### P1：控制效果与鲁棒性

- [ ] 验证 EKF 在真实初始位姿下的首次收敛；必要时按首个有效双目测量重置状态
- [ ] 处理视觉、焦距和关节状态的时间同步，避免混用不同时间的输入
- [ ] 评估 Python 关节限速/加速度限制后“命令相机速度”与“实际相机速度”的偏差，必要时用实际关节速度反馈更新 `rhoDynamics`
- [ ] 验证 AprilTag 倾斜时像素边长尺度模型的误差
- [ ] 在保持左目标尽量居中的前提下，验证右相机可见性保护和变焦恢复策略
- [ ] 从低速开始重新整定 Simulink 相机速度上限、Python 关节限速和控制增益

### P2：工程整理

- [ ] 将 `vision_double`、焦距反馈和变焦驱动加入 `full_system.launch.py`
- [ ] 让 YAML 的采样周期和消息长度真正驱动模型；改进当前正则读取方式，避免同名键冲突
- [ ] topic 或接口参数改变后自动触发/提示重建 SLX
- [ ] 清理或忽略 MATLAB 生成物：`slprj/`、`*.slxc`、`+bus_conv_fcns/` 和临时测试结果
- [ ] 双目链路验证完成后，将 README 中“正在接入”的状态更新为“已验证”
