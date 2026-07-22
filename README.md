# visual_servo_tag

面向 Franka Research 3（FR3）的双目主动变焦视觉伺服项目。项目使用 Simulink 完成视觉控制、机器人运动学和算法层安全保护，并直接输出七维关节速度；Python ROS 2 节点负责最终限速、加速度限制、通信超时保护和底层控制器转发。

> 当前版本处于 Stage 1。Core 模型和 ROS 2 包装模型已经完成，ROS 2 消息收发已经验证；尚未完成双目相机联合实验、真实标定和 FR3 真机闭环验证。

## 项目目标

- 使用双目视觉特征控制 FR3 跟踪目标；
- 使用同一个 Simulink Core 支持 ROS 2 部署和后续离线仿真；
- 在后续阶段加入双目逆深度、目标 EKF、关节限位零空间任务和主动变焦；
- 将算法层安全与硬件发送层安全分离，便于验证和部署。

## 当前状态

| 模块 | 状态 | 说明 |
|---|---|---|
| `stereo_ibvs_core.slx` | 已完成 | Stage 1 固定内参、固定逆深度、左相机中心 IBVS |
| `stereo_ibvs_ros2_stage1.slx` | 已完成 | 5 个 ROS 2 输入、3 个 ROS 2 输出，消息收发已验证 |
| ROS 2 通信测试 | 已完成 | 已验证包装模型能够订阅和发布消息 |
| 双目相机联合实验 | 未完成 | `/vision_double/*` 数据源尚未与本模型联合验证 |
| 真实相机与手眼标定 | 未完成 | 当前配置仍为安全占位值 |
| FR3 真机闭环 | 未完成 | 标定和超时保护完成前禁止非零真机运动 |
| Stage 2～6 | 规划中 | 逆深度、零空间、EKF、主动变焦和工程强化 |

本 README 只介绍 Core 和 ROS 2 Stage 1。离线 sim 模型仍在开发，因此暂不提供 sim 使用方法。

## 系统架构

```text
双目视觉节点
  ├─ /vision_double/target_features
  └─ /vision_double/zoom_position_steps
                                  \
FR3 JointState --------------------→ stereo_ibvs_ros2_stage1.slx
控制使能与复位 --------------------/              │
                                                   ▼
                                         stereo_ibvs_core.slx
                                                   │
                    ┌──────────────────────────────┼──────────────────────────┐
                    ▼                              ▼                          ▼
 /simulink/target_joints_velocities   /simulink/zoom_step_rate_cmd   /simulink/controller_status
                    │
                    ▼
          velocity_command_node
                    │
                    ▼
 /joint_velocity_example_controller/commands
                    │
                    ▼
                  FR3
```

职责分工：

- Simulink Core：输入检查、相机模型、FR3 运动学、IBVS、算法限速、软关节限位和诊断；
- ROS 2 包装模型：ROS 消息收发、数组长度检查、总线转换和单周期复位脉冲；
- `velocity_command_node`：最终速度限制、加速度限制、watchdog 和底层命令发布；
- 双目视觉和变焦驱动：当前由外部节点提供，尚未纳入本仓库的 Stage 1 联合实验。

## 目录结构

```text
visual_servo_tag/
├── config/
│   ├── controllers.yaml
│   ├── velocity_servo_tag.yaml
│   └── urdf/fr3.urdf
├── launch/
│   ├── fr3_hardware.launch.py
│   ├── full_system.launch.py
│   └── velocity_servo_tag.launch.py
├── simulink/
│   ├── config/stereo_ibvs_config.m
│   ├── core/stereo_ibvs_core.slx
│   ├── ros2/stereo_ibvs_ros2_stage1.slx
│   └── sim/build_stereo_ibvs_sim_stage1.m
├── velocity_servo_tag/
│   ├── safety.py
│   ├── velocity_command_node.py
│   └── vision/
│       ├── apriltag_detector.py
│       └── camera.py
├── package.xml
├── setup.py
└── README.md
```

`slprj/`、`*.slxc`、`*.autosave`、`*.asv` 和 ROS 总线转换缓存不是主要源文件。

## 环境要求

当前目标和验证环境：

- Ubuntu 24.04；
- ROS 2 Jazzy；
- Python 3.12；
- MATLAB R2025b；
- Simulink；
- ROS Toolbox；
- Robotics System Toolbox；
- Franka FR3、libfranka 和适配 ROS 2 Jazzy 的 `franka_ros2`。

Python 侧主要依赖：

- `rclpy`；
- `numpy`；
- OpenCV；
- `pupil-apriltags`。

## 安装与构建

### 1. 准备 ROS 2 工作区

将仓库放入工作区的 `src/`：

```bash
mkdir -p ~/franka_ros2_ws/src
cd ~/franka_ros2_ws/src
git clone https://github.com/charliehu329/visual_servo_tag.git
```

如果仓库已经存在，不要重复克隆。

### 2. 安装 ROS 依赖

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

AprilTag 检测器建议安装在能够访问系统 ROS 2 包的虚拟环境中：

```bash
cd ~/franka_ros2_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install pupil-apriltags
```

### 3. 构建 ROS 2 包

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select velocity_servo_tag
source install/setup.bash
```

每个新终端都需要重新加载 ROS 2 和工作区环境。

## Stage 1 ROS 2 使用方法

### 1. 检查 ROS 2 网络

MATLAB、双目视觉节点和 FR3 ROS 2 节点需要位于可互相发现的 ROS 2 网络，并使用相同的 `ROS_DOMAIN_ID`。

```bash
echo "$ROS_DOMAIN_ID"
ros2 node list
ros2 topic list
```

### 2. 打开 ROS 2 包装模型

在 MATLAB 中执行：

```matlab
repoDir = '/绝对路径/visual_servo_tag';
modelFile = fullfile(repoDir, 'simulink', 'ros2', ...
    'stereo_ibvs_ros2_stage1.slx');
open_system(modelFile);
```

模型的回调会自动加入 `simulink/core/` 和 `simulink/config/` 路径，并运行 `stereo_ibvs_config.m`。

如果首次打开时短暂提示找不到引用模型，可执行：

```matlab
addpath(fullfile(repoDir, 'simulink', 'core'));
addpath(fullfile(repoDir, 'simulink', 'config'));
run(fullfile(repoDir, 'simulink', 'config', ...
    'stereo_ibvs_config.m'));
```

随后在 Simulink 中执行 Update Diagram（`Ctrl+D`）。

### 3. 准备输入 Topic

运行模型前，应持续提供以下数据：

- `/franka/joint_states`；
- `/vision_double/target_features`；
- `/vision_double/zoom_position_steps`。

当前仓库内的 `apriltag_detector` 是单目检测节点，发布格式与 Stage 1 双目接口不同，不能直接代替 `/vision_double/target_features`。

### 4. 运行、使能与复位

启动 Simulink 模型后，控制器默认不使能。发送使能：

```bash
ros2 topic pub --once \
  /simulink/controller_enable \
  std_msgs/msg/Bool \
  "{data: true}"
```

需要清除 Core 内部状态时发送一次复位：

```bash
ros2 topic pub --once \
  /simulink/reset \
  std_msgs/msg/Bool \
  "{data: true}"
```

`reset` 在包装模型中被转换为单采样周期脉冲。停止控制时发送：

```bash
ros2 topic pub --once \
  /simulink/controller_enable \
  std_msgs/msg/Bool \
  "{data: false}"
```

### 5. 检查输出

```bash
ros2 topic echo /simulink/controller_status
ros2 topic echo /simulink/target_joints_velocities
ros2 topic echo /simulink/zoom_step_rate_cmd
```

也可以检查发布频率和通信双方：

```bash
ros2 topic hz /simulink/controller_status
ros2 topic info -v /simulink/target_joints_velocities
```

当前真实标定许可为 `false`，因此即使 ROS 2 通信正常并且控制器已经使能，安全模块仍会将关节速度锁为零。这是预期行为。

### 6. 启动 Python 最终安全节点

只进行通信检查时保持默认 `zero` 模式：

```bash
PARAMS_FILE="$(ros2 pkg prefix velocity_servo_tag)/share/velocity_servo_tag/config/velocity_servo_tag.yaml"

ros2 run velocity_servo_tag velocity_command_node \
  --ros-args \
  --params-file "$PARAMS_FILE" \
  -p mode:=zero
```

`topic` 模式会读取 `/simulink/target_joints_velocities` 并向底层控制器 Topic 转发。只有在标定、安全检查和硬件准备完成后才能使用：

```bash
ros2 run velocity_servo_tag velocity_command_node \
  --ros-args \
  --params-file "$PARAMS_FILE" \
  -p mode:=topic
```

## ROS 2 接口

### 包装模型接口

| 方向 | Topic | 消息类型 | 有效数据 |
|---|---|---|---|
| 订阅 | `/franka/joint_states` | `sensor_msgs/msg/JointState` | `position` 前 7 项 |
| 订阅 | `/vision_double/target_features` | `std_msgs/msg/Float64MultiArray` | 8 维视觉特征 |
| 订阅 | `/vision_double/zoom_position_steps` | `std_msgs/msg/Float64MultiArray` | 2 维累计变焦步数 |
| 订阅 | `/simulink/controller_enable` | `std_msgs/msg/Bool` | 控制器电平使能 |
| 订阅 | `/simulink/reset` | `std_msgs/msg/Bool` | 单次复位请求 |
| 发布 | `/simulink/target_joints_velocities` | `std_msgs/msg/Float64MultiArray` | 7 维关节速度，`rad/s` |
| 发布 | `/simulink/zoom_step_rate_cmd` | `std_msgs/msg/Float64MultiArray` | 2 维变焦步速，`steps/s` |
| 发布 | `/simulink/controller_status` | `std_msgs/msg/Float64MultiArray` | 12 维诊断状态 |

视觉特征格式：

```text
[validL, validR, uL, vL, uR, vR, scaleL, scaleR]
```

变焦累计位置格式：

```text
[leftSteps, rightSteps]
```

目标关节速度格式：

```text
[dq1, dq2, dq3, dq4, dq5, dq6, dq7]
```

包装模型会检查关节、视觉和变焦数组的最小长度 `7/8/2`；长度不满足时禁止控制。

## Simulink Core 接口

Core 文件：`simulink/core/stereo_ibvs_core.slx`

所有顶层端口编译类型均为 `double`，Stage 1 采样周期为 `cfg.Ts = 1/120 s`。

### 输入

| 端口 | 尺寸 | 内容 |
|---|---:|---|
| `qRaw` | `7×1` | `fr3_joint1` 到 `fr3_joint7` 的位置 |
| `visionFeatureRaw` | `8×1` | 左右目标有效性、中心像素和尺度 |
| `zoomPositionStepsRaw` | `2×1` | 左右镜头累计步数 |
| `controllerEnableRaw` | 标量 | 控制器使能 |
| `resetRaw` | 标量 | 内部状态复位 |

### 输出

| 端口 | 尺寸 | 内容 |
|---|---:|---|
| `jointVelocityCmd` | `7×1` | 目标关节速度，`rad/s` |
| `zoomStepRateCmd` | `2×1` | 目标变焦步速，Stage 1 为零 |
| `controllerStatus` | `12×1` | 有效性、使能和条件数诊断 |

`controllerStatus` 顺序：

| 索引 | 字段 |
|---:|---|
| 1 | `inputDataValid` |
| 2 | `cameraModelValid` |
| 3 | `kinematicsValid` |
| 4 | `validLeft` |
| 5 | `validRight` |
| 6 | `depthMeasurementValid` |
| 7 | `centerTaskValid` |
| 8 | `safetyValid` |
| 9 | `ekfValid` |
| 10 | `zoomControllerValid` |
| 11 | `controllerEnable` |
| 12 | `jConditionMetric` |

## 配置文件

| 文件 | 作用 |
|---|---|
| `simulink/config/stereo_ibvs_config.m` | 采样时间、FR3 模型、相机参数、控制增益、阻尼、软限位和标定许可 |
| `config/velocity_servo_tag.yaml` | Python 检测节点和最终关节速度命令节点参数 |
| `config/controllers.yaml` | `ros2_control` 关节速度控制器配置 |
| `config/urdf/fr3.urdf` | Simulink 配置脚本当前加载的 FR3 URDF |

Simulink `.slx` 不读取 YAML。ROS 2 Topic 配置当前保存在包装模型的 ROS 2 Block 中；YAML 中的 `simulink_ros2` 段仅用于记录接口。

当前关键安全参数：

```text
cfg.cameraFps = 120
cfg.qDotAlgorithmMax = 0.03 rad/s，每个关节
cfg.cameraMountCalibrated = false
cfg.cameraIntrinsicsCalibrated = false
cfg.stereoCalibrationValid = false
cfg.zoomCalibrationValid = false
```

真实运动许可：

```text
cfg.stage1CalibrationReady =
    cfg.cameraMountCalibrated &&
    cfg.cameraIntrinsicsCalibrated
```

禁止为了得到非零输出而直接把标定标志改成 `true`。必须先写入真实标定结果并完成方向、单位和限幅验证。

## 安全机制

Simulink：

- 检查输入中的 NaN 和 Inf；
- 检查 ROS 数组有效长度；
- 算法层关节速度限幅；
- 关节软限位；
- 标定许可和控制器使能门控；
- 输入或内部状态无效时输出零速度。

Python `velocity_command_node`：

- 检查七维命令长度；
- 拒绝 NaN 和 Inf；
- 按 FR3 官方关节速度上限同比例缩放；
- 限制关节加速度；
- 输入超时后平滑减速到零；
- 退出前平滑减速到零；
- 默认使用 `zero` 模式。

## 已验证范围

- Core 和 ROS 2 包装模型通过 MATLAB R2025b Update Diagram；
- FR3 模型已经移除两个手指活动关节，Jacobian 尺寸为 `6×7`；
- 包装模型的 5 个订阅、3 个发布和消息维度已验证；
- `/simulink/reset` 单周期脉冲已验证；
- ROS 2 消息能够正常发送和接收；
- 7、2、12 维 `Float64MultiArray` 输出总线有效长度已验证。

尚未验证：

- 与真实双目相机节点的联合运行；
- 实际手眼、双目和变焦标定；
- ROS 输入断流保护；
- FR3 非零速度真机闭环；
- 主动变焦硬件闭环。

## 已知限制

- Simulink Subscribe 块在消息停止后会保持上一条数据，当前没有输入 freshness/watchdog；
- `JointState.position` 当前取前 7 项，尚未根据 `JointState.name` 重排；
- ROS 2 Block 当前 QoS 为 Reliable、Volatile、Keep last、Depth 10；
- Stage 1 使用固定逆深度，尚未启用双目深度闭环；
- Stage 1 的 EKF 和主动变焦控制仍是后续阶段接口；
- 当前单目 AprilTag 节点不能直接提供 Stage 1 所需的双目 8 维特征。

## 常见问题

### 控制器已经 enable，但关节速度始终为零

首先检查 `/simulink/controller_status`。当前标定标志默认为 `false`，因此零输出通常是安全门控的预期结果。

### 首次打开包装模型时找不到 Core

手动把 `simulink/core/` 和 `simulink/config/` 加入 MATLAB path，运行 `stereo_ibvs_config.m`，再执行 Update Diagram。

### ROS 2 Topic 存在但收不到数据

检查 `ROS_DOMAIN_ID`、DDS 网络、Topic 类型和 QoS 是否一致：

```bash
ros2 topic info -v /simulink/target_joints_velocities
ros2 topic type /simulink/target_joints_velocities
```

### `JointState` 长度正确但控制不符合预期

确认前 7 个 `position` 的顺序确实是 `fr3_joint1` 到 `fr3_joint7`。真机前建议在包装层实现按名称重排。

## 实现路线

- Stage 1：固定内参与固定逆深度的左相机中心 IBVS；
- Stage 2：双目逆深度滤波与深度任务；
- Stage 3：关节限位零空间任务；
- Stage 4：世界坐标系 9 状态目标 EKF 和速度前馈；
- Stage 5：主动双目变焦及步数—焦距标定；
- Stage 6：预测、时延补偿、目标丢失处理、超时保护和真机验证。

## License

本项目使用 Apache-2.0 License。
