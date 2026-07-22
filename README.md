# visual_servo_tag

面向 Franka Research 3（FR3）的双目主动变焦视觉伺服项目。项目使用 Simulink 完成视觉控制、机器人运动学和算法层安全保护，并直接输出七维关节速度；Python ROS 2 节点负责最终限速、加速度限制、通信超时保护和底层控制器转发。

> 当前版本处于 Stage 1。Core 模型、ROS 2 包装模型和双目视觉节点已经完成，ROS 2 消息收发已经验证；尚未完成双 USB 相机实机联合实验、真实标定和 FR3 真机闭环验证。

## 项目目标

- 使用双目视觉特征控制 FR3 跟踪目标；
- 使用同一个 Simulink Core 支持 ROS 2 部署和后续离线仿真；
- 在后续阶段加入双目逆深度、目标 EKF、关节限位零空间任务和主动变焦；
- 将算法层安全与硬件发送层安全分离，便于验证和部署。

## 当前状态

| 模块 | 状态 | 说明 |
|---|---|---|
| `stereo_ibvs_core.slx` | 已完成 | Stage 1 固定内参、固定逆深度、左相机中心 IBVS |
| `stereo_ibvs_ros2_stage1.slx` | 已完成 | 4 个 ROS 2 输入、3 个 ROS 2 输出，含输入新鲜度和自动使能保护 |
| `vision_double_node` | 已完成 | 双 USB 相机独立采集、AprilTag 检测、8 维特征和 Zoom 零占位发布 |
| ROS 2 通信测试 | 已完成 | 已验证包装模型能够订阅和发布消息 |
| 双目相机联合实验 | 未完成 | 节点代码和纯算法测试已完成，尚未连接两台真实相机验证 |
| 真实相机与手眼标定 | 未完成 | 当前配置仍为安全占位值 |
| FR3 真机闭环 | 未完成 | 标定和 JointState 顺序确认完成前禁止非零真机运动 |
| Stage 2～6 | 规划中 | 逆深度、零空间、EKF、主动变焦和工程强化 |

本 README 主要介绍 Core 和 ROS 2 Stage 1，暂不展开离线 sim 使用方法。

## 系统架构

```text
vision_double_node
  ├─ /vision_double/target_features
  └─ /vision_double/zoom_position_steps
                                  \
FR3 JointState --------------------→ stereo_ibvs_ros2_stage1.slx
人工复位 ---------------------------/              │
                         Joint/Vision freshness     │
                         左目标三帧丢失保护          │
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
- ROS 2 包装模型：ROS 消息收发、数组长度检查、Joint/Vision freshness、左目标三帧丢失保护、自动内部使能和复位脉冲；
- `velocity_command_node`：最终速度限制、加速度限制、watchdog 和底层命令发布；
- `vision_double_node`：左右相机独立采集和 AprilTag 检测，发布 8 维双目特征，并在 Stage 1 发布固定 Zoom 位置 `[0,0]`；
- 变焦驱动：Stage 1 不实现电机控制，待后续阶段加入真实反馈。

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
│   ├── vision_double.launch.py
│   └── velocity_servo_tag.launch.py
├── simulink/
│   ├── build/sim/build_stereo_ibvs_sim_stage1.m
│   ├── config/stereo_ibvs_config.m
│   ├── core/stereo_ibvs_core.slx
│   ├── ros2/stereo_ibvs_ros2_stage1.slx
│   ├── sim/stereo_ibvs_sim_stage1.slx
│   └── sim/plot_stereo_ibvs_sim_stage1_results.m
├── velocity_servo_tag/
│   ├── safety.py
│   ├── velocity_command_node.py
│   └── vision/
│       ├── apriltag_detector.py
│       ├── camera.py
│       ├── stereo_features.py
│       └── vision_double_node.py
├── package.xml
├── setup.py
└── README.md
```

`slprj/`、`*.slxc`、`*.autosave`、`*.asv` 和 ROS 总线转换缓存均已加入 `.gitignore`，不属于项目源码。

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

### 3. 启动双目视觉节点

先在 `config/velocity_servo_tag.yaml` 中设置左右 USB 相机编号：

```yaml
vision_double_node:
  ros__parameters:
    left_camera_index: 0
    right_camera_index: 1
    camera_width: 640
    camera_height: 480
    camera_fps: 60.0
    publish_rate_hz: 60.0
```

然后启动：

```bash
ros2 launch velocity_servo_tag vision_double.launch.py
```

检查输出：

```bash
ros2 topic echo /vision_double/target_features
ros2 topic echo /vision_double/zoom_position_steps
ros2 topic hz /vision_double/target_features
```

节点会持续提供：

- `/vision_double/target_features`；
- `/vision_double/zoom_position_steps`。

`zoom_position_steps` 在 Stage 1 固定发布 `[0.0, 0.0]`，只用于满足 Simulink 的 2 维输入检查，不代表真实电机位置。`/franka/joint_states` 仍由 FR3 状态广播器或独立测试发布器提供。

### 4. 自动使能与复位

包装模型不再订阅 `/simulink/controller_enable`，无需人工发送使能消息。内部使能由安全监督器自动控制，必须同时满足：

- `/franka/joint_states` 在 `0.10 s` 内有新消息；
- `/vision_double/target_features` 在 `0.10 s` 内有新消息；
- 关节、视觉和 Zoom 数组长度合法；
- 左相机 `validL` 连续有效 3 帧。

Stage 1 的中心 IBVS 只使用左相机，因此目标丢失也只以 `validL` 判断；右相机 `validR` 当前只用于数据记录和诊断，不触发停车。运行中允许左目标短暂丢失，连续第 3 帧无效时自动关闭内部使能。有效数据恢复后，需要连续 3 帧 `validL=true` 才重新使能。

需要清除 Core 内部状态时发送一次复位：

```bash
ros2 topic pub --once \
  /simulink/reset \
  std_msgs/msg/Bool \
  "{data: true}"
```

`reset` 在包装模型中被转换为单采样周期脉冲，同时清除安全监督器保存的上一帧视觉特征、消息年龄和恢复计数。安全监督器从允许运动切换到故障停止时，也会自动产生一次内部复位脉冲。

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

当前真实标定许可为 `false`，因此即使 ROS 2 通信正常并且自动内部使能已经满足，Core 仍会将关节速度锁为零。这是预期行为。

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

### 7. 使用完整 ROS 2 启动入口

默认只启动双目视觉，不连接真实 FR3：

```bash
ros2 launch velocity_servo_tag full_system.launch.py \
  start_vision:=true \
  start_hardware:=false \
  command_mode:=zero
```

连接 FR3 但只允许底层零速度：

```bash
ros2 launch velocity_servo_tag full_system.launch.py \
  robot_ip:=172.16.0.2 \
  start_vision:=true \
  start_hardware:=true \
  command_mode:=zero \
  max_velocity_scale:=0.10
```

`full_system.launch.py` 不会启动 MATLAB/Simulink。真实标定和 JointState 顺序确认完成前，不允许使用 `command_mode:=topic` 进行非零真机运动。

## ROS 2 接口

### 包装模型接口

| 方向 | Topic | 消息类型 | 有效数据 |
|---|---|---|---|
| 订阅 | `/franka/joint_states` | `sensor_msgs/msg/JointState` | `position` 前 7 项 |
| 订阅 | `/vision_double/target_features` | `std_msgs/msg/Float64MultiArray` | 8 维视觉特征 |
| 订阅 | `/vision_double/zoom_position_steps` | `std_msgs/msg/Float64MultiArray` | 2 维累计变焦步数 |
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

包装模型会检查关节、视觉和变焦数组的最小长度 `7/8/2`；还会使用 Joint/Vision Subscribe 的 `IsNew` 监控消息新鲜度。内部使能由包装模型自动生成，不再需要外部使能 Topic。

### `vision_double_node` 输出

| Topic | 发布频率 | 数据 |
|---|---:|---|
| `/vision_double/target_features` | 目标 60 Hz | `[validL, validR, uL, vL, uR, vR, scaleL, scaleR]` |
| `/vision_double/zoom_position_steps` | 目标 60 Hz | Stage 1 固定 `[0.0, 0.0]` |

左右相机分别使用独立采集线程。检测结果超时后对应 `valid` 置零；左右有效结果时间差超过 `max_pair_skew_sec` 时，较旧一侧置零。`scale` 定义为 AprilTag 四角像素面积的平方根。

## Simulink Core 接口

Core 文件：`simulink/core/stereo_ibvs_core.slx`

所有顶层端口编译类型均为 `double`，Stage 1 采样周期为 `cfg.Ts = 1/120 s`。

### 输入

| 端口 | 尺寸 | 内容 |
|---|---:|---|
| `qRaw` | `7×1` | `fr3_joint1` 到 `fr3_joint7` 的位置 |
| `visionFeatureRaw` | `8×1` | 左右目标有效性、中心像素和尺度 |
| `zoomPositionStepsRaw` | `2×1` | 左右镜头累计步数 |
| `controllerEnableRaw` | 标量 | ROS 2 包装层安全监督器生成的自动内部使能 |
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
| `config/velocity_servo_tag.yaml` | 单目/双目检测节点和最终关节速度命令节点参数 |
| `config/controllers.yaml` | `ros2_control` 关节速度控制器配置 |
| `config/urdf/fr3.urdf` | Simulink 配置脚本当前加载的 FR3 URDF |

Simulink `.slx` 不读取 YAML。ROS 2 Topic 配置当前保存在包装模型的 ROS 2 Block 中；YAML 中的 `simulink_ros2` 段仅用于记录接口。

当前关键安全参数：

```text
cfg.cameraFps = 120
cfg.jointStateTimeoutSec = 0.10 s
cfg.visionTimeoutSec = 0.10 s
cfg.targetLossFrameLimit = 3
cfg.targetRecoveryFrameCount = 3
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
- 使用 Subscribe `IsNew` 检查 JointState 和视觉 Topic 是否在 `0.10 s` 内更新；
- Stage 1 仅以左相机 `validL` 判断目标状态；连续丢失 3 帧后关闭内部使能；
- 短暂丢失期间保持最后一次合法视觉特征；恢复时连续有效 3 帧才重新使能；
- 安全状态从使能切换到停止时自动复位 Core；
- 算法层关节速度限幅；
- 关节软限位；
- 标定许可和自动内部使能门控；
- 输入或内部状态无效时输出零速度。

Python `velocity_command_node`：

- 检查七维命令长度；
- 拒绝 NaN 和 Inf；
- 按 FR3 官方关节速度上限同比例缩放；
- 限制关节加速度；
- Simulink 输出零目标或输入超时后平滑减速到零；
- 退出前平滑减速到零；
- 默认使用 `zero` 模式。

## 已验证范围

- Core 和 ROS 2 包装模型通过 MATLAB R2025b Update Diagram；
- FR3 模型已经移除两个手指活动关节，Jacobian 尺寸为 `6×7`；
- 包装模型的 4 个订阅、3 个发布和消息维度已验证；
- ROS 2 包装模型的安全监督器已经通过 MATLAB R2025b Update Diagram；
- `/simulink/reset` 单周期脉冲已验证；
- ROS 2 消息能够正常发送和接收；
- 7、2、12 维 `Float64MultiArray` 输出总线有效长度已验证。
- 双目尺度、超时、双目时间差和 Zoom 零占位通过 6 项纯算法单元测试。

尚未验证：

- `vision_double_node` 与两台真实 USB 相机的联合运行和实际 60 Hz 性能；
- 实际手眼、双目和变焦标定；
- 使用真实 ROS 2 发布器验证 Joint/Vision 断流和三帧丢失恢复时序；
- FR3 非零速度真机闭环；
- 主动变焦硬件闭环。

## 已知限制

- Stage 1 的目标丢失只监控左相机 `validL`，右相机暂不触发停车；
- `JointState.position` 当前取前 7 项，尚未根据 `JointState.name` 重排；
- ROS 2 Block 当前 QoS 为 Reliable、Volatile、Keep last、Depth 10；
- Stage 1 使用固定逆深度，尚未启用双目深度闭环；
- Stage 1 的 EKF 和主动变焦控制仍是后续阶段接口；
- 两台普通 USB 相机没有硬件同步，当前使用软件单调时钟检查左右检测时间差；
- 实际检测帧率取决于相机、USB 带宽和 CPU，配置的 60 Hz 是目标值；
- Zoom 位置当前始终为 `[0.0, 0.0]`，没有电机控制和真实位置反馈。

## 常见问题

### 自动使能已经满足，但关节速度始终为零

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
- Stage 6：预测、时延补偿、更完整的故障恢复和真机验证。

## License

本项目使用 Apache-2.0 License。
