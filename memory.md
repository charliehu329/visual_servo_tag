# 项目目标

说明：记录整个项目的目标

构建一套工程化的 Franka FR3 双目主动变焦视觉伺服系统：

- 双目相机检测目标并输出图像特征；
- Simulink 完成特征处理、机器人运动学、视觉控制、状态估计和算法层安全保护；
- Simulink 直接输出 `7×1` 关节速度 `qDot`；
- Python ROS 2 节点负责最终速度/加速度限制、watchdog、消息检查和底层转发；
- 同一套 `stereo_ibvs_core.slx` 同时服务于离线仿真、ROS 2 联调和真机部署；
- 后续逐步加入双目逆深度、关节限位零空间任务、目标 EKF、主动变焦和高速跟踪。

安全分层：Simulink 负责算法层保护和失效输出零速度，Python 负责最终硬限速、断流保护和底层发送。

# 文件架构

说明：把当前文件夹的架构记录，项目树记录在下面

当前主要文件：

```text
visual_servo_tag/
├── AGENTS.md
├── Memory.md
├── README.md
├── config/
│   ├── controllers.yaml
│   ├── velocity_servo_tag.yaml
│   └── urdf/
│       └── fr3.urdf
├── docs/
│   └── SIMULINK_VALIDATION_GUIDE.md
├── launch/
│   ├── fr3_hardware.launch.py
│   ├── full_system.launch.py
│   └── velocity_servo_tag.launch.py
├── simulink/
│   ├── config/
│   │   ├── stereo_ibvs_config.m
│   │   └── urdf/fr3.urdf
│   ├── core/
│   │   ├── stereo_ibvs_core.slx
│   │   └── stereo_ibvs_core.slxc
│   ├── ros2/
│   │   └── stereo_ibvs_ros2_stage1.slx
│   └── sim/
│       └── build_stereo_ibvs_sim_stage1.m
├── velocity_servo_tag/
│   ├── robot_kinematics.py
│   ├── safety.py
│   ├── velocity_command_node.py
│   ├── velocity_mapper_node.py
│   └── vision/
│       ├── apriltag_detector.py
│       └── camera.py
├── package.xml
├── setup.cfg
└── setup.py
```

架构约定：

- `simulink/core/`：不直接收发 ROS 2 消息的统一核心算法；
- `simulink/ros2/`：ROS 2 包装模型，通过 Model 块引用同一个 core；
- `simulink/sim/`：已有 Stage 1 离线模型生成脚本，目标 `.slx` 尚未生成和验证；
- 当前不需要单独维护 `simulink/build/`；
- `*.slxc`、`slprj/` 和 `*.asv` 是缓存或自动保存文件，不是核心源文件。

# 已完成的文件状态

说明：每次做完都要修改这里，把当前做了什么，什么已经做完了，下一步要做什么都记录明白

## `simulink/core/stereo_ibvs_core.slx`

Stage 1 核心模型已经完成，并通过 MATLAB R2025b Update Diagram。当前使用固定相机内参和固定逆深度，执行左相机图像中心 IBVS，输出 `7×1` 关节速度；EKF 和主动变焦暂为后续阶段接口。顶层端口编译类型均为 `double`，采样周期为 `cfg.Ts = 1/60 s`。

常用顶层接口：

```text
输入：
qRaw                   7×1
visionFeatureRaw       8×1  [validL; validR; uL; vL; uR; vR; scaleL; scaleR]
zoomPositionStepsRaw   2×1
controllerEnableRaw    标量
resetRaw               标量

输出：
jointVelocityCmd       7×1
zoomStepRateCmd        2×1
controllerStatus       12×1
```

`controllerStatus` 顺序：

```text
[inputDataValid; cameraModelValid; kinematicsValid;
 validLeft; validRight; depthMeasurementValid; centerTaskValid;
 safetyValid; ekfValid; zoomControllerValid;
 controllerEnable; jConditionMetric]
```

核心子系统：

```text
01_Input_Validity
02_Camera_Model
03_Feature_Processing
04_FR3_Camera_Kinematics
05_Target_EKF
06_Inverse_Depth_Filter
07_Arm_Priority_Controller
08_Zoom_Controller
09_Safety_Supervisor
10_Diagnostics
```

## `simulink/ros2/stereo_ibvs_ros2_stage1.slx`

Stage 1 ROS 2 包装模型已经完成，并通过多次短时仿真冒烟测试。Model 块接口与 core 的 `5` 个输入、`3` 个输出一致。

| 方向 | Topic | 消息类型 / 有效维度 |
|---|---|---|
| 订阅 | `/franka/joint_states` | `sensor_msgs/JointState`，当前取 `position` 前 7 项 |
| 订阅 | `/vision_double/target_features` | `std_msgs/Float64MultiArray`，8 维 |
| 订阅 | `/vision_double/zoom_position_steps` | `std_msgs/Float64MultiArray`，2 维 |
| 订阅 | `/simulink/controller_enable` | `std_msgs/Bool` |
| 订阅 | `/simulink/reset` | `std_msgs/Bool`，新消息且为 true 时产生单采样复位脉冲 |
| 发布 | `/velocity_mapper_node/target_joints_velocities` | `std_msgs/Float64MultiArray`，7 维 |
| 发布 | `/simulink/zoom_step_rate_cmd` | `std_msgs/Float64MultiArray`，2 维 |
| 发布 | `/simulink/controller_status` | `std_msgs/Float64MultiArray`，12 维 |

已实现关节/视觉/变焦数组最小长度 `7/8/2` 检查；长度不满足时禁止控制。三个输出按 ROS 总线的 128 元素缓存生成，并设置正确的有效长度。

## `simulink/config/stereo_ibvs_config.m`

已移除机器人模型中的 `fr3_leftfinger` 和 `fr3_rightfinger` 分支，Jacobian 已由错误的 `6×9` 修正为 `6×7`，并增加 7 自由度检查。

## Python ROS 2 节点

现有视觉检测、速度映射、安全检查和速度发送节点保留。本次 Stage 1 Simulink 包装工作没有修改 Python 文件。

## 当前限制

- 真实标定许可仍为 `false`，所以安全模块会把实际关节速度锁为零；
- ROS 2 输入尚无新鲜度/超时 watchdog，断流后 Subscribe 块会保留上一条消息；
- `JointState.position` 尚未根据 `JointState.name` 重排，真机前必须核对关节顺序；
- ROS 2 块当前 QoS 为 Reliable、Volatile、Keep last、Depth 10，联调时需确认发布端兼容；
- 使用 Simulink 包装模型时必须关闭旧 `velocity_mapper_node`，避免两个节点同时发布同一速度 Topic；
- 全新 MATLAB 会话首次加载时可能先提示找不到 core，模型 PostLoad 加入路径后执行 Ctrl+D 可以通过；
- macOS 上结束短仿真时可能显示 DDS thread affinity 信息，但已测试仿真能正常成功退出。

## 下一步

1. 用 ROS 2 测试发布器完成桌面话题联调，核对三个输出 Topic；
2. 采集真实 `/franka/joint_states`，确认前 7 项顺序或增加按名称重排；
3. 启动 Simulink 包装模型时设置 `start_mapper:=false`，并让 `full_system.launch.py` 正确传递该参数；
4. 创建并验证 `simulink/sim/stereo_ibvs_sim_stage1.slx` 离线仿真包装；
5. 真机前加入 ROS 输入 freshness/watchdog，再对接 Python 最终安全转发链路。

# 工作日志

说明：每次做完工作就记录在下面，记录时间，修改的文件，新增的功能，以及备注（如果需要）

## 2026-07-21：完成 Stage 1 ROS 2 包装模型

修改文件：

```text
simulink/ros2/stereo_ibvs_ros2_stage1.slx
simulink/config/stereo_ibvs_config.m
config/velocity_servo_tag.yaml
Memory.md
```

完成内容：创建 ROS 2 包装模型；接入 5 个订阅和 3 个发布；加入输入长度门控、单周期复位和 ROS 数组有效长度处理；修正 FR3 模型为 7 自由度；通过 MATLAB R2025b 编译和短时 ROS 2 仿真测试。

备注：Python 节点未修改；真机联调前仍需完成标定许可、JointState 顺序确认和输入超时保护。

## 2026-07-21：重构项目 Memory

修改文件：`Memory.md`

完成内容：按固定六段格式整理项目记忆，保留常用接口、当前状态、后续计划和配置分工，删除重复的子系统说明。

# 实现计划

说明：如果有分步实行的stage1，2，3，可以记录

- Stage 1：固定内参与固定深度的左相机中心 IBVS。core 和 ROS 2 包装已完成；离线仿真、桌面话题联调和硬件接入未完成。
- Stage 2：加入双目逆深度滤波与深度闭环任务。
- Stage 3：加入关节限位零空间任务和任务优先级控制。
- Stage 4：加入世界坐标系 9 状态目标 EKF `[p; v; a]` 和目标速度前馈。
- Stage 5：加入双目主动变焦，将累计步数通过标定曲线转换为焦距和相机内参，并处理回零、限位和编码器反馈。
- Stage 6：加入预测、时延补偿、目标丢失处理、超时保护、平滑/变化率限制、奇异性监控、恢复流程和真机验证。

# 配置文件

说明：把配置文件，.yaml，.m配置文件简单记录

| 文件 | 作用 |
|---|---|
| `config/velocity_servo_tag.yaml` | Python ROS 2 节点的 Topic、安全转发、watchdog、发布频率和最终速度限制参数 |
| `config/controllers.yaml` | `ros2_control` 控制器配置 |
| `simulink/config/stereo_ibvs_config.m` | Simulink 的采样时间、FR3 模型、相机参数、控制增益、阻尼、软限位、安全阈值和阶段参数 |
| `config/urdf/fr3.urdf` | `stereo_ibvs_config.m` 当前实际加载的 FR3 模型 |
| `simulink/config/urdf/fr3.urdf` | Simulink 目录内的 URDF 副本，目前不是配置脚本实际加载路径 |

配置关系：

- Simulink 的主要工作区参数由 `stereo_ibvs_config.m` 设置；模型块内部仍可能包含自身参数；
- Python 节点读取 `velocity_servo_tag.yaml`，Simulink `.slx` 当前不会读取这个 YAML；
- YAML 中现有 `simulink_ros2` 段只是接口记录，不会自动创建或修改 Simulink 接口，后续可决定是否删除；
- 当前 `cameraMountCalibrated`、`cameraIntrinsicsCalibrated`、`stereoCalibrationValid` 和 `zoomCalibrationValid` 均为 `false`；
- 后续手眼、双目和变焦标定结果计划独立保存在 `simulink/calibration/`。
