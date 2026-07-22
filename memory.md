# 项目目标

说明：记录整个项目的目标

构建一套工程化的 Franka FR3 双目主动变焦视觉伺服系统：

- 双目相机检测目标并输出图像特征；
- Simulink 完成特征处理、机器人运动学、视觉控制、状态估计和算法层安全保护；
- Simulink 直接输出 `7×1` 关节速度 `qDot`；
- Python ROS 2 节点负责最终速度/加速度限制、watchdog、消息检查和底层转发；
- 同一套 `stereo_ibvs_core.slx` 同时服务于离线仿真、ROS 2 联调和真机部署；
- 后续逐步加入双目逆深度、关节限位零空间任务、目标 EKF、主动变焦和高速跟踪。

当前目标环境：Ubuntu 24.04、ROS 2 Jazzy、MATLAB R2025b。

安全分层：Simulink 负责算法层保护和失效输出零速度，Python 负责最终硬限速、断流保护和底层发送。

# 文件架构

说明：把当前文件夹的架构记录，项目树记录在下面

当前主要文件：

```text
visual_servo_tag/
├── AGENTS.md
├── memory.md
├── README.md
├── config/
│   ├── controllers.yaml
│   ├── velocity_servo_tag.yaml
│   └── urdf/
│       └── fr3.urdf
├── launch/
│   ├── fr3_hardware.launch.py
│   ├── full_system.launch.py
│   ├── vision_double.launch.py
│   └── velocity_servo_tag.launch.py
├── simulink/
│   ├── build/
│   │   └── sim/build_stereo_ibvs_sim_stage1.m
│   ├── config/
│   │   └── stereo_ibvs_config.m
│   ├── core/
│   │   └── stereo_ibvs_core.slx
│   ├── ros2/
│   │   └── stereo_ibvs_ros2_stage1.slx
│   └── sim/
│       ├── stereo_ibvs_sim_stage1.slx
│       └── plot_stereo_ibvs_sim_stage1_results.m
├── velocity_servo_tag/
│   ├── robot_kinematics.py
│   ├── safety.py
│   ├── velocity_command_node.py
│   └── vision/
│       ├── apriltag_detector.py
│       ├── camera.py
│       ├── stereo_features.py
│       └── vision_double_node.py
├── test/
│   └── test_stereo_features.py
├── package.xml
├── setup.cfg
└── setup.py
```

架构约定：

- `simulink/core/`：不直接收发 ROS 2 消息的统一核心算法；
- `simulink/ros2/`：ROS 2 包装模型，通过 Model 块引用同一个 core；
- `simulink/build/`：可重复生成模型的 MATLAB 脚本；
- `simulink/sim/`：Stage 1 离线仿真模型和结果绘图脚本；
- `*.slxc`、`slprj/`、`*.autosave`、`*.asv` 和 ROS 总线转换文件是自动生成缓存，已经加入 `.gitignore`。

# 已完成的文件状态

说明：每次做完都要修改这里，把当前做了什么，什么已经做完了，下一步要做什么都记录明白

## `simulink/core/stereo_ibvs_core.slx`

Stage 1 核心模型已经完成，并通过 MATLAB R2025b Update Diagram。当前使用固定相机内参和固定逆深度，执行左相机图像中心 IBVS，输出 `7×1` 关节速度；EKF 和主动变焦暂为后续阶段接口。顶层端口编译类型均为 `double`，采样周期为 `cfg.Ts = 1/120 s`。

Core 的 `InitFcn` 仅在基础工作区缺少完整 `cfg` 时加载正式配置；已有完整 `cfg` 会直接使用。因此 ROS 2 模型仍加载真实标定许可，离线仿真可临时设置 `stage1CalibrationReady=true`，两者不再互相覆盖。

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

Stage 1 ROS 2 包装模型已经完成。Model 块接口与 core 的 `5` 个输入、`3` 个输出一致，包装模型使用 `4` 个 ROS 2 订阅和 `3` 个发布；消息收发已经验证，尚未与真实双目相机进行联合实验。

| 方向 | Topic | 消息类型 / 有效维度 |
|---|---|---|
| 订阅 | `/franka/joint_states` | `sensor_msgs/JointState`，当前取 `position` 前 7 项 |
| 订阅 | `/vision_double/target_features` | `std_msgs/Float64MultiArray`，8 维 |
| 订阅 | `/vision_double/zoom_position_steps` | `std_msgs/Float64MultiArray`，2 维 |
| 订阅 | `/simulink/reset` | `std_msgs/Bool`，新消息且为 true 时产生单采样复位脉冲 |
| 发布 | `/simulink/target_joints_velocities` | `std_msgs/Float64MultiArray`，7 维 |
| 发布 | `/simulink/zoom_step_rate_cmd` | `std_msgs/Float64MultiArray`，2 维 |
| 发布 | `/simulink/controller_status` | `std_msgs/Float64MultiArray`，12 维 |

已实现关节/视觉/变焦数组最小长度 `7/8/2` 检查；长度不满足时禁止控制。三个输出按 ROS 总线的 128 元素缓存生成，并设置正确的有效长度。

ROS 输入安全监督已经完成：

- 使用 JointState 和 Vision Subscribe 的 `IsNew` 监控新鲜度，超时均为 `0.10 s`；
- Stage 1 仅以左相机 `validL` 判断目标是否丢失，右相机不触发停车；
- 运行中连续第 3 帧左目标无效时关闭内部使能，短暂丢失期间保持最后一次合法视觉特征；
- 初始启动和故障恢复均要求连续 3 帧左目标有效；
- 从使能切换到停止时自动产生一次 Core 复位脉冲；人工 `/simulink/reset` 还会清除监督器保存的视觉特征和计数；
- `/simulink/controller_enable` 已删除，内部使能完全由包装模型自动生成；
- Simulink 只负责切换到零目标，最终平滑减速由 Python `velocity_command_node` 的 `0.20 rad/s²` 加速度限制完成。

MATLAB R2025b Update Diagram 和临时动态测试均通过；动态测试覆盖三帧丢失、三帧恢复、Vision 超时、JointState 超时和人工复位。

## `simulink/config/stereo_ibvs_config.m`

已移除机器人模型中的 `fr3_leftfinger` 和 `fr3_rightfinger` 分支，Jacobian 已由错误的 `6×9` 修正为 `6×7`，并增加 7 自由度检查。

## `simulink/sim/stereo_ibvs_sim_stage1.slx`

Stage 1 离线仿真已由 `simulink/build/sim/build_stereo_ibvs_sim_stage1.m` 重新生成。模型根据自身 `.slx` 位置查找 `core/` 和 `config/`，不再保存本机绝对路径；离线标定许可只在当前 MATLAB 工作区临时开启，不修改正式配置文件。

验证结果：启动关闭、目标居中和目标丢失阶段的关节速度均为零；目标偏离阶段最大绝对关节速度为 `0.03 rad/s`。

## Python ROS 2 节点

当前保留单目/双目视觉检测、通用安全函数和 `velocity_command_node`。`velocity_mapper_node.py` 已删除，Stage 1 关节速度由 Simulink 直接计算。setup、launch、YAML 和 Python 默认 Topic 中的旧 Mapper 配置已经清理。

`full_system.launch.py` 当前组合 `vision_double_node` 和可选 FR3 硬件链路；默认 `start_hardware:=false`、`command_mode:=zero`。MATLAB/Simulink 仍需单独运行。

### `velocity_servo_tag/vision/vision_double_node.py`

Stage 1 双目视觉节点代码已经完成：

- 左右普通 USB 相机 ID、分辨率和帧率由 YAML 配置，默认 `640×480 @ 60 Hz`；
- 左右相机各使用独立采集线程和独立 AprilTag 检测器；
- 发布 `/vision_double/target_features`：`[validL; validR; uL; vL; uR; vR; scaleL; scaleR]`；
- `scale` 定义为 AprilTag 四角像素面积的平方根；
- 检测结果超时后对应侧 `valid` 置零；双目时间差超限时较旧一侧置零；
- 以目标 `60 Hz` 发布 `/vision_double/zoom_position_steps = [0;0]`，仅作为 Stage 1 占位，不控制电机；
- 已增加 `vision_double.launch.py`、YAML 参数、setup 入口和 6 项纯算法单元测试。

代码语法和纯算法测试已经通过；尚未连接两台真实 USB 相机验证实际帧率、设备 ID 和 USB 带宽。

## 当前限制

- 真实标定许可仍为 `false`，所以安全模块会把实际关节速度锁为零；
- `JointState.position` 尚未根据 `JointState.name` 重排，真机前必须核对关节顺序；
- Stage 1 目标丢失只监控左相机，右相机仅用于数据记录和诊断；
- ROS 2 块当前 QoS 为 Reliable、Volatile、Keep last、Depth 10，联调时需确认发布端兼容；
- 全新 MATLAB 会话首次加载时可能先提示找不到 core，模型 PostLoad 加入路径后执行 Ctrl+D 可以通过；
- ROS 2 通信已经验证，但新双目节点尚未与真实双相机和 Simulink 联合运行；
- 普通 USB 相机没有硬件同步，当前使用软件单调时钟检查双目检测时间差；
- Zoom 位置当前固定为 `[0;0]`，没有真实位置反馈；
- 当前自定义 C++ `JointVelocityExampleController` 没有命令 freshness watchdog，不能用关闭终端代替实体急停。

## 下一步

1. 按 README 的实验顺序，对全部代码、Launch、Topic、配置和安全链路进行一次实验前全面检查；
2. 在 Ubuntu 24.04 / ROS 2 Jazzy 上重新构建包，验证 Launch 参数和 `full_system.launch.py`；
3. 设置左右相机 ID，验证两台 USB 相机、实际检测帧率和输出 Topic；
4. 将 `vision_double_node` 与 Simulink 联合运行，并采集真实 `/franka/joint_states` 核对关节顺序；
5. 启动控制器后核对命令 Topic 订阅者和 `filter_coefficient=0.01`；
6. 完成真实标定后，对接 Python 最终安全转发链路并进行零速到低速真机验证。

# 工作日志

说明：每次做完工作后记录到分钟，每次改完代码后都要在Memory.md里记录本次工作的内容和时间，精确到分钟，格式为：（按逐个文件说）
1:修改什么文件
2:修改了什么内容（简要概括）
3:修改的原因，目的，作用
4:备注（可选写和不写）

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

## 2026-07-22：重写工程 README 并校正项目记录

修改文件：

```text
README.md
memory.md
```

完成内容：按照工程项目 README 结构重新记录项目目标、当前状态、系统架构、Ubuntu 24.04 / ROS 2 Jazzy 环境、构建方式、Stage 1 ROS 2 使用方法、接口、安全机制、验证范围和路线图；不包含未完成的 sim 使用方法。

备注：同步将实际关节速度输出 Topic 修正为 `/simulink/target_joints_velocities`，采样周期修正为 `1/120 s`，并记录 ROS 通信已经验证、双目相机联合实验尚未完成。Mapper 遗留配置将在下一步单独清理。

## 2026-07-22：完成 Stage 1 双目视觉节点和 Zoom 零占位

修改文件：

```text
velocity_servo_tag/vision/stereo_features.py
velocity_servo_tag/vision/vision_double_node.py
launch/vision_double.launch.py
config/velocity_servo_tag.yaml
setup.py
test/test_stereo_features.py
README.md
memory.md
```

完成内容：实现左右 USB 相机独立采集和 AprilTag 检测，发布 8 维双目特征；加入检测新鲜度和双目时间差检查；以 60 Hz 目标频率发布 `[0;0]` Zoom 位置占位；增加 YAML、launch、安装入口、README 使用说明和纯算法测试。

验证：新增 Python 文件和 launch 通过语法检查；尺度、有效双目、检测超时、双目时间差和 Zoom 占位共 6 项单元测试通过。

备注：当前没有 Zoom 电机控制；真实双 USB 相机、实际 60 Hz 性能和 Simulink 联合运行仍待 Ubuntu 24.04 / ROS 2 Jazzy 环境验证。

## 2026-07-22：清理 Mapper 遗留配置并重构完整启动入口

修改文件：

```text
setup.py
package.xml
config/velocity_servo_tag.yaml
launch/velocity_servo_tag.launch.py
launch/full_system.launch.py
velocity_servo_tag/velocity_command_node.py
README.md
memory.md
```

完成内容：删除已不存在的 Mapper executable 和整段 YAML 参数；将 `velocity_command_node` 默认输入统一为 `/simulink/target_joints_velocities`；将 YAML 中 Simulink 接口记录同步为 `1/120 s` 和实际关节速度 Topic；上层 launch 改为启动 `vision_double_node`；`full_system.launch.py` 改为组合双目视觉和可选 FR3 硬件链路。

默认安全行为：`start_hardware:=false`、`start_vision:=true`、`command_mode:=zero`。Simulink 仍需单独启动，非零真机运动仍受标定、JointState 顺序和输入 watchdog 限制。

## 2026-07-22：加入 Stage 1 ROS 输入安全监督并整理工程缓存

修改文件：

```text
.gitignore
simulink/config/stereo_ibvs_config.m
simulink/ros2/stereo_ibvs_ros2_stage1.slx
simulink/build/sim/build_stereo_ibvs_sim_stage1.m
config/velocity_servo_tag.yaml
README.md
memory.md
```

完成内容：删除外部 `/simulink/controller_enable`，由包装模型根据 Joint/Vision freshness、数组长度和左相机 `validL` 自动产生内部使能；加入连续 3 帧丢失停止、连续 3 帧有效恢复、合法视觉保持和故障单脉冲复位；保留人工 `/simulink/reset`。平滑停车继续由 Python 最终命令节点负责，不在 Simulink 重复实现加速度限制。

配置：JointState 和 Vision 超时均为 `0.10 s`，目标判断只使用左相机。整理离线生成脚本路径和函数名，并将 Python、ROS 2、MATLAB/Simulink 和 macOS 缓存统一加入 `.gitignore`。

验证：包装模型通过 MATLAB R2025b Update Diagram；临时动态 Simulink 测试验证第 3 帧丢失停机、连续 3 帧恢复、Vision 超时、JointState 超时和人工复位；项目文件未进行真实 ROS 2 双相机或 FR3 非零运动测试。

## 2026-07-22：按实验流程精简重写 README

修改文件：

```text
README.md
memory.md
```

完成内容：按照“概述、项目框架、部署、运行、数据流向、ROS 2 接口、配置、安全、分阶段目标、测试”的结构重写 README；将编译和 source 命令分开，补充四个 Launch 的启动示例、全部可传入参数，以及标明节点、Simulink、Topic 和消息尺寸的 Mermaid 数据流图；测试章节按双相机、零速度全链路、JointState、标定和 FR3 低速实验排列。

下一项工作：开始实验前全面检查，确认代码和配置是否适合按照 README 的测试顺序进入真实设备实验。

## 2026-07-22：修正离线仿真配置覆盖并补充控制器依赖

修改文件：

```text
README.md
simulink/build/sim/build_stereo_ibvs_sim_stage1.m
simulink/core/stereo_ibvs_core.slx
simulink/sim/stereo_ibvs_sim_stage1.slx
memory.md
```

完成内容：Core 只在缺少完整 `cfg` 时加载正式配置；离线仿真改为按 `.slx` 位置寻找 core 和 config，并修正生成脚本的候选路径维度和首次保存顺序；README 明确依赖 `sunflower050105/franka_ros2` 的 `jazzy` 分支及其自定义 `JointVelocityExampleController`，补充控制器、Topic 订阅者和滤波参数检查命令，并记录 C++ 控制器当前没有命令 freshness watchdog。

验证：Core 和 ROS 2 包装模型通过 MATLAB R2025b Update Diagram；离线仿真通过 6 秒场景测试，偏移阶段最大绝对关节速度为 `0.03 rad/s`，关闭、居中和丢失阶段均为零；正式标定配置仍保持关闭。

## 2026-07-23 02:26：精简 README 运行与配置说明

修改文件：

```text
README.md
memory.md
```

完成内容：精简四个 Launch 的启动说明，只列出实际启动的节点或组件；在 MATLAB/Simulink 启动部分区分终端命令和 MATLAB 命令窗口，并补充每次启动及修改不同文件后的操作；配置表改为“配置文件、谁读取、控制哪些文件或节点”。保留用户已修改的 Python 虚拟环境说明，不修改 Launch 代码。

## 2026-07-22 工作总结

记录时间：2026-07-23 02:44

### `AGENTS.md`

- 修改：增加修改完成后提供中文 Commit 内容和文件分组的协作要求。
- 目的：方便后续按功能整理和提交修改。

### `.gitignore`

- 修改：补充 Python、ROS 2、MATLAB/Simulink 和 macOS 自动生成文件的忽略规则。
- 目的：避免缓存、构建目录和临时文件进入 Git。

### `README.md`

- 修改：重写项目概述、部署、运行、数据流、ROS 2 接口、配置、安全机制、阶段目标和测试流程；补充 sunflower `franka_ros2` 依赖、控制器检查命令、四个 Launch 的启动组件、MATLAB 启动和文件修改后的操作。
- 目的：形成可以直接用于 Stage 1 部署和实验的工程说明。
- 备注：真实双相机和 FR3 非零实验尚未完成。

### `memory.md`

- 修改：重构项目目标、文件架构、文件状态、实现计划和配置关系；同步当天全部修改和验证结果；规定后续日志按文件逐个记录。
- 目的：保证下一次工作能够直接接续当前状态。

### `setup.py`

- 修改：增加双目视觉节点安装入口并删除 Mapper 遗留入口。
- 目的：使 ROS 2 能够安装和启动 `vision_double_node`，同时清理已删除节点。

### `package.xml`

- 修改：清理已删除 Mapper 相关内容并同步当前运行依赖。
- 目的：让包描述与实际节点和启动方式一致。

### `config/velocity_servo_tag.yaml`

- 修改：增加双相机、AprilTag、60 Hz 发布、特征超时、Zoom 零占位和最终速度命令参数；统一 Simulink Topic 与安全参数记录。
- 目的：集中配置 `vision_double_node` 和 `velocity_command_node`。
- 备注：其中 `simulink_ros2` 段只记录接口，不会自动修改 Simulink 模型。

### `launch/vision_double.launch.py`

- 修改：新增双目视觉独立启动入口。
- 目的：便于单独启动和测试 `vision_double_node`。

### `launch/velocity_servo_tag.launch.py`

- 修改：改为启动 `vision_double_node`，并保留 `start_vision` 开关。
- 目的：作为 `full_system.launch.py` 的视觉入口，清理旧 Mapper 链路。

### `launch/full_system.launch.py`

- 修改：组合双目视觉和可选 FR3 硬件链路，默认 `start_hardware=false`、`command_mode=zero`。
- 目的：提供当前 Stage 1 的统一 ROS 2 启动入口。
- 备注：MATLAB/Simulink 仍需单独启动。

### `velocity_servo_tag/vision/stereo_features.py`

- 修改：新增双目特征组合、检测新鲜度、左右时间差和 Zoom 零占位算法。
- 目的：把左右 AprilTag 检测结果整理为 Simulink 使用的 8 维输入。

### `velocity_servo_tag/vision/vision_double_node.py`

- 修改：实现左右 USB 相机独立采集和 AprilTag 检测，发布双目特征与 `[0,0]` Zoom 位置。
- 目的：完成 Stage 1 双目视觉 ROS 2 输入节点。
- 备注：相机 ID、USB 带宽和真实 60 Hz 性能待 Ubuntu 实机验证。

### `velocity_servo_tag/velocity_command_node.py`

- 修改：统一接收 `/simulink/target_joints_velocities`；保留 zero/topic 模式、七维检查、速度限制、加速度限制、超时和平滑停车。
- 目的：作为 Simulink 与 FR3 C++ 速度控制器之间的最终安全转发层。

### `test/test_stereo_features.py`

- 修改：新增尺度、有效双目、检测超时、双目时间差和 Zoom 占位测试。
- 目的：验证不依赖真实相机的双目特征算法。
- 备注：共 6 项测试已经通过。

### `simulink/config/stereo_ibvs_config.m`

- 修改：加入 Joint/Vision freshness、目标丢失和恢复帧数等包装层参数，并保持真实标定许可为 `false`。
- 目的：统一 Core、ROS 2 包装和离线仿真的 Stage 1 参数。

### `simulink/ros2/stereo_ibvs_ros2_stage1.slx`

- 修改：删除外部 controller enable；加入 Joint/Vision 超时、三帧目标丢失停止、三帧恢复、合法视觉保持、自动内部使能和故障复位。
- 目的：在 Simulink 包装层完成输入有效性和目标丢失保护。
- 备注：Update Diagram 和动态安全场景测试已经通过。

### `simulink/build/sim/build_stereo_ibvs_sim_stage1.m`

- 修改：整理生成脚本名称和路径；改用相对模型位置查找 Core 与配置；修正候选路径维度和首次保存顺序。
- 目的：让离线仿真模型能够在不同用户名和工作区路径下重新生成。

### `simulink/core/stereo_ibvs_core.slx`

- 修改：调整 InitFcn，仅在缺少完整 `cfg` 时加载正式配置。
- 目的：避免 Core 覆盖离线仿真的临时标定许可，同时保持真实 ROS 2 模型默认锁零。
- 备注：MATLAB R2025b Update Diagram 已通过。

### `simulink/sim/stereo_ibvs_sim_stage1.slx`

- 修改：重新生成模型并移除 `/home/harry/...` 绝对路径。
- 目的：恢复可移植的 Stage 1 离线仿真。
- 备注：偏移阶段最大绝对关节速度为 `0.03 rad/s`，关闭、居中和目标丢失阶段均为零。

### 下一步

1. 在 Ubuntu 24.04 / ROS 2 Jazzy 编译项目并检查自定义速度控制器；
2. 设置左右相机 ID，验证 AprilTag 检测和实际发布频率；
3. 保持 `command_mode=zero`，完成双相机、Simulink、Python 和控制器全链路联调；
4. 核对 `/franka/joint_states` 前七项的关节顺序；
5. 完成左相机内参和手眼标定；
6. 准备实体急停后进行 FR3 低速实验；
7. Stage 1 实验验证完成后再进入双目深度等 Stage 2 内容。

## 2026-07-23 03:11：补充首次使用配置说明

### `README.md`

- 修改：在配置文件章节增加“首次使用需要配置的参数”，记录相机编号、采集模式、AprilTag、机器人 IP、相机内参、手眼矩阵、固定目标深度和标定许可的位置及获取方法；原配置关系和常用参数分别整理为 7.2、7.3。
- 目的：让首次部署和标定时能够直接确认需要修改哪些参数，以及每个参数从哪里获得。
- 备注：Stage 1 暂不要求双目标定和 Zoom 标定，速度、安全超时及底层滤波继续保留当前默认值。

### `memory.md`

- 修改：按逐文件格式记录本次 README 配置说明更新和时间。
- 目的：保证后续工作能够追踪配置文档的修改原因。

# 实现计划

说明：如果有分步实行的stage1，2，3，可以记录

- Stage 1：固定内参与固定深度的左相机中心 IBVS。core、ROS 2 包装、消息收发、双目视觉节点和 ROS 输入安全监督已完成；真实双相机联合实验、标定和硬件接入未完成。
- Stage 2：加入双目逆深度滤波与深度闭环任务。
- Stage 3：加入关节限位零空间任务和任务优先级控制。
- Stage 4：加入世界坐标系 9 状态目标 EKF `[p; v; a]` 和目标速度前馈。
- Stage 5：加入双目主动变焦，将累计步数通过标定曲线转换为焦距和相机内参，并处理回零、限位和编码器反馈。
- Stage 6：加入预测、时延补偿、更完整的故障恢复、奇异性监控和真机验证。

# 配置文件

说明：把配置文件，.yaml，.m配置文件简单记录

| 文件 | 作用 |
|---|---|
| `config/velocity_servo_tag.yaml` | 单目/双目检测节点和最终关节速度命令节点的 Topic、相机、watchdog、发布频率及安全限制参数 |
| `config/controllers.yaml` | `ros2_control` 控制器配置 |
| `simulink/config/stereo_ibvs_config.m` | Simulink 的采样时间、FR3 模型、相机参数、控制增益、阻尼、软限位、Joint/Vision 超时、目标丢失/恢复帧数和阶段参数 |
| `config/urdf/fr3.urdf` | `stereo_ibvs_config.m` 当前实际加载的 FR3 模型 |

配置关系：

- Simulink 的主要工作区参数由 `stereo_ibvs_config.m` 设置；模型块内部仍可能包含自身参数；
- Python 节点读取 `velocity_servo_tag.yaml`，Simulink `.slx` 当前不会读取这个 YAML；
- YAML 中现有 `simulink_ros2` 段只是接口记录，不会自动创建或修改 Simulink 接口；当前记录已与 `.slx` 的 `1/120 s` 和 `/simulink/target_joints_velocities` 对齐；
- ROS 包装安全参数实际由 `stereo_ibvs_config.m` 加载：Joint/Vision 超时 `0.10 s`、丢失 `3` 帧停止、恢复 `3` 帧使能，目标来源为左相机 `validL`；
- 当前 `cameraMountCalibrated`、`cameraIntrinsicsCalibrated`、`stereoCalibrationValid` 和 `zoomCalibrationValid` 均为 `false`；
- 后续手眼、双目和变焦标定结果计划独立保存在 `simulink/calibration/`。
