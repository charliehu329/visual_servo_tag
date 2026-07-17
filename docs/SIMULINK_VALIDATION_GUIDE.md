# Simulink 与 `velocity_servo_tag` 分阶段验证指南

本文用于验证模型 `single_camera_xy_tracking(1).slx` 与 ROS 2 Python 包
`velocity_servo_tag` 的接口、控制逻辑和真机输出。

> 必须按顺序验证。前一阶段未通过时，不进入下一阶段；首次真机测试始终保持
> `dry_run=true`，并确保操作者可以立即使用急停。

## 1. 当前模型检查结论

新模型与 Python 的主要接口已经对应：

| 方向 | Topic | 类型 | 数据 |
|---|---|---|---|
| Python → Simulink | `/apriltag_detector/target_position` | `std_msgs/Float64MultiArray` | `[valid, u, v]` |
| Franka → Simulink | `/franka/joint_states` | `sensor_msgs/JointState` | 7 个关节的位置和速度 |
| Simulink → Python | `/simulink/camera_velocity` | `std_msgs/Float64MultiArray` | `[vx, vy, 0, 0, 0, 0]` |
| Python → Franka | `/velocity_mapper_node/target_joints_velocities` | `std_msgs/Float64MultiArray` | `[dq1, ..., dq7]` |

模型已经内置：

- 三维目标消息 `[valid, u, v]` 的解析；
- 目标消息和关节状态新鲜度检查；
- 六维相机速度输出；
- 无效输入时输出六维零速度的安全门；
- `Float64MultiArray` 六元素有效长度打包。

当前阻塞项：模型的 `InitFcn` 调用
`init_single_camera_xy_tracking.m`，但该文件目前不在模型目录或 MATLAB
搜索路径中。因此模型还不能直接编译运行。

## 2. 补齐初始化脚本

在 `single_camera_xy_tracking(1).slx` 同一目录创建：

```text
init_single_camera_xy_tracking.m
```

可以先使用下面的“只编译、禁止非零控制”模板：

```matlab
%% Model timing
Ts = 0.01;             % 100 Hz
T_end = 20.0;

%% ROS/offline selection
USE_ROS = false;       % 初始必须为 false

%% Camera calibration -- 真机前替换成当前分辨率的标定值
fx = 891.1;
fy = 879.2;
cx = 266.8;
cy = 200.8;

%% Depth and offline initial state
Z_hat = 0.50;
Z0 = 0.50;
X0 = 0.05;
Y0 = 0.05;
noise_pixel = zeros(2, 1);

%% Message watchdogs
target_timeout_sec = 0.20;
joint_state_timeout_sec = 0.10;

%% EKF matrices -- 仅供第一阶段结构验证
P0 = eye(6);
Q_ekf = 1.0e-6 * eye(6);
R_ekf = 1.0e-4 * eye(2);

%% Controller parameters
Kpx = 0.0;
Kpy = 0.0;
k_ff = 0.0;
gamma = 0.0;
sigma = 0.0;
adapt_max = 0.001;
v_max = 0.001;
a_max = 0.01;
nis_gate = 9.21;
enable_p = false;
enable_ff = false;
enable_adapt = false;
controller_enable = false;  % 初始硬锁：禁止非零速度
reset_timeout = 0.50;

controller_parameters = [
    Ts;
    Z_hat;
    Kpx;
    Kpy;
    k_ff;
    gamma;
    sigma;
    adapt_max;
    v_max;
    a_max;
    nis_gate;
    double(enable_p);
    double(enable_ff);
    double(enable_adapt);
    double(controller_enable);
    reset_timeout
];
```

这些数值只用于结构检查，不是最终控制参数。相机内参、深度、控制增益、
EKF 噪声和手眼矩阵必须通过实际标定和离线试验确定。

## 3. 阶段 A：Simulink 离线结构验证

1. MATLAB 进入模型目录。
2. 运行初始化脚本。
3. 打开模型并更新模型图。

```matlab
cd('<模型所在目录>')
init_single_camera_xy_tracking
open_system('single_camera_xy_tracking(1).slx')
set_param('single_camera_xy_tracking', 'SimulationCommand', 'update')
```

通过标准：

- 没有未定义变量；
- 没有消息总线或可变长度错误；
- 固定步长为 `Ts=0.01`；
- `camera_velocity` 始终是 6×1；
- `controller_enable=false` 时输出始终为六维零速度。

然后保持 `USE_ROS=false` 运行离线仿真。检查：

- 所有输出均为有限数，不出现 NaN/Inf；
- 目标无效时相机速度为零；
- 速度范数不超过 `v_max`；
- 相邻周期速度变化不超过 `a_max * Ts`。

## 4. 阶段 B：单独验证 Python 检测器

启动检测器，但不启动真机速度输出：

```bash
ros2 run velocity_servo_tag apriltag_detector \
  --ros-args --params-file \
  <工作区>/install/velocity_servo_tag/share/velocity_servo_tag/config/velocity_servo_tag.yaml
```

检查 Topic：

```bash
ros2 topic echo /apriltag_detector/target_position
ros2 topic hz /apriltag_detector/target_position
ros2 topic info -v /apriltag_detector/target_position
```

通过标准：

- 检测成功：`[1.0, u, v]`，恰好 3 个元素；
- 遮挡或移走 Tag：`[0.0, 0.0, 0.0]`；
- `u、v` 与显示窗口中的中心位置一致；
- 终端定期输出成功读取相机的实际 FPS；
- Topic 实际频率稳定且无持续丢帧。

## 5. 阶段 C：只验证 Simulink ROS 输入与零速度输出

保持：

```matlab
USE_ROS = true;
controller_enable = false;
```

启动 AprilTag 检测器和 Simulink 模型，不启动机器人运动。

检查 Simulink 是否收到：

- `target_count == 3`；
- `target_data == [valid; u; v]`；
- `target_age` 持续刷新；
- Tag 丢失时 `safe_valid == false`；
- `camera_velocity` 始终为六维零速度。

检查 Simulink 发布结果：

```bash
ros2 topic echo /simulink/camera_velocity
ros2 topic hz /simulink/camera_velocity
```

通过标准：

```text
data: [0, 0, 0, 0, 0, 0]
```

并且消息长度必须恰好为 6。若 `Ts=0.01`，发布频率应接近 100 Hz。

## 6. 阶段 D：验证关节状态输入

启动 Franka 驱动，但不要解锁非零速度控制。检查：

```bash
ros2 topic echo --once /franka/joint_states
ros2 topic hz /franka/joint_states
```

必须确认：

- `position` 至少包含 7 个机械臂关节；
- `velocity` 也至少包含 7 个元素；
- 数据中没有 NaN/Inf；
- 消息间隔小于 `joint_state_timeout_sec=0.1 s`。

注意：当前 Simulink 验证逻辑使用
`min(numel(position), numel(velocity))`。如果 `/franka/joint_states`
没有提供 velocity，`safe_valid` 将一直为 false。

## 7. 阶段 E：验证 Python 速度映射，保持 dry-run

启动 Python 后半段：

```bash
ros2 launch velocity_servo_tag velocity_servo_tag.launch.py \
  start_detector:=false dry_run:=true
```

此时启动 Simulink ROS 模式。先保持 `controller_enable=false`，再短时间输入
非常小的测试速度进行计算验证。`dry_run=true` 时 Python 不会向机器人发布
关节速度，只在日志中显示：

```text
q_dot_target
q_dot_cmd
```

通过标准：

- Python 收到的相机速度长度为 6；
- Jacobian 为 6×7；
- 目标和命令关节速度均为有限数；
- 各关节速度和加速度不超过 YAML 限制；
- 停止 Simulink 后约 0.2 s，Python 进入安全停止。

## 8. 阶段 F：验证两级看门狗

仍然保持 `dry_run=true`，依次测试：

1. 遮挡 Tag：Simulink 应持续发布六维零速度；
2. 停止 detector：Simulink 的 target watchdog 应输出零速度；
3. 停止 Simulink：Python 的 0.2 s watchdog 应触发；
4. 停止 `/franka/joint_states`：Python 的 joint-state watchdog 应触发；
5. 恢复数据：确认节点只在数据重新合法后恢复计算。

底层 `franka_velocity_ctrl` 还必须有独立的命令超时保护。Python 进程崩溃时
无法继续发布零速度，因此不能只依赖 Python 看门狗。

## 9. 阶段 G：真机零速度链路

只有前面阶段全部通过后才能执行：

1. 使用实时内核；
2. 电脑通过有线网口直连机器人；
3. 启动 Franka 底层速度控制器；
4. 保持 `controller_enable=false`；
5. 将 Python 改为 `dry_run=false`；
6. 确认两个速度 Topic 都持续为零。

```bash
ros2 topic echo /simulink/camera_velocity
ros2 topic echo /velocity_mapper_node/target_joints_velocities
```

通过标准：

- Simulink 输出始终为 6 个零；
- Python 输出始终为 7 个零；
- 机器人不运动；
- 停止任一上游节点后底层控制器安全停止且无持续非零命令。

## 10. 阶段 H：首次低速闭环运动

解锁前必须完成：

- 相机内参标定；
- `T_end_effector_camera` 手眼标定；
- 相机速度方向与 Python 坐标变换的单轴验证；
- 底层 watchdog 验证；
- 急停验证。

首次运动建议：

- 静止 AprilTag；
- 只启用比例项：`enable_p=true`；
- 暂时关闭 EKF 前馈与自适应补偿；
- 使用很小的 `Kpx、Kpy、v_max、a_max`；
- 先测试一个方向，再测试另一个方向；
- 确认误差减小而不是增大后，才逐步增加参数。

若目标向错误方向运动、出现振荡、Topic 超时、NaN/Inf 或速度跳变，立即急停，
恢复 `controller_enable=false` 和 `dry_run=true` 后排查。

## 11. 每次实验前检查表

- [ ] MATLAB 初始化脚本已运行且参数无未定义项
- [ ] `Ts` 与期望控制频率一致
- [ ] detector 消息为 `[valid,u,v]`
- [ ] Simulink 消息为六维相机速度
- [ ] `/franka/joint_states` 的 position 和 velocity 均至少 7 维
- [ ] 手眼矩阵不是未经确认的单位阵
- [ ] Python 速度/加速度限制已设置
- [ ] Simulink、Python 和底层三层安全停止均已验证
- [ ] 实时内核和有线网络正常
- [ ] 操作者可立即使用急停
