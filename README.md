# velocity_servo_tag

FR3 的 ROS 2 AprilTag 视觉伺服接口包框架。

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

## 构建与运行

把本目录放入 ROS 2 工作区的 `src` 后执行：

```bash
colcon build --packages-select velocity_servo_tag --symlink-install
source install/setup.bash
```

依赖 Python 包：`pinocchio`、`opencv-python`、`pupil-apriltags`。
底层 Franka 速度控制器需另外启动。

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
