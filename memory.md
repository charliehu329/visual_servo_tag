# Simulink 部署项目记忆

## 1. 最终目标

构建一套工程化的 **Franka FR3 双目主动变焦视觉伺服系统**：

- 双目相机检测目标并输出图像特征；
- Simulink 完成特征处理、机器人运动学、视觉控制、状态估计和算法层安全保护；
- Simulink 直接输出 `7×1` 关节速度 `qDot`；
- Python ROS 2 节点只负责最终限速、加速度限制、watchdog、消息检查和底层转发；
- 后续加入双目逆深度闭环、关节限位零空间任务、目标运动 EKF、主动变焦和高速目标跟踪；
- 同一套核心算法同时服务于离线仿真、ROS 2 联调和真机部署。

---

## 2. 总体模型架构

统一使用一个核心模型，不为 Stage 1～6 复制多个独立 core：

```text
stereo_ibvs_core.slx
    核心控制算法，不直接订阅或发布ROS 2话题

stereo_ibvs_sim_stage1.slx
    离线仿真包装模型，为core提供测试输入

stereo_ibvs_ros2_stage1.slx
    后续ROS 2包装模型，负责话题订阅和发布
```

`sim` 和 `ros2` 都通过 Model 块引用同一个 `stereo_ibvs_core.slx`。

建议目录：

```text
visual_servo_tag/
├── config/
│   └── urdf/
│       └── fr3.urdf
│
├── simulink_new/
│   ├── core/
│   │   └── stereo_ibvs_core.slx
│   ├── config/
│   │   └── stereo_ibvs_config.m
│   ├── sim/
│   │   └── stereo_ibvs_sim_stage1.slx
│   ├── ros2/
│   │   └── stereo_ibvs_ros2_stage1.slx
│   ├── scripts/
│   │   ├── connect_stereo_ibvs_core_stage1_v5.m
│   │   └── build_stereo_ibvs_sim_stage1.m
│   └── calibration/
│       ├── camera_mount.mat
│       ├── stereo_calibration.mat
│       └── zoom_calibration.mat
│
└── velocity_servo_tag/
    └── Python ROS 2安全转发节点
```

`slprj/` 和 `.slxc` 都是缓存或临时构建文件，可以删除，后续会自动重新生成。

---

## 3. 已完成的核心模型

当前核心模型：

```text
stereo_ibvs_core.slx
```

顶层输入：

```text
qRaw                   7×1
visionFeatureRaw       8×1
zoomPositionStepsRaw   2×1
controllerEnableRaw    标量
resetRaw               标量
```

顶层输出：

```text
jointVelocityCmd       7×1
zoomStepRateCmd        2×1
controllerStatus       12×1
```

内部包含 10 个子系统：

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

---

## 4. 各子系统当前状态

### 01_Input_Validity

作用：

- 检查 NaN / Inf；
- 保存最近一次合法输入；
- 输入非法时关闭控制器；
- 处理 reset；
- 输出安全版本的输入。

统一尺寸：

```text
qRaw                   [7 1]
visionFeatureRaw       [8 1]
zoomPositionStepsRaw   [2 1]
controllerEnableRaw    1
resetRaw               1
```

### 02_Camera_Model

Stage 1 使用固定内参：

```text
cameraIntrinsics   8×1
focalLengthPx      2×1
cameraModelValid   标量
```

内参格式：

```text
[fxL; fyL; cxL; cyL; fxR; fyR; cxR; cyR]
```

Stage 5 再替换为“步进电机累计步数 → 当前焦距和内参”。

### 03_Feature_Processing

视觉输入格式：

```text
[validL; validR; uL; vL; uR; vR; scaleL; scaleR]
```

输出：

```text
cLMeasured      2×1
cRMeasured      2×1
scaleMeasured   2×1
validLeft       标量
validRight      标量
```

归一化坐标：

```text
x = (u-cx)/fx
y = (v-cy)/fy
```

尺度定义：

```text
scale = sqrt(AprilTag像素面积)
```

### 04_FR3_Camera_Kinematics

输出：

```text
JL                6×7
T_CL2B            4×4
kinematicsValid   标量
```

统一命名规则：

```text
T_CL2B：CL → B
T_B2CL：B → CL
T_CL2L8：CL → fr3_link8
T_L82CL：fr3_link8 → CL
R_CL2B：CL → B
R_B2CL：B → CL
```

`JL` 的速度排列统一为：

```text
[线速度; 角速度]
```

并在左相机坐标系中表达。

重要修正：URDF 中两个夹爪手指关节会使 Jacobian 变成 `6×9`。当前视觉伺服只控制 7 个机械臂关节，所以配置文件必须移除左右手指分支。验证：

```matlab
numel(homeConfiguration(cfg.robot))
```

必须输出：

```text
7
```

### 05_Target_EKF

Stage 1 为占位模块：

```text
pHatB    = 0
vHatB    = 0
aHatB    = 0
vHatCL   = 0
ekfValid = 0
```

Stage 4 替换为世界坐标系九状态 EKF：

```text
[p; v; a]
```

### 06_Inverse_Depth_Filter

Stage 1：

- 控制器仍使用固定逆深度；
- 双目逆深度测量暂用于诊断。

计算：

```text
disparityNormalized = xL-xR
rhoMeasured = disparityNormalized/stereoBaseline
```

Stage 2 加入真正的预测—校正逆深度滤波和深度闭环。

### 07_Arm_Priority_Controller

Stage 1 只实现左相机中心任务：

```text
Jc = Lc*JL
nuC = -Kc*centerError
qDotCenter = Jc#*nuC
```

使用阻尼最小二乘伪逆。

输出：

```text
qDotCenter          7×1
centerError         2×1
centerTaskValid     标量
jConditionMetric    标量
```

当前仍使用内部固定 `cfg.rhoD`。Stage 2 再把 `06.rhoForControl` 接入 07。

### 08_Zoom_Controller

Stage 1 为占位模块：

```text
zoomStepRateCmd = [0;0]
zoomControllerValid = 0
```

Stage 5 实现“尺度误差 → 焦距变化率 → 步进电机速度”。

### 09_Safety_Supervisor

功能：

- 检查 qDot 有限性；
- 算法层关节速度限幅；
- 关节软限位；
- 接近下限时禁止继续向下运动；
- 接近上限时禁止继续向上运动；
- 标定未完成或控制器未使能时输出零速度。

实际输入顺序：

```text
输入1 = q
输入2 = qDotRaw
输入3 = centerTaskValid
输入4 = controllerEnable
输入5 = stage1CalibrationReady
```

### 10_Diagnostics

`controllerStatus` 格式：

```text
[inputDataValid;
 cameraModelValid;
 kinematicsValid;
 validLeft;
 validRight;
 depthMeasurementValid;
 centerTaskValid;
 safetyValid;
 ekfValid;
 zoomControllerValid;
 controllerEnable;
 jConditionMetric]
```

---

## 5. 配置文件

配置文件：

```text
stereo_ibvs_config.m
```

当前包含：

- 采样时间；
- FR3 URDF 路径和机器人模型；
- 7 个关节名称；
- `qInitial`；
- `qMin/qMax`；
- 左相机虚拟刚体和 `T_CL2L8`；
- 固定相机内参；
- 双目基线；
- 初始变焦累计步数；
- 中心控制增益和阻尼；
- 算法层速度限制；
- 关节软限位；
- 标定许可；
- 仿真许可。

当前真实标定标志：

```matlab
cfg.cameraMountCalibrated = false;
cfg.cameraIntrinsicsCalibrated = false;
cfg.stereoCalibrationValid = false;
cfg.zoomCalibrationValid = false;
```

正常真机模式：

```matlab
cfg.stage1CalibrationReady = ...
    cfg.cameraMountCalibrated && ...
    cfg.cameraIntrinsicsCalibrated;
```

因此目前真实运动许可为 `0`，这是正确的安全状态。

---

## 6. 已解决的主要问题

### importrobot 的 HomePosition 警告

```text
Current joint home position is outside the new joint limits
```

这是警告，不是错误。部分 FR3 关节默认零位不在 URDF 合法范围内，MATLAB 会自动调整内部 HomePosition，不影响关节限位读取和 Jacobian 计算。

### 信号维度

已经统一：

```text
7维向量 → [7 1]
8维向量 → [8 1]
2维向量 → [2 1]
3维向量 → [3 1]
Jacobian → [6 7]
变换矩阵 → [4 4]
状态向量 → [12 1]
标志量 → 1
```

Constant 块应取消：

```text
Interpret vector parameters as 1-D
```

### Jacobian 6×9

原因是夹爪的两个手指关节被计入机器人自由度。正确做法是从 `cfg.robot` 移除左右手指分支，而不是把控制器改成 9 维。

### 模型 InitFcn

不再只写裸命令：

```matlab
stereo_ibvs_config
```

而是根据模型文件路径自动查找 `config/stereo_ibvs_config.m`，避免依赖 MATLAB 当前目录。

---

## 7. Stage 1 仿真

已经编写：

```text
build_stereo_ibvs_sim_stage1.m
```

目标生成：

```text
sim/stereo_ibvs_sim_stage1.slx
```

测试场景：

```text
0～0.5秒：控制器关闭，jointVelocityCmd应为0
0.5～3秒：目标偏离中心，jointVelocityCmd应非零
3秒以后：目标回到中心，jointVelocityCmd应回到0附近
```

仿真记录变量：

```matlab
simJointVelocityCmd
simZoomStepRateCmd
simControllerStatus
```

---

## 8. Stage 1～6 计划

### Stage 1：左相机中心控制

- 固定相机内参；
- 固定逆深度；
- 左相机中心任务；
- 输出 7 维关节速度；
- 完成离线仿真；
- 暂不连接真机。

当前状态：核心模型基本完成，正在验证仿真模型。

### Stage 2：双目逆深度闭环

主要修改：

```text
06_Inverse_Depth_Filter
07_Arm_Priority_Controller
```

加入真实逆深度滤波和深度控制任务，并连接：

```text
06.rhoForControl → 07.rhoForControl
```

### Stage 3：关节限位零空间任务

主要修改：

```text
07_Arm_Priority_Controller
```

加入：

```text
中心主任务
深度次任务
关节限位回避零空间任务
```

当前 Stage 3 只做关节限位回避，奇异性回避和姿态任务后置。

### Stage 4：世界坐标系九状态 EKF

主要修改：

```text
05_Target_EKF
07_Arm_Priority_Controller
```

状态：

```text
位置p、速度v、加速度a
```

控制器加入目标速度前馈。

### Stage 5：主动双目变焦

主要修改：

```text
02_Camera_Model
08_Zoom_Controller
```

硬件当前为步进电机，能够控制方向和速度，无真实焦距反馈。计划通过累计步数和标定曲线估计焦距：

```text
累计步数n → 标定曲线f=F(n) → 当前内参
```

最终需要可靠回零、限位开关或编码器，避免开环丢步后焦距估计漂移。

### Stage 6：高速目标和工程强化

加入：

- EKF前向预测；
- 延迟补偿；
- 视觉丢失保持；
- 输入超时；
- 速度平滑和变化率限制；
- 奇异性监控；
- 故障恢复；
- ROS 2实机部署。

---

## 9. 接下来要做什么

### 第一步：生成仿真模型

```matlab
build_stereo_ibvs_sim_stage1
```

目标：成功生成 `stereo_ibvs_sim_stage1.slx`，并且 Ctrl+D 更新通过。

### 第二步：运行 6 秒仿真

```matlab
sim('stereo_ibvs_sim_stage1')
```

检查：

```text
0～0.5秒 qDot=0
0.5～3秒 qDot非零
3秒以后 qDot回到0附近
```

### 第三步：验证控制方向

确认：

- 目标在图像右侧时，机械臂运动方向是否正确；
- 目标在图像上方时，运动方向是否正确；
- 交互矩阵和 Jacobian 是否存在符号反向；
- 相机光学坐标系和 `T_CL2L8` 定义是否一致。

手眼外参未标定前只做离线检查，不连接真机。

### 第四步：建立 ROS 2 包装模型

创建：

```text
stereo_ibvs_ros2_stage1.slx
```

负责：

- 订阅关节状态；
- 订阅视觉特征；
- 订阅变焦累计步数；
- 调用同一个 core；
- 发布 7 维 `jointVelocityCmd`；
- 发布变焦命令和诊断状态。

### 第五步：对接 Python 安全转发节点

最终链路：

```text
检测节点
  ↓
Simulink ROS 2包装模型
  ↓
stereo_ibvs_core
  ↓
7维qDot
  ↓
Python安全转发节点
  ↓
Franka底层速度控制器
```

---

## 10. 工程约束

### 代码注释

所有子系统 MATLAB Function 代码统一使用中文，并按以下顺序：

```matlab
% 代码作用：
% ...

% 输入参数含义：
% ...

% 输出参数含义：
% ...
```

### 安全分层

Simulink：

- 输入合法性；
- 算法层速度软限幅；
- 关节软限位；
- 标定许可；
- 失效输出零速度。

Python：

- 最终硬限速；
- 加速度限制；
- watchdog；
- ROS 2消息检查；
- 断流零速度；
- 最终底层转发。

### 配置分层

```text
config/velocity_servo_tag.yaml
```

负责 ROS 2 topic、Python安全转发、watchdog、发布频率和最终速度限制。

```text
simulink_new/config/stereo_ibvs_config.m
```

负责 Simulink 控制器、机器人、相机、增益、阻尼、软限位和 Stage 参数。

```text
simulink_new/calibration/*.mat
```

负责手眼标定、双目标定和变焦标定。

---

## 11. 当前状态一句话总结

已经完成一个可持续扩展到 Stage 1～6 的统一核心 Simulink 模型，并完成 Stage 1 的控制结构、端口、配置、安全逻辑和仿真构建脚本。

当前最重要的下一步是：

```text
让stereo_ibvs_sim_stage1.slx成功生成并通过离线仿真，
确认尺寸、有效状态、控制方向和收敛趋势正确。
```
