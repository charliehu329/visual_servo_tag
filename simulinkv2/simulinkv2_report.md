# simulinkv2 项目报告

生成日期：2026-07-13  
项目：主动变焦双目视觉伺服 Simulink 模型  
主模型：`arm_stereo_ibvs_ekf_v1.slx`

## 1. 项目概况

本项目基于 Franka Research 3（FR3）七自由度机器人、刚性平行双目相机、独立左右变焦和九状态匀加速 EKF，实现 60 Hz 离散视觉闭环。

- 机器人：Franka Research 3，运动学来自 `fr3.urdf`
- 离散周期：`Ts=1/60 s`
- 正式双目基线：`0.12 m`
- 图像尺寸：1920×1080
- 目标深度范围：0.5–1.0 m，期望深度0.75 m
- 焦距硬件范围：5.2–99 mm
- 正常工作范围：10–90 mm
- 设计/保证/硬限速度：9.3 / 15.5 / 18.75 mm/s

当前 `T_link8_CL=eye(4)`、主点 `(960,540)` 和 `0.00290 mm/output-pixel` 仍属于仿真占位假设。IMX415物理像元为1.45 μm，但实际1920×1080输出究竟来自binning、resize还是ROI尚未确认。

## 2. 控制与估计结构

机械臂主任务始终为左相机归一化图像中心：

```text
centerError = [xLeftMeasured; yLeftMeasured] - [0;0]
```

逆深度为机械臂次任务。由于使用阻尼伪逆，本模型只称为近似任务优先级。变焦优先状态机先调节焦距，随后通过smoothstep平滑引入机械臂深度任务。

EKF状态为世界坐标系九状态：

```text
xEKF = [pWorld; vWorld; aWorld]
```

正式测量为：

```text
zMeasured = [xLeftMeasured; yLeftMeasured; rhoMeasured]
rhoMeasured = (xLeftMeasured - xRightMeasured) / 0.12
```

EKF使用白jerk过程噪声、Joseph协方差更新和协方差对称化。控制器只使用 `vHatLeft` 和经过物理范围保护的 `rhoHatSafe`；`rhoTrue`、`pTrue`、`vTrue`、`aTrue` 只用于仿真对象、日志和性能评估。

## 3. 右相机可见性与降级

右相机优先满足80–1840 px安全可见区，然后才跟踪右图尺度。当前版本使用 `CURRENT_STATE_VISIBILITY_GUARD`，尚未实现下一采样一步预测。

右相机短时丢失而左相机仍有效时：

- 左中心主任务继续工作；
- 机械臂逆深度权重置零；
- EKF只预测，不使用无效视差更新；
- 右焦距向广角方向恢复；
- 连续5帧重新有效后恢复EKF测量更新；
- 深度任务通过smoothstep平滑恢复。

## 4. 实际验证结果

使用 MATLAB 25.2（R2025b Update 5）和 Simulink 25.2实际重建并运行。

- Update Diagram：PASS
- 求解器：FixedStepDiscrete
- 代数环：0
- 编译后MATLAB Function数量：16
- FR3 Jacobian验证：PASS，总相对误差约 `2.15e-08`
- 真值控制路径隔离：PASS
- 0.12 m解析双目公式：PASS，最大误差 `4.44e-16 1/m`
- 无噪声EKF：PASS，`rhoHat RMSE=0.00503352 1/m`
- 默认0.5 px噪声EKF：PASS，`rhoMeasured RMSE=0.00107759 1/m`，`rhoHat RMSE=0.00509075 1/m`
- 右相机丢失/重捕获：PASS，最长prediction-only约0.366667 s，重捕获恢复约0.0666667 s
- 动态复合场景：PASS，左右/双目有效比例均为1，`rhoHat RMSE=0.00436663 1/m`
- 所有正式运行：NaN/Inf计数为0

全套正式结论为 **FAIL**，不能写成完整系统已经验收。失败项为Z=0.5 m、左图中心附近的静态可见性：

```text
validLeftFraction   = 0.232365
validRightFraction  = 0.0248963
validStereoFraction = 0.0248963
rightVisibilityInfeasibleFraction = 0.796681
```

Z=0.75 m和Z=1.0 m静态工况通过。没有通过缩小正式基线、关闭左中心任务、使用真值控制或调整增益来掩盖Z=0.5 m失败。

## 5. 使用方法

要求 MATLAB/Simulink R2025b；其他较新版本通常也可尝试，但未在本次验证中确认。

解压后进入文件夹，在MATLAB命令窗口运行：

```matlab
START_HERE
```

该脚本会切换到包目录、执行初始化、加载主模型、执行Update Diagram并打开模型。也可手动执行：

```matlab
init_arm_stereo_ibvs_ekf_v1
load_system('arm_stereo_ibvs_ekf_v1')
set_param('arm_stereo_ibvs_ekf_v1','SimulationCommand','update')
simOut = sim('arm_stereo_ibvs_ekf_v1');
```

如需从构建源重新生成模型：

```matlab
build_arm_stereo_ibvs_ekf_v1
```

构建会使用包内 `active_stereo_ibvs.slx` 作为必要模板。

## 6. 包内文件

- `START_HERE.m`：一键初始化、检查并打开模型
- `arm_stereo_ibvs_ekf_v1.slx`：当前主模型
- `init_arm_stereo_ibvs_ekf_v1.m`：当前参数与初始化
- `build_arm_stereo_ibvs_ekf_v1.m`：主模型构建源
- `active_stereo_ibvs.slx`：重建所需模板
- `fr3.urdf`：FR3机器人描述
- `load_fr3_urdf_parameters.m`：URDF参数解析
- `fr3_camera_kinematics.m`：FR3相机运动学
- `focal_mm_to_pixels.m`、`focal_pixels_to_mm.m`：毫米/像素焦距转换
- `simulinkv2_report.md`：本报告

压缩包不包含测试日志、缓存、历史结果、旧交接文档或调参输出。

## 7. 交付包验证

压缩包仅包含上列11个文件。创建后已逐项比较压缩包条目与源文件SHA-256，结果一致。随后使用与包内相同的文件集合执行：

- `START_HERE`：PASS；
- 初始化和Update Diagram：PASS；
- 0.5 s烟雾仿真：PASS；
- 采样点：31；
- 日志信号：137；
- 运行时基线：0.12 m；
- 运行时采样周期：1/60 s。

## 8. 后续唯一建议

先确认实际1920×1080输出链并标定左右像素焦距、主点、畸变和相对外参，然后重新检查Z=0.5 m左中心可行域；在此之前不建议启动Kc、kRho、Kf调参。
