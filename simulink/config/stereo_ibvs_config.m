%% Stereo IBVS configuration
% 配置作用：
% 统一配置Stage 1双目视觉伺服核心模型需要的机器人、相机、
% 控制器、关节限位和安全参数。
%
% 本脚本运行后会在MATLAB基础工作区中生成结构体cfg。

%% 路径
% 当前脚本建议放在：
% simulink_new/config/stereo_ibvs_config.m
projectDir = fileparts(fileparts(mfilename('fullpath')));
repoDir = fileparts(projectDir);

cfg = struct();

%% 阶段与采样时间
cfg.stage = 1;

cfg.cameraFps = 60;
cfg.Ts = 1 / cfg.cameraFps;

%% FR3机器人模型
cfg.urdfPath = fullfile( ...
    repoDir, ...
    'config', ...
    'urdf', ...
    'fr3.urdf');

if ~isfile(cfg.urdfPath)
    error('StereoIBVS:URDFNotFound', ...
        '找不到FR3 URDF文件：%s', cfg.urdfPath);
end

% importrobot出现“home position超出关节限位”的警告通常不是错误。
% 原因是部分FR3关节的默认零位不在其合法范围内，
% MATLAB会自动把内部HomePosition调整到关节范围中心。
cfg.robot = importrobot(cfg.urdfPath);
cfg.robot.DataFormat = 'column';
cfg.robot.Gravity = [0 0 -9.81];

cfg.robotBaseName = cfg.robot.BaseName;
cfg.cameraBodyName = 'left_camera_optical';
cfg.cameraParentBodyName = 'fr3_link8';

%% FR3关节名称、初始位置和关节限位
cfg.jointNames = {
    'fr3_joint1'
    'fr3_joint2'
    'fr3_joint3'
    'fr3_joint4'
    'fr3_joint5'
    'fr3_joint6'
    'fr3_joint7'
};

% Stage 1离线仿真使用的合法初始关节位置，单位rad。
cfg.qInitial = [
    0
    -pi/4
    0
    -3*pi/4
    0
    pi/2
    pi/4
];

cfg.qMin = zeros(7,1);
cfg.qMax = zeros(7,1);

for jointIndex = 1:7
    jointFound = false;

    for bodyIndex = 1:numel(cfg.robot.Bodies)
        currentJoint = cfg.robot.Bodies{bodyIndex}.Joint;

        if strcmp(currentJoint.Name, cfg.jointNames{jointIndex})
            positionLimits = currentJoint.PositionLimits;

            if numel(positionLimits) ~= 2 || ...
                    any(~isfinite(positionLimits)) || ...
                    positionLimits(2) <= positionLimits(1)
                error('StereoIBVS:InvalidJointLimits', ...
                    '关节%s的位置限位无效。', ...
                    cfg.jointNames{jointIndex});
            end

            cfg.qMin(jointIndex) = positionLimits(1);
            cfg.qMax(jointIndex) = positionLimits(2);
            jointFound = true;
            break;
        end
    end

    if ~jointFound
        error('StereoIBVS:JointNotFound', ...
            '机器人模型中找不到关节：%s', ...
            cfg.jointNames{jointIndex});
    end
end

if any(cfg.qInitial <= cfg.qMin) || any(cfg.qInitial >= cfg.qMax)
    error('StereoIBVS:InvalidInitialConfiguration', ...
        'cfg.qInitial中至少有一个关节不在URDF关节限位内。');
end

%% 左相机安装关系
% T_CL2L8：
% 将左相机坐标系CL中的坐标转换到fr3_link8坐标系。
%
% p_L8 = T_CL2L8 * p_CL
%
% 当前仅为单位阵占位值。
% 完成手眼标定前，禁止真实机械臂运动。
cfg.T_CL2L8 = eye(4);
cfg.cameraMountCalibrated = false;

cameraBody = rigidBody(cfg.cameraBodyName);
cameraJoint = rigidBodyJoint( ...
    'left_camera_fixed_joint', ...
    'fixed');

setFixedTransform( ...
    cameraJoint, ...
    cfg.T_CL2L8);

cameraBody.Joint = cameraJoint;

addBody( ...
    cfg.robot, ...
    cameraBody, ...
    cfg.cameraParentBodyName);

%% Stage 1固定相机内参
% 格式：
% [fxL; fyL; cxL; cyL; fxR; fyR; cxR; cyR]
%
% 当前数值仅用于模型编译和离线联调。
% 真机实验前必须替换为实际标定结果。
cfg.cameraIntrinsicsStage1 = [
    5250
    5250
    960
    540
    5250
    5250
    960
    540
];

cfg.cameraIntrinsicsCalibrated = false;

%% 双目标定参数
% 左右相机光心基线长度，单位m。
% 当前仅为占位值。
cfg.stereoBaseline = 0.12;
cfg.stereoCalibrationValid = false;

%% 变焦位置
% 左右变焦步进电机累计位置，格式：
% [leftSteps; rightSteps]
cfg.zoomInitialSteps = [0;0];
cfg.zoomCalibrationValid = false;

%% Stage 1中心任务控制器
% 期望归一化图像中心。
cfg.centerDesired = [0;0];

% Stage 1使用固定目标深度和固定逆深度。
cfg.Zd = 0.75;
cfg.rhoD = 1 / cfg.Zd;

% 中心误差反馈增益。
cfg.Kc = diag([1.0,1.0]);

% 阻尼最小二乘系数。
cfg.lambdaC = 0.03;

% 默认关闭控制器。
cfg.controllerEnableDefault = 0;

%% Simulink算法层安全限制
% 各关节最大算法输出速度，单位rad/s。
cfg.qDotAlgorithmMax = 0.03 * ones(7,1);

% 关节软限位区域宽度，单位rad。
cfg.qLimitSoftMargin = 5*pi/180;

%% Stage 1真实运动许可
% Stage 1只使用左相机中心任务，因此至少要求：
% 1. 左相机安装矩阵已经标定；
% 2. 左相机内参已经标定。
%
% stereoCalibrationValid在Stage 2启用双目深度闭环后再加入许可条件。
cfg.stage1CalibrationReady = ...
    cfg.cameraMountCalibrated && ...
    cfg.cameraIntrinsicsCalibrated;

%% 写入MATLAB基础工作区
assignin('base','cfg',cfg);

fprintf('\nStereo IBVS配置加载完成。\n');
fprintf('Stage：%d\n',cfg.stage);
fprintf('采样时间：%.6f s\n',cfg.Ts);
fprintf('机器人基座名称：%s\n',cfg.robotBaseName);
fprintf('真实运动许可：%d\n',double(cfg.stage1CalibrationReady));
