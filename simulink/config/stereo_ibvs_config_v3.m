%% Stereo IBVS V2 deployment configuration
% 配置作用：
% 将已验证的 arm_stereo_ibvs_ekf_v1（V2）算法迁移到
% stereo_ibvs_core，同时保留 ROS 2 包装和 Python 安全转发所需参数。
%
% 统一接口：
% 1. 焦距反馈 Topic 直接发布 [fL_mm; fR_mm]；
% 2. Core 内仅在 02_Camera_and_Zoom_Model 中执行 mm -> pixel 焦距换算；
% 3. Zoom 命令直接使用 [fDotL_mm_s; fDotR_mm_s]，不再使用 step/s；
% 4. 启动后收到第一帧有效焦距前，cameraModelValid 必须为 false；
% 5. 不包含目标轨迹、关节积分、相机投影、真值和噪声注入等仿真对象。

%% 1. 路径与配置版本
projectDir = fileparts(fileparts(mfilename('fullpath')));
repoDir = fileparts(projectDir);

cfg = struct();
cfg.configurationName = 'v2_full_deployment_mm_interface';
cfg.configurationVersion = 3;
cfg.stage = 5;

%% 2. 采样频率
% V2 Core 按 60 Hz 运行。
cfg.cameraFps = 60;
cfg.visionRateHz = cfg.cameraFps;
cfg.controlRateHz = 60;
cfg.Ts = 1 / cfg.controlRateHz;

% Simulink 以 Core 周期发布，Python 安全转发层可按 120 Hz 保持并转发。
cfg.simulinkPublishRateHz = cfg.controlRateHz;
cfg.pythonSafetyRateHz = 120;

%% 3. ROS 2 焦距反馈与焦距速度命令接口
% 焦距反馈：
% data = [leftFocalLengthMm; rightFocalLengthMm]
cfg.focalLengthTopic = '/stereo/focal_length';
cfg.focalLengthMessageType = 'std_msgs/Float64MultiArray';
cfg.focalLengthMessageLength = 2;
cfg.focalLengthRateHz = 60;
cfg.focalLengthInputUnit = 'mm';

% 连续 5 个 Core 周期没有新焦距消息时：
% 保持最近一次有效焦距，但 focalLengthFresh=false，Zoom 命令归零。
cfg.focalLengthTimeoutFrames = 5;
cfg.focalLengthTimeoutSec = ...
    cfg.focalLengthTimeoutFrames / cfg.controlRateHz;

% Zoom 命令：
% data = [leftFocalRateMmPerSec; rightFocalRateMmPerSec]
cfg.focalRateCommandTopic = '/simulink/focal_rate_cmd';
cfg.focalRateCommandMessageType = 'std_msgs/Float64MultiArray';
cfg.focalRateMessageLength = 2;
cfg.focalRateCommandUnit = 'mm/s';

% 状态记忆块的安全初值。该值不是有效焦距，不能使 cameraModelValid=true。
cfg.focalLengthStateInitialMm = zeros(2,1);

%% 4. FR3 URDF 与机器人模型
cfg.urdfPath = fullfile(repoDir, 'config', 'urdf', 'fr3.urdf');

if ~isfile(cfg.urdfPath)
    error('StereoIBVS:URDFNotFound', ...
        '找不到 FR3 URDF 文件：%s', cfg.urdfPath);
end

fr3 = parseFr3UrdfForController(cfg.urdfPath);

cfg.fr3RobotName = fr3.robotName;
cfg.fr3OriginXYZ = fr3.originXYZ;
cfg.fr3OriginRPY = fr3.originRPY;
cfg.fr3Axis = fr3.axis;
cfg.fr3Joint8OriginXYZ = fr3.joint8OriginXYZ;
cfg.fr3Joint8OriginRPY = fr3.joint8OriginRPY;
cfg.fr3QMinURDF = fr3.qMin;
cfg.fr3QMaxURDF = fr3.qMax;
cfg.fr3QDotMaxURDF = fr3.qDotMax;

cfg.robot = importrobot(cfg.urdfPath);
cfg.robot.DataFormat = 'column';
cfg.robot.Gravity = [0 0 -9.81];

if any(strcmp(cfg.robot.BodyNames, 'fr3_leftfinger'))
    removeBody(cfg.robot, 'fr3_leftfinger');
end

if any(strcmp(cfg.robot.BodyNames, 'fr3_rightfinger'))
    removeBody(cfg.robot, 'fr3_rightfinger');
end

if numel(homeConfiguration(cfg.robot)) ~= 7
    error('StereoIBVS:RobotDOFMismatch', ...
        '移除夹爪手指分支后，FR3 机器人模型仍不是 7 自由度。');
end

cfg.robotBaseName = cfg.robot.BaseName;
cfg.cameraBodyName = 'left_camera_optical';
cfg.cameraParentBodyName = 'fr3_link8';

cfg.jointNames = {
    'fr3_joint1'
    'fr3_joint2'
    'fr3_joint3'
    'fr3_joint4'
    'fr3_joint5'
    'fr3_joint6'
    'fr3_joint7'
};

cfg.q0 = [
    0
    -pi/4
    0
    -3*pi/4
    0
    pi/2
    pi/4
];

cfg.qMin = fr3.qMin;
cfg.qMax = fr3.qMax;
cfg.qMid = 0.5 * (cfg.qMin + cfg.qMax);
cfg.qDotMax = fr3.qDotMax;
cfg.jointTorqueMax = fr3.jointTorqueMax;

% 当前工程兼容别名。
cfg.qInitial = cfg.q0;

if any(cfg.q0 <= cfg.qMin) || any(cfg.q0 >= cfg.qMax)
    error('StereoIBVS:InvalidInitialConfiguration', ...
        'cfg.q0 中至少有一个关节不在 URDF 关节限位内。');
end

%% 5. 世界、左相机与右相机安装关系
% T_A_B 表示 B 坐标系相对于 A 坐标系的位姿。
cfg.T_W_B = eye(4);

% 左相机相对于 fr3_link8 的安装变换。
% 当前为占位值，手眼标定后替换。
cfg.T_link8_CL = eye(4);
cfg.T_CL2L8 = cfg.T_link8_CL;
cfg.cameraMountCalibrated = false;
cfg.cameraMountIsPlaceholder = ~cfg.cameraMountCalibrated;

% 双目外参。V2 使用校正后的平行双目模型。
cfg.baseline = 0.12;
cfg.B = cfg.baseline;
cfg.stereoBaseline = cfg.baseline;
cfg.R_CL_CR = eye(3);
cfg.p_CL_CR = [cfg.baseline; 0; 0];
cfg.T_CL_CR = [
    cfg.R_CL_CR, cfg.p_CL_CR
    0 0 0 1
];
cfg.stereoCalibrationValid = false;

cameraBody = rigidBody(cfg.cameraBodyName);
cameraJoint = rigidBodyJoint('left_camera_fixed_joint', 'fixed');
setFixedTransform(cameraJoint, cfg.T_link8_CL);
cameraBody.Joint = cameraJoint;
addBody(cfg.robot, cameraBody, cfg.cameraParentBodyName);

%% 6. 图像尺寸、主点与实时焦距换算
cfg.imageWidthPx = 1920;
cfg.imageHeightPx = 1080;

% 兼容 V2 旧命名。
cfg.imageWidth = cfg.imageWidthPx;
cfg.imageHeight = cfg.imageHeightPx;

% 暂时使用图像中心，正式相机标定后替换。
cfg.cxL = cfg.imageWidthPx / 2;
cfg.cyL = cfg.imageHeightPx / 2;
cfg.cxR = cfg.imageWidthPx / 2;
cfg.cyR = cfg.imageHeightPx / 2;

% 等效输出像元尺寸，单位 mm/pixel。
% 当前 2.90e-3 为占位值，确认实际值后替换并设为 true。
cfg.outputPixelPitchXmm = 2.90e-3;
cfg.outputPixelPitchYmm = 2.90e-3;
cfg.pixelPitchCalibrated = false;

% 主点、畸变和成像模型是否已正式标定。
cfg.cameraIntrinsicsCalibrated = false;

cfg.focalMmToPixelsIsPlaceholder = ...
    ~cfg.pixelPitchCalibrated;

cfg.cameraIntrinsicsArePlaceholder = ...
    ~cfg.cameraIntrinsicsCalibrated || ...
    cfg.focalMmToPixelsIsPlaceholder;

% 02 模块中的唯一实时换算：
% fxMeasuredPx = focalLengthMeasuredMm / outputPixelPitchXmm
% fyMeasuredPx = focalLengthMeasuredMm / outputPixelPitchYmm
%
% 不设置虚假的 focalLength0Mm、fx0Px、fy0Px。
% 第一帧真实焦距到来前，相机模型保持无效。

%% 7. 焦距范围与焦距速度限制
cfg.focalLengthHardwareMinMm = [5; 5];
cfg.focalLengthHardwareMaxMm = [99; 99];

cfg.focalLengthWorkingMinMm = [10; 10];
cfg.focalLengthWorkingMaxMm = [90; 90];

if any(cfg.focalLengthHardwareMinMm >= ...
        cfg.focalLengthHardwareMaxMm) || ...
        any(cfg.focalLengthHardwareMinMm > ...
        cfg.focalLengthWorkingMinMm) || ...
        any(cfg.focalLengthWorkingMinMm >= ...
        cfg.focalLengthWorkingMaxMm) || ...
        any(cfg.focalLengthWorkingMaxMm > ...
        cfg.focalLengthHardwareMaxMm)
    error('StereoIBVS:InvalidFocalLengthRanges', ...
        '焦距硬件范围和工作范围不一致。');
end

% V2 Zoom 执行器能力，全部使用 mm/s。
cfg.focalRateGuaranteedMmPerSec = 15.5;
cfg.focalRateAbsoluteMaxMmPerSec = 18.75;
cfg.focalRateUnit = 'mm/s';

cfg.etaZoom = 0.60;
cfg.focalRateDesignMmPerSec = ...
    cfg.etaZoom * cfg.focalRateGuaranteedMmPerSec;

cfg.rightReacquireZoomRateMmPerSec = ...
    cfg.focalRateDesignMmPerSec;

% Python 底层是否已经验证：
% 1. 接收 mm/s；
% 2. 正负方向正确；
% 3. 速度限制正确；
% 4. 停止命令可靠。
cfg.focalRateCommandInterfaceValidated = false;
cfg.zoomCalibrationValid = ...
    cfg.focalRateCommandInterfaceValidated;
cfg.zoomCalibrationIsPlaceholder = ...
    ~cfg.zoomCalibrationValid;
cfg.zoomRateLimitIsPlaceholder = ...
    ~cfg.focalRateCommandInterfaceValidated;

%% 8. 视觉测量、可见性与双目逆深度
cfg.numericalEpsilon = 1e-8;
cfg.visibilityEpsilon = 1e-6;
cfg.visibilityZMin = 0.10;

cfg.targetDepthMin = 0.50;
cfg.targetDepthMax = 1.00;
cfg.Zd = 0.75;
cfg.rhoD = 1 / cfg.Zd;

cfg.rhoEstimateMin = 0.80;
cfg.rhoEstimateMax = 2.20;
cfg.rhoMin = cfg.rhoEstimateMin;
cfg.rhoMax = cfg.rhoEstimateMax;
cfg.disparityMin = 1e-4;

cfg.rightVisibilityMarginPx = 80;
cfg.rightVisibilityHysteresisPx = 20;
cfg.rightReacquireValidSamples = 5;
cfg.usePredictiveRightVisibility = false;

% AprilTag 尺度：四角面积平方根。
cfg.targetCharacteristicSize = 0.10;
cfg.scaleDesired = [700; 700];
cfg.scaleDesignIsProvisional = true;

%% 9. 完整 Arm Priority Controller
cfg.centerDesired = [0; 0];

cfg.Kc = diag([2.5, 2.5]);
cfg.kRho = 1.5;

cfg.lambdaC = 0.02;
cfg.lambdaRho = 0.02;

cfg.betaC = 0;
cfg.betaRho = 0;
cfg.epsilonC = 1e-3;
cfg.epsilonRho = 1e-3;

cfg.robustEnable = 0;
cfg.nullspaceEnable = 0;
cfg.kNull = 0.05;

cfg.armControlEnable = 1;
cfg.centerTaskEnable = 1;
cfg.depthTaskEnable = 1;
cfg.leftOnlyCenterControlEnable = true;

% 总使能由 ROS 2 包装和安全监督共同决定。
cfg.controllerEnableDefault = 0;

% 用于打断 Core 内反馈环的两个 Unit Delay 初值。
cfg.qDotAppliedInitial = zeros(7,1);
cfg.depthErrorInitial = 0;

%% 10. Target EKF
% 表示检测噪声，不向真实测量主动注入噪声。
cfg.pixelNoiseStd = 0.5;
cfg.Rpixel = cfg.pixelNoiseStd^2 * eye(4);

cfg.sigmaAcceleration = 0.5;
cfg.sigmaJerk = 1.0;
cfg.ekfCovarianceJitter = 1e-12;
cfg.ekfSConditionMin = 1e-12;

% 首次有效双目测量初始化。
cfg.ekfInitializationMode = 1;
cfg.ekfRequireNewMeasurement = true;

% 测量协方差由 05 模块根据实时 fxMeasuredPx、fyMeasuredPx 和 baseline 计算。
cfg.ekfMeasurementCovarianceRuntime = true;

% 首次测量到来前的安全后备状态。
T_W_CL0 = getTransform( ...
    cfg.robot, ...
    cfg.q0, ...
    cfg.cameraBodyName, ...
    cfg.robotBaseName);

cfg.pWCL0 = T_W_CL0(1:3,4);
cfg.RWCL0 = T_W_CL0(1:3,1:3);
cfg.target0CL = [0; 0; cfg.Zd];
cfg.target0 = cfg.pWCL0 + cfg.RWCL0 * cfg.target0CL;

cfg.ekfX0 = [
    cfg.target0
    0
    0
    0
    0
    0
    0
];

cfg.ekfP0 = diag([
    0.02^2
    0.02^2
    0.05^2
    0.20^2
    0.20^2
    0.20^2
    0.50^2
    0.50^2
    0.50^2
]);

%% 11. Zoom Controller
cfg.Kf = diag([1.5, 1.5]);
cfg.betaF = [0; 0];
cfg.epsilonF = [1e-3; 1e-3];
cfg.zoomControlEnable = 1;

%% 12. Zoom Priority Supervisor
cfg.zoomPriorityEnable = true;
cfg.scaleErrorEnterThreshold = 0.04;
cfg.scaleErrorExitThreshold = 0.015;
cfg.scaleSettledHoldTime = 0.25;
cfg.disturbanceConfirmTime = 0.10;
cfg.zoomOnlyMaxTime = 1.00;
cfg.armDepthRampTime = 0.50;
cfg.zoomLimitMarginFraction = 0.02;

cfg.depthErrorLoggingThreshold = 0.02;
cfg.zoomResponseThreshold = 1e-3;
cfg.armDepthResponseThreshold = 1e-4;

%% 13. V2 Safety and Saturation
cfg.qLimitSoftMargin = 5*pi/180;
cfg.cartesianLinearSpeedMax = 2.0;

% Core 最终算法层低速限制。
cfg.qDotAlgorithmMax = ...
    min(cfg.qDotMax, 0.03 * ones(7,1));

%% 14. ROS 2 输入监督与消息尺寸
cfg.jointStateTimeoutSec = 0.10;
cfg.visionTimeoutSec = 0.10;
cfg.visionTimeoutFrames = max(1,ceil(cfg.visionTimeoutSec/cfg.Ts));

cfg.targetLossFrameLimit = 3;
cfg.targetRecoveryFrameCount = 3;

cfg.visionMessageLength = 8;
cfg.jointPositionMessageLength = 7;
cfg.jointVelocityMessageLength = 7;
cfg.controllerStatusMessageLength = 13;

% controllerStatus 固定顺序：
% [inputDataValid;
%  cameraModelValid;
%  focalLengthFresh;
%  kinematicsValid;
%  validLeft;
%  validRight;
%  validStereoQualified;
%  ekfPredictionValid;
%  ekfMeasurementUpdated;
%  depthTaskWeight;
%  schedulerMode;
%  safetyValid;
%  controllerEnableSafe]
cfg.controllerStatusOrder = {
    'inputDataValid'
    'cameraModelValid'
    'focalLengthFresh'
    'kinematicsValid'
    'validLeft'
    'validRight'
    'validStereoQualified'
    'ekfPredictionValid'
    'ekfMeasurementUpdated'
    'depthTaskWeight'
    'schedulerMode'
    'safetyValid'
    'controllerEnableSafe'
};

% data = [validL; validR; uL; vL; uR; vR; scaleL; scaleR]
cfg.visionFeatureOrder = {
    'validL'
    'validR'
    'uL'
    'vL'
    'uR'
    'vR'
    'scaleL'
    'scaleR'
};

%% 15. 标定许可与真机安全锁
cfg.cameraModelCalibrationReady = ...
    cfg.cameraIntrinsicsCalibrated && ...
    cfg.pixelPitchCalibrated;

cfg.armControlCalibrationReady = ...
    cfg.cameraMountCalibrated && ...
    cfg.cameraModelCalibrationReady;

cfg.depthControlCalibrationReady = ...
    cfg.armControlCalibrationReady && ...
    cfg.stereoCalibrationValid;

cfg.zoomControlCalibrationReady = ...
    cfg.depthControlCalibrationReady && ...
    cfg.focalRateCommandInterfaceValidated;

cfg.fullDeploymentReady = ...
    (~logical(cfg.armControlEnable) || ...
        cfg.armControlCalibrationReady) && ...
    (~logical(cfg.depthTaskEnable) || ...
        cfg.depthControlCalibrationReady) && ...
    (~logical(cfg.zoomControlEnable) || ...
        cfg.zoomControlCalibrationReady);

% 当前工程兼容别名。
cfg.stage1CalibrationReady = ...
    cfg.armControlCalibrationReady;
cfg.controllerCalibrationReady = ...
    cfg.fullDeploymentReady;

%% 16. 写入 MATLAB 基础工作区
% 整个项目统一只维护 cfg 结构体。
% Simulink Constant 块使用 cfg.xxx，不再生成 cfg_xxx 独立变量。
assignin('base', 'cfg', cfg);

fprintf('\nStereo IBVS V2 部署配置加载完成。\n');
fprintf('配置：%s\n', cfg.configurationName);
fprintf('Core周期：%.6f s（%.1f Hz）\n', ...
    cfg.Ts, cfg.controlRateHz);
fprintf('焦距反馈：%s，单位%s\n', ...
    cfg.focalLengthTopic, cfg.focalLengthInputUnit);
fprintf('Zoom命令：%s，单位%s\n', ...
    cfg.focalRateCommandTopic, cfg.focalRateCommandUnit);
fprintf('Controller状态长度：%d\n', ...
    cfg.controllerStatusMessageLength);
fprintf('Arm标定许可：%d\n', ...
    double(cfg.armControlCalibrationReady));
fprintf('Depth标定许可：%d\n', ...
    double(cfg.depthControlCalibrationReady));
fprintf('Zoom标定许可：%d\n', ...
    double(cfg.zoomControlCalibrationReady));
fprintf('完整真机许可：%d\n', ...
    double(cfg.fullDeploymentReady));

clear projectDir repoDir fr3 cameraBody cameraJoint T_W_CL0;


function fr3 = parseFr3UrdfForController(urdfFile)
% 从 URDF 提取 V2 固定尺寸运动学和关节限制参数。

doc = xmlread(urdfFile);
root = doc.getDocumentElement;
fr3.robotName = char(root.getAttribute('name'));

jointNodes = root.getElementsByTagName('joint');
numberOfJoints = jointNodes.getLength;

jointTemplate = struct( ...
    'name', '', ...
    'type', '', ...
    'originXYZ', zeros(3,1), ...
    'originRPY', zeros(3,1), ...
    'axis', zeros(3,1), ...
    'lower', NaN, ...
    'upper', NaN, ...
    'velocity', NaN, ...
    'effort', NaN);

allJoints = repmat( ...
    jointTemplate, ...
    numberOfJoints, ...
    1);

for jointIndex = 1:numberOfJoints
    node = jointNodes.item(jointIndex-1);

    currentJoint = jointTemplate;
    currentJoint.name = ...
        char(node.getAttribute('name'));
    currentJoint.type = ...
        char(node.getAttribute('type'));

    currentJoint.originXYZ = parseUrdfVector( ...
        readChildAttribute( ...
        node, 'origin', 'xyz', '0 0 0'));

    currentJoint.originRPY = parseUrdfVector( ...
        readChildAttribute( ...
        node, 'origin', 'rpy', '0 0 0'));

    currentJoint.axis = parseUrdfVector( ...
        readChildAttribute( ...
        node, 'axis', 'xyz', '0 0 0'));

    currentJoint.lower = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'lower', 'NaN'));

    currentJoint.upper = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'upper', 'NaN'));

    currentJoint.velocity = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'velocity', 'NaN'));

    currentJoint.effort = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'effort', 'NaN'));

    allJoints(jointIndex) = currentJoint;
end

armJoints = repmat(jointTemplate, 7, 1);

for armIndex = 1:7
    expectedName = ...
        sprintf('fr3_joint%d', armIndex);

    foundIndex = find( ...
        strcmp({allJoints.name}, expectedName), ...
        1);

    if isempty(foundIndex)
        error('StereoIBVS:JointNotFound', ...
            'URDF 中找不到关节：%s', ...
            expectedName);
    end

    armJoints(armIndex) = ...
        allJoints(foundIndex);

    if ~strcmp( ...
            armJoints(armIndex).type, ...
            'revolute')
        error('StereoIBVS:InvalidJointType', ...
            '%s 类型为 %s，应为 revolute。', ...
            expectedName, ...
            armJoints(armIndex).type);
    end
end

joint8Index = find( ...
    strcmp({allJoints.name}, 'fr3_joint8'), ...
    1);

if isempty(joint8Index)
    error('StereoIBVS:Joint8NotFound', ...
        'URDF 中找不到 fr3_joint8。');
end

joint8 = allJoints(joint8Index);

fr3.originXYZ = ...
    reshape([armJoints.originXYZ], 3, 7);
fr3.originRPY = ...
    reshape([armJoints.originRPY], 3, 7);
fr3.axis = ...
    reshape([armJoints.axis], 3, 7);
fr3.qMin = ...
    reshape([armJoints.lower], 7, 1);
fr3.qMax = ...
    reshape([armJoints.upper], 7, 1);
fr3.qDotMax = ...
    reshape([armJoints.velocity], 7, 1);
fr3.jointTorqueMax = ...
    reshape([armJoints.effort], 7, 1);
fr3.joint8OriginXYZ = ...
    joint8.originXYZ;
fr3.joint8OriginRPY = ...
    joint8.originRPY;

if any(~isfinite(fr3.qMin)) || ...
        any(~isfinite(fr3.qMax)) || ...
        any(~isfinite(fr3.qDotMax)) || ...
        any(~isfinite(fr3.jointTorqueMax))
    error('StereoIBVS:InvalidURDFLimits', ...
        'FR3 URDF 中至少有一个关节限制不是有限值。');
end
end


function value = readChildAttribute( ...
        node, ...
        tagName, ...
        attributeName, ...
        defaultValue)

children = node.getElementsByTagName(tagName);

if children.getLength == 0
    value = defaultValue;
    return;
end

value = ...
    char(children.item(0).getAttribute(attributeName));

if isempty(value)
    value = defaultValue;
end
end


function vector = parseUrdfVector(textValue)
vector = sscanf(textValue, '%f');

if numel(vector) ~= 3
    error('StereoIBVS:URDFVectorParse', ...
        '无法解析三维向量：%s', ...
        textValue);
end

vector = reshape(vector, 3, 1);
end


function scalar = parseUrdfScalar(textValue)
scalar = sscanf(textValue, '%f', 1);

if isempty(scalar)
    scalar = NaN;
end
end