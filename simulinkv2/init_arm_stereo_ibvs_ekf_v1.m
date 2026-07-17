%% Initialization for arm_stereo_ibvs_ekf_v1
% All tunable model and test parameters are centralized in cfg.

hasCfgOverride = evalin('base','exist(''cfgOverride'',''var'') == 1');
if hasCfgOverride
    cfgOverrideLocal = evalin('base','cfgOverride');
end

cfg = struct();
cfg.cameraFps = 60;
cfg.Ts = 1/cfg.cameraFps;
cfg.stopTime = 30;

% ROS 2接口与Python节点共用同一份YAML。
rosYamlFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'config','velocity_servo_tag.yaml');
if ~isfile(rosYamlFile)
    error('ROS2Interface:MissingYAML','Missing interface YAML: %s',rosYamlFile);
end
rosYaml = yamlread(rosYamlFile);
rosParams = rosYaml.simulink_ros2.ros__parameters;
cfg.ros2VisionTopic = char(rosParams.vision_topic);
cfg.ros2JointStateTopic = char(rosParams.joint_state_topic);
cfg.ros2FocalStateTopic = char(rosParams.focal_state_topic);
cfg.ros2CameraVelocityTopic = char(rosParams.camera_velocity_topic);
cfg.ros2ZoomVelocityTopic = char(rosParams.zoom_velocity_topic);
cfg.ros2InputTimeoutSec = double(rosParams.input_timeout_sec);
cfg.ros2VisionMessageLength = double(rosParams.vision_message_length);
cfg.ros2FocalMessageLength = double(rosParams.focal_message_length);
cfg.ros2CameraVelocityMessageLength = double(rosParams.camera_velocity_message_length);
cfg.ros2ZoomVelocityMessageLength = double(rosParams.zoom_velocity_message_length);
cfg.ros2InterfaceYamlFile = rosYamlFile;

projectDir = fileparts(mfilename('fullpath'));
fr3 = load_fr3_urdf_parameters(fullfile(projectDir,'fr3.urdf'), ...
    fullfile(projectDir,'fr3_urdf_parse_report.txt'));
cfg.fr3RobotName = char(fr3.robotName);
cfg.fr3OriginXYZ = fr3.originXYZ;
cfg.fr3OriginRPY = fr3.originRPY;
cfg.fr3Axis = fr3.axis;
cfg.fr3Joint8OriginXYZ = fr3.joint8OriginXYZ;
cfg.fr3Joint8OriginRPY = fr3.joint8OriginRPY;
cfg.fr3QMinURDF = fr3.qMin;
cfg.fr3QMaxURDF = fr3.qMax;
cfg.fr3QDotMaxURDF = fr3.qDotMax;

cfg.baseline = 0.12;
cfg.B = cfg.baseline; % compatibility alias; formal stereo geometry uses cfg.baseline
cfg.R_CL_CR = eye(3);
cfg.p_CL_CR = [cfg.baseline;0;0];
cfg.T_CL_CR = [cfg.R_CL_CR cfg.p_CL_CR;0 0 0 1];
cfg.T_W_B = eye(4);
cfg.T_link8_CL = eye(4);
cfg.cameraMountIsPlaceholder = true;
cfg.cameraIntrinsicsArePlaceholder = true;
cfg.imageWidth = 1920;
cfg.imageHeight = 1080;
cfg.imageWidthPx = cfg.imageWidth;
cfg.imageHeightPx = cfg.imageHeight;
cfg.cxL = 960;
cfg.cyL = 540;
cfg.cxR = 960;
cfg.cyR = 540;
cfg.numericalEpsilon = 1e-8;
cfg.visibilityEpsilon = 1e-6;
cfg.rightVisibilityMarginPx = 80;
cfg.rightVisibilityHysteresisPx = 20;
cfg.disparityMin = 1e-4;
cfg.rightReacquireValidSamples = 5;
cfg.usePredictiveRightVisibility = false; % CURRENT_STATE_VISIBILITY_GUARD
cfg.leftOnlyCenterControlEnable = true;

cfg.outputPixelPitchXmm = 2.90e-3;
cfg.outputPixelPitchYmm = 2.90e-3;
cfg.focalMmToPixelsIsPlaceholder = true; % IMX415 2x2-binning assumption, not camera calibration.
cfg.focalLengthHardwareMinMm = [5.2;5.2];
cfg.focalLengthHardwareMaxMm = [99;99];
cfg.focalLengthWorkingMinMm = [10;10];
cfg.focalLengthWorkingMaxMm = [90;90];
if any(cfg.focalLengthHardwareMinMm>cfg.focalLengthWorkingMinMm) || ...
        any(cfg.focalLengthWorkingMinMm>=cfg.focalLengthWorkingMaxMm) || ...
        any(cfg.focalLengthWorkingMaxMm>cfg.focalLengthHardwareMaxMm)
    error('FocalLength:InvalidRanges','Hardware and working focal-length ranges are inconsistent.');
end
cfg.focalRateGuaranteedMmPerSec = 15.5;
cfg.focalRateAbsoluteMaxMmPerSec = 18.75;
cfg.focalRateUnit = 'mm/s';
cfg.etaZoom = 0.60;
cfg.focalRateDesignMmPerSec = cfg.etaZoom*cfg.focalRateGuaranteedMmPerSec;
cfg.rightReacquireZoomRateMmPerSec = cfg.focalRateDesignMmPerSec;
cfg.scaleDesired = [700;700];
cfg.scaleDesignIsProvisional = true;
cfg.focalLength0Mm = [15.225;15.225];
cfg.focalLength0IsPlaceholder = true;
[cfg.f0,~] = focal_mm_to_pixels(cfg.focalLength0Mm,cfg);
cfg.fMin = cfg.focalLengthHardwareMinMm/cfg.outputPixelPitchXmm; % deprecated pixel-domain compatibility
cfg.fMax = cfg.focalLengthHardwareMaxMm/cfg.outputPixelPitchXmm; % deprecated pixel-domain compatibility
cfg.fDotMax = cfg.focalRateDesignMmPerSec/cfg.outputPixelPitchXmm*ones(2,1); % deprecated
cfg.zoomValueMinDeprecated = cfg.focalLengthHardwareMinMm;
cfg.zoomValueMaxDeprecated = cfg.focalLengthHardwareMaxMm;
cfg.zoomRateMaxDeprecated = [12;12];
cfg.zoomCalibrationIsPlaceholder = false;
cfg.zoomRateLimitIsPlaceholder = false;

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
cfg.gainTuningScenarioId = 0;

cfg.targetCharacteristicSize = 0.10;
cfg.scaleNoiseStd = 0;

cfg.targetDepthMin = 0.50;
cfg.targetDepthMax = 1.00;
cfg.Zd = 0.75;
cfg.rhoD = 1/cfg.Zd;
cfg.rhoEstimateMin = 0.80;
cfg.rhoEstimateMax = 2.20;

cfg.q0 = [0;-pi/4;0;-3*pi/4;0;pi/2;pi/4];
cfg.qMin = fr3.qMin;
cfg.qMax = fr3.qMax;
cfg.qMid = 0.5*(cfg.qMin+cfg.qMax);
cfg.qDotMax = fr3.qDotMax;
cfg.qLimitSoftMargin = 5*pi/180;
cfg.cartesianLinearSpeedMax = 2.0;
cfg.jointTorqueMax = fr3.jointTorqueMax;
if ~all(cfg.q0>cfg.qMin) || ~all(cfg.q0<cfg.qMax)
    error('FR3:InitialPoseOutsideLimits','cfg.q0 must lie strictly inside all URDF limits.');
end

cfg.Kc = diag([2.5,2.5]);
cfg.kRho = 1.5;
cfg.Kf = diag([1.5,1.5]);

cfg.lambdaC = 0.02;
cfg.lambdaRho = 0.02;

cfg.betaC = 0;
cfg.betaRho = 0;
cfg.betaF = [0;0];

cfg.epsilonC = 1e-3;
cfg.epsilonRho = 1e-3;
cfg.epsilonF = [1e-3;1e-3];

cfg.robustEnable = 0;
cfg.nullspaceEnable = 0;
cfg.kNull = 0.05;
cfg.armControlEnable = 1;
cfg.centerTaskEnable = 1;
cfg.depthTaskEnable = 1;
cfg.zoomControlEnable = 1;

cfg.pixelNoiseStd = 0.5;
% Deprecated: retained for compatibility with older CV-EKF scripts.
cfg.sigmaAcceleration = 0.5;
cfg.sigmaJerk = 1.0;
cfg.ekfCovarianceJitter = 1e-12;
cfg.ekfSConditionMin = 1e-12;
cfg.randomSeed = 13579;

cfg.experimentMode = 3;

% Define target trajectories in the fixed initial left-camera frame.
[cfg.pWCL0,cfg.RWCL0] = fr3_camera_kinematics(cfg.q0,cfg);
cfg.target0CL = [0.5*cfg.baseline;0;cfg.Zd]; % midpoint between parallel stereo optical centres
cfg.target0 = cfg.pWCL0 + cfg.RWCL0*cfg.target0CL;

cfg.ekfX0 = [
    cfg.target0 + [0.02;-0.01;0.03];
    0;
    0;
    0;
    0;
    0;
    0
];

cfg.ekfP0 = diag([
    0.02^2;
    0.02^2;
    0.05^2;
    0.20^2;
    0.20^2;
    0.20^2;
    0.50^2;
    0.50^2;
    0.50^2
]);

cfg.Rpixel = cfg.pixelNoiseStd^2*eye(4);
fL0 = cfg.f0(1); fR0 = cfg.f0(2); pixelVariance = cfg.pixelNoiseStd^2;
cfg.Rmeasurement = [pixelVariance/(fL0*fL0),0,pixelVariance/(cfg.baseline*fL0*fL0); ...
    0,pixelVariance/(fL0*fL0),0; ...
    pixelVariance/(cfg.baseline*fL0*fL0),0, ...
    pixelVariance/(cfg.baseline^2)*(1/(fL0*fL0)+1/(fR0*fR0))];
cfg.visibilityZMin = 0.10;
cfg.rhoMin = cfg.rhoEstimateMin;
cfg.rhoMax = cfg.rhoEstimateMax;

% Test-only switches. The production default never routes truth into control.
cfg.truthFeedforwardDiagnostic = 0;
cfg.testName = 'default';

if hasCfgOverride
    cfg = cfgOverrideLocal;
end

assignin('base','cfg',cfg);
cfgFieldNames = fieldnames(cfg);
for cfgFieldIndex = 1:numel(cfgFieldNames)
    cfgFieldName = cfgFieldNames{cfgFieldIndex};
    assignin('base', ['cfg_' cfgFieldName], cfg.(cfgFieldName));
end
clear hasCfgOverride cfgOverrideLocal cfgFieldNames cfgFieldIndex cfgFieldName projectDir fr3 fL0 fR0 pixelVariance
