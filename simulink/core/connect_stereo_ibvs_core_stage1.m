function connect_stereo_ibvs_core_stage1(modelName)
% 代码作用：
% 自动完成 stereo_ibvs_core Stage 1 顶层模型的全部连线。
% 本函数只修改顶层连线，不修改任何子系统内部结构。
% 运行前必须保证顶层端口、01~10子系统名称及各端口顺序与设计一致。
%
% 输入参数含义：
% modelName：
% 要连接的Simulink模型名称。可以省略，默认使用
% stereo_ibvs_core。
%
% 输出参数含义：
% 本函数没有输出参数。
% 执行成功后会更新并保存Simulink模型。

if nargin < 1 || isempty(modelName)
    modelName = 'stereo_ibvs_core';
end

modelInput = char(modelName);
[~, modelRoot, extension] = fileparts(modelInput);

if isempty(extension)
    modelRoot = modelInput;
end

% 加载模型。
load_system(modelInput);

% 检查顶层必要模块是否存在。
requiredBlocks = {
    'qRaw'
    'visionFeatureRaw'
    'zoomPositionStepsRaw'
    'controllerEnableRaw'
    'resetRaw'
    '01_Input_Validity'
    '02_Camera_Model'
    '03_Feature_Processing'
    '04_FR3_Camera_Kinematics'
    '05_Target_EKF'
    '06_Inverse_Depth_Filter'
    '07_Arm_Priority_Controller'
    '08_Zoom_Controller'
    '09_Safety_Supervisor'
    '10_Diagnostics'
    'jointVelocityCmd'
    'zoomStepRateCmd'
    'controllerStatus'
    'systemReady'
    'faultCode'
};

for i = 1:numel(requiredBlocks)
    blockPath = [modelRoot '/' requiredBlocks{i}];

    if getSimulinkBlockHandle(blockPath) == -1
        error('Stage1:MissingBlock', ...
            '顶层缺少模块：%s', blockPath);
    end
end

% 只删除顶层已有连线，不影响任何子系统内部连线。
topLevelLines = find_system( ...
    modelRoot, ...
    'FindAll', 'on', ...
    'SearchDepth', 1, ...
    'Type', 'line');

for i = 1:numel(topLevelLines)
    try
        delete_line(topLevelLines(i));
    catch
        % 某些支路线会随着主线删除而自动消失，因此这里忽略重复删除。
    end
end

%% 创建Stage 1顶层辅助模块

% 手眼标定完成标志。
ensureBlock( ...
    modelRoot, ...
    'simulink/Sources/Constant', ...
    'Camera Mount Calibrated', ...
    [1450 900 1640 935]);

set_param( ...
    [modelRoot '/Camera Mount Calibrated'], ...
    'Value', 'double(cfg.cameraMountCalibrated)', ...
    'SampleTime', '-1');

% Stage 1暂时未使用的输出连接到Terminator。
terminatorNames = {
    'Term_02_focalLengthPx'
    'Term_05_pHatB'
    'Term_05_vHatB'
    'Term_05_aHatB'
    'Term_05_vHatCL'
    'Term_06_disparityNormalized'
    'Term_07_centerError'
    'Term_08_scaleError'
    'Term_09_jointLimitWarning'
    'Term_09_velocitySaturationFlag'
};

terminatorPositions = [
    900 180 920 200
    1210 330 1230 350
    1210 370 1230 390
    1210 410 1230 430
    1210 450 1230 470
    1210 560 1230 580
    1450 640 1470 660
    1450 760 1470 780
    1780 720 1800 740
    1780 760 1800 780
];

for i = 1:numel(terminatorNames)
    ensureBlock( ...
        modelRoot, ...
        'simulink/Sinks/Terminator', ...
        terminatorNames{i}, ...
        terminatorPositions(i,:));
end

%% 顶层输入连接到01_Input_Validity

connectPorts(modelRoot, 'qRaw', 1, ...
    '01_Input_Validity', 1, 'qRaw');

connectPorts(modelRoot, 'visionFeatureRaw', 1, ...
    '01_Input_Validity', 2, 'visionFeatureRaw');

connectPorts(modelRoot, 'zoomPositionStepsRaw', 1, ...
    '01_Input_Validity', 3, 'zoomPositionStepsRaw');

connectPorts(modelRoot, 'controllerEnableRaw', 1, ...
    '01_Input_Validity', 4, 'controllerEnableRaw');

connectPorts(modelRoot, 'resetRaw', 1, ...
    '01_Input_Validity', 5, 'resetRaw');

%% 01_Input_Validity输出连接

connectPorts(modelRoot, '01_Input_Validity', 1, ...
    '04_FR3_Camera_Kinematics', 1, 'q');

connectPorts(modelRoot, '01_Input_Validity', 1, ...
    '09_Safety_Supervisor', 2, 'q');

connectPorts(modelRoot, '01_Input_Validity', 2, ...
    '03_Feature_Processing', 1, 'visionFeature');

connectPorts(modelRoot, '01_Input_Validity', 3, ...
    '02_Camera_Model', 1, 'zoomPositionSteps');

connectPorts(modelRoot, '01_Input_Validity', 3, ...
    '08_Zoom_Controller', 1, 'zoomPositionSteps');

connectPorts(modelRoot, '01_Input_Validity', 4, ...
    '07_Arm_Priority_Controller', 6, 'controllerEnableSafe');

connectPorts(modelRoot, '01_Input_Validity', 4, ...
    '08_Zoom_Controller', 6, 'controllerEnableSafe');

connectPorts(modelRoot, '01_Input_Validity', 4, ...
    '09_Safety_Supervisor', 4, 'controllerEnableSafe');

connectPorts(modelRoot, '01_Input_Validity', 4, ...
    '10_Diagnostics', 11, 'controllerEnableSafe');

connectPorts(modelRoot, '01_Input_Validity', 5, ...
    '05_Target_EKF', 5, 'resetSafe');

connectPorts(modelRoot, '01_Input_Validity', 5, ...
    '06_Inverse_Depth_Filter', 5, 'resetSafe');

connectPorts(modelRoot, '01_Input_Validity', 5, ...
    '08_Zoom_Controller', 7, 'resetSafe');

connectPorts(modelRoot, '01_Input_Validity', 6, ...
    '10_Diagnostics', 1, 'inputDataValid');

%% 02_Camera_Model输出连接

connectPorts(modelRoot, '02_Camera_Model', 1, ...
    '03_Feature_Processing', 2, 'cameraIntrinsics');

connectPorts(modelRoot, '02_Camera_Model', 2, ...
    'Term_02_focalLengthPx', 1, 'focalLengthPx');

connectPorts(modelRoot, '02_Camera_Model', 3, ...
    '07_Arm_Priority_Controller', 4, 'cameraModelValid');

connectPorts(modelRoot, '02_Camera_Model', 3, ...
    '10_Diagnostics', 2, 'cameraModelValid');

%% 03_Feature_Processing输出连接

connectPorts(modelRoot, '03_Feature_Processing', 1, ...
    '05_Target_EKF', 1, 'cLMeasured');

connectPorts(modelRoot, '03_Feature_Processing', 1, ...
    '06_Inverse_Depth_Filter', 1, 'cLMeasured');

connectPorts(modelRoot, '03_Feature_Processing', 1, ...
    '07_Arm_Priority_Controller', 1, 'cLMeasured');

connectPorts(modelRoot, '03_Feature_Processing', 2, ...
    '06_Inverse_Depth_Filter', 2, 'cRMeasured');

connectPorts(modelRoot, '03_Feature_Processing', 3, ...
    '08_Zoom_Controller', 2, 'scaleMeasured');

connectPorts(modelRoot, '03_Feature_Processing', 4, ...
    '06_Inverse_Depth_Filter', 3, 'validLeft');

connectPorts(modelRoot, '03_Feature_Processing', 4, ...
    '07_Arm_Priority_Controller', 3, 'validLeft');

connectPorts(modelRoot, '03_Feature_Processing', 4, ...
    '08_Zoom_Controller', 4, 'validLeft');

connectPorts(modelRoot, '03_Feature_Processing', 4, ...
    '10_Diagnostics', 4, 'validLeft');

connectPorts(modelRoot, '03_Feature_Processing', 5, ...
    '06_Inverse_Depth_Filter', 4, 'validRight');

connectPorts(modelRoot, '03_Feature_Processing', 5, ...
    '08_Zoom_Controller', 5, 'validRight');

connectPorts(modelRoot, '03_Feature_Processing', 5, ...
    '10_Diagnostics', 5, 'validRight');

%% 04_FR3_Camera_Kinematics输出连接

connectPorts(modelRoot, '04_FR3_Camera_Kinematics', 1, ...
    '07_Arm_Priority_Controller', 2, 'JL');

connectPorts(modelRoot, '04_FR3_Camera_Kinematics', 2, ...
    '05_Target_EKF', 4, 'T_CL2B');

connectPorts(modelRoot, '04_FR3_Camera_Kinematics', 3, ...
    '07_Arm_Priority_Controller', 5, 'kinematicsValid');

connectPorts(modelRoot, '04_FR3_Camera_Kinematics', 3, ...
    '10_Diagnostics', 3, 'kinematicsValid');

%% 05_Target_EKF输出连接

connectPorts(modelRoot, '05_Target_EKF', 1, ...
    'Term_05_pHatB', 1, 'pHatB');

connectPorts(modelRoot, '05_Target_EKF', 2, ...
    'Term_05_vHatB', 1, 'vHatB');

connectPorts(modelRoot, '05_Target_EKF', 3, ...
    'Term_05_aHatB', 1, 'aHatB');

connectPorts(modelRoot, '05_Target_EKF', 4, ...
    'Term_05_vHatCL', 1, 'vHatCL');

connectPorts(modelRoot, '05_Target_EKF', 5, ...
    '10_Diagnostics', 9, 'ekfValid');

%% 06_Inverse_Depth_Filter输出连接

connectPorts(modelRoot, '06_Inverse_Depth_Filter', 1, ...
    '07_Arm_Priority_Controller', 7, 'rhoForControl');

connectPorts(modelRoot, '06_Inverse_Depth_Filter', 1, ...
    '08_Zoom_Controller', 3, 'rhoForControl');

connectPorts(modelRoot, '06_Inverse_Depth_Filter', 2, ...
    '05_Target_EKF', 2, 'rhoMeasured');

connectPorts(modelRoot, '06_Inverse_Depth_Filter', 3, ...
    'Term_06_disparityNormalized', 1, 'disparityNormalized');

connectPorts(modelRoot, '06_Inverse_Depth_Filter', 4, ...
    '05_Target_EKF', 3, 'depthMeasurementValid');

connectPorts(modelRoot, '06_Inverse_Depth_Filter', 4, ...
    '10_Diagnostics', 6, 'depthMeasurementValid');

%% 07_Arm_Priority_Controller输出连接

connectPorts(modelRoot, '07_Arm_Priority_Controller', 1, ...
    '09_Safety_Supervisor', 1, 'qDotCenter');

connectPorts(modelRoot, '07_Arm_Priority_Controller', 2, ...
    'Term_07_centerError', 1, 'centerError');

connectPorts(modelRoot, '07_Arm_Priority_Controller', 3, ...
    '09_Safety_Supervisor', 3, 'centerTaskValid');

connectPorts(modelRoot, '07_Arm_Priority_Controller', 3, ...
    '10_Diagnostics', 7, 'centerTaskValid');

connectPorts(modelRoot, '07_Arm_Priority_Controller', 4, ...
    '10_Diagnostics', 12, 'jConditionMetric');

%% 08_Zoom_Controller输出连接

connectPorts(modelRoot, '08_Zoom_Controller', 1, ...
    'zoomStepRateCmd', 1, 'zoomStepRateCmd');

connectPorts(modelRoot, '08_Zoom_Controller', 2, ...
    'Term_08_scaleError', 1, 'scaleError');

connectPorts(modelRoot, '08_Zoom_Controller', 3, ...
    '10_Diagnostics', 10, 'zoomControllerValid');

%% 09_Safety_Supervisor输出连接

connectPorts(modelRoot, 'Camera Mount Calibrated', 1, ...
    '09_Safety_Supervisor', 5, 'cameraMountCalibrated');

connectPorts(modelRoot, '09_Safety_Supervisor', 1, ...
    'jointVelocityCmd', 1, 'jointVelocityCmd');

connectPorts(modelRoot, '09_Safety_Supervisor', 2, ...
    'Term_09_jointLimitWarning', 1, 'jointLimitWarning');

connectPorts(modelRoot, '09_Safety_Supervisor', 3, ...
    'Term_09_velocitySaturationFlag', 1, ...
    'velocitySaturationFlag');

connectPorts(modelRoot, '09_Safety_Supervisor', 4, ...
    '10_Diagnostics', 8, 'safetyValid');

%% 10_Diagnostics输出连接

connectPorts(modelRoot, '10_Diagnostics', 1, ...
    'controllerStatus', 1, 'controllerStatus');

connectPorts(modelRoot, '10_Diagnostics', 2, ...
    'systemReady', 1, 'systemReady');

connectPorts(modelRoot, '10_Diagnostics', 3, ...
    'faultCode', 1, 'faultCode');

%% 更新并保存模型

set_param(modelRoot, 'SimulationCommand', 'update');
save_system(modelRoot);

fprintf('\nStage 1顶层连线完成：%s\n', modelRoot);
fprintf('注意：cfg.cameraMountCalibrated=false时，jointVelocityCmd会保持为零。\n');

end


function ensureBlock(modelRoot, libraryBlock, blockName, position)
% 代码作用：
% 如果顶层辅助模块不存在，则创建该模块；如果已经存在，则保留。

blockPath = [modelRoot '/' blockName];

if getSimulinkBlockHandle(blockPath) == -1
    add_block( ...
        libraryBlock, ...
        blockPath, ...
        'Position', position);
end
end


function connectPorts(modelRoot,sourceBlock,sourcePort, ...
    destinationBlock,destinationPort,signalName)
% 代码作用：
% 按模块端口句柄连接两个顶层模块，并检查端口号是否存在。
%
% 输入参数含义：
% modelRoot：
% 顶层模型名称。
%
% sourceBlock、destinationBlock：
% 源模块和目标模块在顶层模型中的名称。
%
% sourcePort、destinationPort：
% 源模块输出端口号和目标模块输入端口号。
%
% signalName：
% 需要显示在线上的信号名称。
%
% 输出参数含义：
% 本函数没有输出参数。

sourcePath = [modelRoot '/' sourceBlock];
destinationPath = [modelRoot '/' destinationBlock];

sourceHandles = get_param(sourcePath,'PortHandles');
destinationHandles = get_param(destinationPath,'PortHandles');

if numel(sourceHandles.Outport) < sourcePort
    error('Stage1:InvalidSourcePort', ...
        '模块%s不存在输出端口%d。', ...
        sourcePath,sourcePort);
end

if numel(destinationHandles.Inport) < destinationPort
    error('Stage1:InvalidDestinationPort', ...
        '模块%s不存在输入端口%d。', ...
        destinationPath,destinationPort);
end

sourceHandle = sourceHandles.Outport(sourcePort);
destinationHandle = destinationHandles.Inport(destinationPort);

% 如果目标端口已经被连接，先删除该目标端口上的旧线。
oldLine = get_param(destinationHandle,'Line');

if oldLine ~= -1
    delete_line(oldLine);
end

add_line( ...
    modelRoot, ...
    sourceHandle, ...
    destinationHandle, ...
    'autorouting', 'on');

% 设置线名。分支信号可能已经有同名主线，因此这里容错处理。
newLine = get_param(destinationHandle,'Line');

if newLine ~= -1 && ~isempty(signalName)
    try
        set_param(newLine,'Name',signalName);
    catch
        % 不影响连线结果。
    end
end
end
