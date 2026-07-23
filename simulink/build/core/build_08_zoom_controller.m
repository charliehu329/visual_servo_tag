function build_08_zoom_controller()
%BUILD_08_ZOOM_CONTROLLER
% 重建 stereo_ibvs_core/08_Zoom_Controller。
%
% 结构：
%   08A Right Visibility Guard
%   08B_Zoom_Controller_Mm
%   08C_Zoom_Priority_Supervisor
%
% 约定：
% 1. 动态运行信号使用子系统输入端口；
% 2. 所有配置使用 Constant 块，Value 采用 cfg.xxx；
% 3. Core 内焦距单位为 mm，焦距速度单位为 mm/s；
% 4. 本脚本连接 08A、08B、08C 的内部线路；
% 5. 本脚本不连接 Core 顶层其他模块；
% 6. 所有子系统背景均为白色。

%% 路径
buildDir = fileparts(mfilename('fullpath'));
simulinkDir = fileparts(fileparts(buildDir));
modelFile = fullfile(simulinkDir, 'core', 'stereo_ibvs_core.slx');

if ~isfile(modelFile)
    error('StereoIBVS:CoreModelNotFound', ...
        '找不到 Core 模型：%s', modelFile);
end

[~, modelName] = fileparts(modelFile);
load_system(modelFile);

subsystemPath = [modelName '/08_Zoom_Controller'];

%% 可选配置字段检查
requiredFields = {
    'scaleDesired'
    'numericalEpsilon'
    'baseline'
    'outputPixelPitchXmm'
    'cxR'
    'focalLengthHardwareMaxMm'
    'visibilityEpsilon'
    'imageWidthPx'
    'rightVisibilityMarginPx'
    'focalLengthWorkingMaxMm'
    'focalLengthWorkingMinMm'
    'targetCharacteristicSize'
    'rightVisibilityHysteresisPx'
    'robustEnable'
    'betaF'
    'epsilonF'
    'Kf'
    'rightReacquireZoomRateMmPerSec'
    'focalRateDesignMmPerSec'
    'Ts'
    'focalRateGuaranteedMmPerSec'
    'focalRateAbsoluteMaxMmPerSec'
    'zoomControlEnable'
    'zoomPriorityEnable'
    'armDepthRampTime'
    'scaleErrorEnterThreshold'
    'scaleErrorExitThreshold'
    'scaleSettledHoldTime'
    'disturbanceConfirmTime'
    'zoomOnlyMaxTime'
};

warnMissingConfigurationFields(requiredFields);

%% 删除旧08并重新创建
if getSimulinkBlockHandle(subsystemPath) ~= -1
    delete_block(subsystemPath);
end

add_block( ...
    'simulink/Ports & Subsystems/Subsystem', ...
    subsystemPath, ...
    'Position', [1650 90 1990 900], ...
    'BackgroundColor', 'white');

deleteIfExists([subsystemPath '/In1']);
deleteIfExists([subsystemPath '/Out1']);

%% 08顶层动态输入
topInputs = {
    'xLeftMeasured',                 1,  '1'
    'rhoHatSafe',                    2,  '1'
    'focalLengthMeasuredMm',         3,  '[2 1]'
    'focalLengthDataValid',          4,  '1'
    'cameraModelValid',              5,  '1'
    'validLeft',                     6,  '1'
    'validRight',                    7,  '1'
    'ekfPredictionValid',            8,  '1'
    'rightReacquireActive',          9,  '1'
    'resetSafe',                    10,  '1'
    'scaleMeasured',                11,  '[2 1]'
    'rhoDotHat',                    12,  '[2 1]'
    'focalLengthFresh',             13,  '1'
    'focalAtWorkingLowerLimit',     14,  '[2 1]'
    'focalAtWorkingUpperLimit',     15,  '[2 1]'
    'depthErrorDelayed',            16,  '1'
    'focalHeadroomMm',              17,  '[2 1]'
    'validStereoQualified',         18,  '1'
};

for inputIndex = 1:size(topInputs,1)
    y = 35 + 42*(inputIndex-1);
    addInport( ...
        subsystemPath, ...
        topInputs{inputIndex,1}, ...
        topInputs{inputIndex,2}, ...
        topInputs{inputIndex,3}, ...
        [25 y 55 y+14]);
end

%% 创建三个子系统
pathA = [subsystemPath '/08A Right Visibility Guard'];
pathB = [subsystemPath '/08B_Zoom_Controller_Mm'];
pathC = [subsystemPath '/08C_Zoom_Priority_Supervisor'];

build08A(pathA);
build08B(pathB);
build08C(pathC);

set_param(pathA, 'Position', [245 35 510 455]);
set_param(pathB, 'Position', [245 500 510 950]);
set_param(pathC, 'Position', [245 995 510 1445]);

%% 08顶层输出：按08A、08B、08C顺序
topOutputs = {
    'uRightPredicted',                         1,  '1'
    'rightVisibilityMarginActualPx',           2,  '1'
    'fRightMaxVisibleMm',                      3,  '1'
    'fRightMaxEffectiveMm',                    4,  '1'
    'rightScaleDesiredEffective',              5,  '1'
    'rightVisibilityActive',                   6,  '1'
    'rightVisibilityInfeasible',               7,  '1'
    'scaleDesiredEffective',                   8,  '[2 1]'
    'rightScaleDesiredNominal',                9,  '1'
    'fRightMeasuredMm',                       10,  '1'
    'fDotMmCmd',                              11,  '[2 1]'
    'fDotMmLimited',                          12,  '[2 1]'
    'scaleError',                             13,  '[2 1]'
    'gDotCmd',                                14,  '[2 1]'
    'focalCommandOutwardAtWorkingLimit',      15,  '[2 1]'
    'focalFeasible',                          16,  '[2 1]'
    'focalRateUsageGuaranteedCmd',            17,  '[2 1]'
    'focalRateUsageHardCmd',                  18,  '[2 1]'
    'depthTaskWeight',                        19,  '1'
    'schedulerMode',                          20,  '1'
    'zoomPriorityActive',                     21,  '1'
    'zoomPriorityTimer',                      22,  '1'
    'scaleSettledTimer',                      23,  '1'
    'armRecoveryReason',                      24,  '1'
    'newDisturbanceDetected',                 25,  '1'
};

for outputIndex = 1:size(topOutputs,1)
    y = 30 + 44*(outputIndex-1);
    addOutport( ...
        subsystemPath, ...
        topOutputs{outputIndex,1}, ...
        topOutputs{outputIndex,2}, ...
        topOutputs{outputIndex,3}, ...
        [660 y 690 y+14]);
end

%% 顶层输入 -> 08A
connect(subsystemPath, 'xLeftMeasured/1',             '08A Right Visibility Guard/1');
connect(subsystemPath, 'rhoHatSafe/1',                '08A Right Visibility Guard/2');
connect(subsystemPath, 'focalLengthMeasuredMm/1',     '08A Right Visibility Guard/3');
connect(subsystemPath, 'focalLengthDataValid/1',      '08A Right Visibility Guard/4');
connect(subsystemPath, 'cameraModelValid/1',          '08A Right Visibility Guard/5');
connect(subsystemPath, 'validLeft/1',                 '08A Right Visibility Guard/6');
connect(subsystemPath, 'validRight/1',                '08A Right Visibility Guard/7');
connect(subsystemPath, 'ekfPredictionValid/1',        '08A Right Visibility Guard/8');
connect(subsystemPath, 'rightReacquireActive/1',      '08A Right Visibility Guard/9');
connect(subsystemPath, 'resetSafe/1',                 '08A Right Visibility Guard/10');

%% 顶层输入和08A输出 -> 08B
connect(subsystemPath, 'focalLengthMeasuredMm/1',     '08B_Zoom_Controller_Mm/1');
connect(subsystemPath, 'focalLengthFresh/1',          '08B_Zoom_Controller_Mm/2');
connect(subsystemPath, 'focalLengthDataValid/1',      '08B_Zoom_Controller_Mm/3');
connect(subsystemPath, 'cameraModelValid/1',          '08B_Zoom_Controller_Mm/4');
connect(subsystemPath, 'scaleMeasured/1',             '08B_Zoom_Controller_Mm/5');
connect(subsystemPath, 'rhoHatSafe/1',                '08B_Zoom_Controller_Mm/6');
connect(subsystemPath, 'rhoDotHat/1',                 '08B_Zoom_Controller_Mm/7');
connect(subsystemPath, '08A Right Visibility Guard/8','08B_Zoom_Controller_Mm/8');
connect(subsystemPath, 'validLeft/1',                 '08B_Zoom_Controller_Mm/9');
connect(subsystemPath, 'ekfPredictionValid/1',        '08B_Zoom_Controller_Mm/10');
connect(subsystemPath, 'focalAtWorkingLowerLimit/1', '08B_Zoom_Controller_Mm/11');
connect(subsystemPath, 'focalAtWorkingUpperLimit/1', '08B_Zoom_Controller_Mm/12');
connect(subsystemPath, '08A Right Visibility Guard/4','08B_Zoom_Controller_Mm/13');
connect(subsystemPath, '08A Right Visibility Guard/6','08B_Zoom_Controller_Mm/14');
connect(subsystemPath, 'rightReacquireActive/1',      '08B_Zoom_Controller_Mm/15');

%% 顶层输入、08B输出 -> 08C
connect(subsystemPath, '08B_Zoom_Controller_Mm/3',    '08C_Zoom_Priority_Supervisor/1');
connect(subsystemPath, 'depthErrorDelayed/1',         '08C_Zoom_Priority_Supervisor/2');
connect(subsystemPath, 'focalLengthMeasuredMm/1',     '08C_Zoom_Priority_Supervisor/3');
connect(subsystemPath, 'focalLengthFresh/1',          '08C_Zoom_Priority_Supervisor/4');
connect(subsystemPath, 'focalLengthDataValid/1',      '08C_Zoom_Priority_Supervisor/5');
connect(subsystemPath, 'cameraModelValid/1',          '08C_Zoom_Priority_Supervisor/6');
connect(subsystemPath, 'focalAtWorkingLowerLimit/1', '08C_Zoom_Priority_Supervisor/7');
connect(subsystemPath, 'focalAtWorkingUpperLimit/1', '08C_Zoom_Priority_Supervisor/8');
connect(subsystemPath, 'focalHeadroomMm/1',           '08C_Zoom_Priority_Supervisor/9');
connect(subsystemPath, '08B_Zoom_Controller_Mm/6',    '08C_Zoom_Priority_Supervisor/10');
connect(subsystemPath, 'validLeft/1',                 '08C_Zoom_Priority_Supervisor/11');
connect(subsystemPath, 'validStereoQualified/1',      '08C_Zoom_Priority_Supervisor/12');
connect(subsystemPath, 'ekfPredictionValid/1',        '08C_Zoom_Priority_Supervisor/13');
connect(subsystemPath, 'resetSafe/1',                 '08C_Zoom_Priority_Supervisor/14');
connect(subsystemPath, '08B_Zoom_Controller_Mm/5',    '08C_Zoom_Priority_Supervisor/15');
connect(subsystemPath, '08B_Zoom_Controller_Mm/7',    '08C_Zoom_Priority_Supervisor/16');
connect(subsystemPath, '08B_Zoom_Controller_Mm/8',    '08C_Zoom_Priority_Supervisor/17');
connect(subsystemPath, 'rightReacquireActive/1',      '08C_Zoom_Priority_Supervisor/18');

%% 08A输出 -> 顶层输出1~10
for outputIndex = 1:10
    connect( ...
        subsystemPath, ...
        sprintf('08A Right Visibility Guard/%d', outputIndex), ...
        [topOutputs{outputIndex,1} '/1']);
end

%% 08B输出 -> 顶层输出11~18
for localOutputIndex = 1:8
    topOutputIndex = 10 + localOutputIndex;
    connect( ...
        subsystemPath, ...
        sprintf('08B_Zoom_Controller_Mm/%d', localOutputIndex), ...
        [topOutputs{topOutputIndex,1} '/1']);
end

%% 08C输出 -> 顶层输出19~25
for localOutputIndex = 1:7
    topOutputIndex = 18 + localOutputIndex;
    connect( ...
        subsystemPath, ...
        sprintf('08C_Zoom_Priority_Supervisor/%d', localOutputIndex), ...
        [topOutputs{topOutputIndex,1} '/1']);
end

%% 自动整理，仅影响08内部显示
try
    Simulink.BlockDiagram.arrangeSystem(pathA);
    Simulink.BlockDiagram.arrangeSystem(pathB);
    Simulink.BlockDiagram.arrangeSystem(pathC);
    Simulink.BlockDiagram.arrangeSystem(subsystemPath);
catch arrangeError
    warning('StereoIBVS:ArrangeSystemFailed', ...
        '08已创建，但自动布局失败：%s', arrangeError.message);
end

%% 保存
save_system(modelName);

fprintf('\n08_Zoom_Controller 重建完成。\n');
fprintf('模型：%s\n', modelFile);
fprintf('动态输入：18个。\n');
fprintf('诊断和控制输出：25个。\n');
fprintf('配置方式：Constant(Value = cfg.xxx)。\n');
fprintf('注意：没有连接 Core 顶层其他模块。\n');
end


function build08A(subsystemPath)
addCleanSubsystem(subsystemPath, [0 0 260 420]);

runtimeInputs = {
    'xLeftMeasured',              1,  '1'
    'rhoHatSafe',                 2,  '1'
    'focalLengthMeasuredMm',      3,  '[2 1]'
    'focalLengthDataValid',       4,  '1'
    'cameraModelValid',           5,  '1'
    'validLeft',                  6,  '1'
    'validRight',                 7,  '1'
    'ekfPredictionValid',         8,  '1'
    'rightReacquireActive',       9,  '1'
    'resetSafe',                 10,  '1'
};

constants = {
    'Scale Desired',                     'cfg.scaleDesired'
    'Numerical Epsilon',                 'cfg.numericalEpsilon'
    'Stereo Baseline',                   'cfg.baseline'
    'Output Pixel Pitch X Mm',           'cfg.outputPixelPitchXmm'
    'Right Principal Point X',           'cfg.cxR'
    'Hardware Focal Maximum Mm',         'cfg.focalLengthHardwareMaxMm'
    'Visibility Epsilon',                'cfg.visibilityEpsilon'
    'Image Width Px',                    'cfg.imageWidthPx'
    'Right Visibility Margin Px',        'cfg.rightVisibilityMarginPx'
    'Working Focal Maximum Mm',          'cfg.focalLengthWorkingMaxMm'
    'Working Focal Minimum Mm',          'cfg.focalLengthWorkingMinMm'
    'Target Characteristic Size',        'cfg.targetCharacteristicSize'
    'Right Visibility Hysteresis Px',    'cfg.rightVisibilityHysteresisPx'
};

outputs = {
    'uRightPredicted',                    1,  '1'
    'rightVisibilityMarginActualPx',      2,  '1'
    'fRightMaxVisibleMm',                 3,  '1'
    'fRightMaxEffectiveMm',               4,  '1'
    'rightScaleDesiredEffective',         5,  '1'
    'rightVisibilityActive',              6,  '1'
    'rightVisibilityInfeasible',          7,  '1'
    'scaleDesiredEffective',              8,  '[2 1]'
    'rightScaleDesiredNominal',           9,  '1'
    'fRightMeasuredMm',                  10,  '1'
};

populateFunctionSubsystem( ...
    subsystemPath, ...
    'Right Visibility Guard', ...
    visibilityGuardCode(), ...
    runtimeInputs, ...
    constants, ...
    outputs);
end


function build08B(subsystemPath)
addCleanSubsystem(subsystemPath, [0 0 260 450]);

runtimeInputs = {
    'focalLengthMeasuredMm',             1,  '[2 1]'
    'focalLengthFresh',                  2,  '1'
    'focalLengthDataValid',              3,  '1'
    'cameraModelValid',                  4,  '1'
    'scaleMeasured',                     5,  '[2 1]'
    'rhoHatSafe',                        6,  '1'
    'rhoDotHat',                         7,  '[2 1]'
    'scaleDesiredEffective',             8,  '[2 1]'
    'validLeft',                         9,  '1'
    'ekfPredictionValid',               10,  '1'
    'focalAtWorkingLowerLimit',         11,  '[2 1]'
    'focalAtWorkingUpperLimit',         12,  '[2 1]'
    'fRightMaxEffectiveMm',             13,  '1'
    'rightVisibilityActive',            14,  '1'
    'rightReacquireActive',             15,  '1'
};

constants = {
    'Numerical Epsilon',                 'cfg.numericalEpsilon'
    'Working Focal Maximum Mm',          'cfg.focalLengthWorkingMaxMm'
    'Robust Enable',                     'cfg.robustEnable'
    'Zoom Robust Beta',                  'cfg.betaF'
    'Zoom Robust Epsilon',               'cfg.epsilonF'
    'Zoom Gain Kf',                      'cfg.Kf'
    'Target Characteristic Size',        'cfg.targetCharacteristicSize'
    'Output Pixel Pitch X Mm',           'cfg.outputPixelPitchXmm'
    'Working Focal Minimum Mm',          'cfg.focalLengthWorkingMinMm'
    'Right Reacquire Zoom Rate',         'cfg.rightReacquireZoomRateMmPerSec'
    'Design Focal Rate',                 'cfg.focalRateDesignMmPerSec'
    'Sample Time',                       'cfg.Ts'
    'Guaranteed Focal Rate',             'cfg.focalRateGuaranteedMmPerSec'
    'Absolute Focal Rate Maximum',       'cfg.focalRateAbsoluteMaxMmPerSec'
    'Zoom Control Enable',               'cfg.zoomControlEnable'
};

outputs = {
    'fDotMmCmd',                         1,  '[2 1]'
    'fDotMmLimited',                     2,  '[2 1]'
    'scaleError',                        3,  '[2 1]'
    'gDotCmd',                           4,  '[2 1]'
    'focalCommandOutwardAtWorkingLimit', 5,  '[2 1]'
    'focalFeasible',                     6,  '[2 1]'
    'focalRateUsageGuaranteedCmd',       7,  '[2 1]'
    'focalRateUsageHardCmd',             8,  '[2 1]'
};

populateFunctionSubsystem( ...
    subsystemPath, ...
    'V2 Zoom Controller', ...
    zoomControllerCode(), ...
    runtimeInputs, ...
    constants, ...
    outputs);
end


function build08C(subsystemPath)
addCleanSubsystem(subsystemPath, [0 0 260 450]);

runtimeInputs = {
    'scaleError',                         1,  '[2 1]'
    'depthErrorDelayed',                  2,  '1'
    'focalLengthMeasuredMm',              3,  '[2 1]'
    'focalLengthFresh',                   4,  '1'
    'focalLengthDataValid',               5,  '1'
    'cameraModelValid',                   6,  '1'
    'focalAtWorkingLowerLimit',           7,  '[2 1]'
    'focalAtWorkingUpperLimit',           8,  '[2 1]'
    'focalHeadroomMm',                    9,  '[2 1]'
    'focalFeasible',                     10,  '[2 1]'
    'validLeft',                         11,  '1'
    'validStereoQualified',              12,  '1'
    'ekfPredictionValid',                13,  '1'
    'resetSafe',                         14,  '1'
    'focalCommandOutwardAtWorkingLimit', 15,  '[2 1]'
    'focalRateUsageGuaranteedCmd',       16,  '[2 1]'
    'focalRateUsageHardCmd',             17,  '[2 1]'
    'rightReacquireActive',              18,  '1'
};

constants = {
    'Sample Time',                       'cfg.Ts'
    'Zoom Priority Enable',              'cfg.zoomPriorityEnable'
    'Arm Depth Ramp Time',               'cfg.armDepthRampTime'
    'Scale Error Enter Threshold',       'cfg.scaleErrorEnterThreshold'
    'Scale Error Exit Threshold',        'cfg.scaleErrorExitThreshold'
    'Scale Settled Hold Time',           'cfg.scaleSettledHoldTime'
    'Disturbance Confirm Time',           'cfg.disturbanceConfirmTime'
    'Zoom Only Maximum Time',             'cfg.zoomOnlyMaxTime'
};

outputs = {
    'depthTaskWeight',                    1,  '1'
    'schedulerMode',                      2,  '1'
    'zoomPriorityActive',                 3,  '1'
    'zoomPriorityTimer',                  4,  '1'
    'scaleSettledTimer',                  5,  '1'
    'armRecoveryReason',                  6,  '1'
    'newDisturbanceDetected',             7,  '1'
};

populateFunctionSubsystem( ...
    subsystemPath, ...
    'V2 Zoom Priority Supervisor', ...
    zoomPrioritySupervisorCode(), ...
    runtimeInputs, ...
    constants, ...
    outputs);
end


function populateFunctionSubsystem( ...
        subsystemPath, functionName, functionCode, ...
        runtimeInputs, constants, outputs)

for inputIndex = 1:size(runtimeInputs,1)
    y = 35 + 34*(inputIndex-1);
    addInport( ...
        subsystemPath, ...
        runtimeInputs{inputIndex,1}, ...
        runtimeInputs{inputIndex,2}, ...
        runtimeInputs{inputIndex,3}, ...
        [20 y 50 y+14]);
end

constantStartY = 35 + 34*size(runtimeInputs,1) + 20;

for constantIndex = 1:size(constants,1)
    y = constantStartY + 34*(constantIndex-1);
    add_block( ...
        'simulink/Sources/Constant', ...
        [subsystemPath '/' constants{constantIndex,1}], ...
        'Value', constants{constantIndex,2}, ...
        'OutDataTypeStr', 'double', ...
        'SampleTime', '-1', ...
        'Position', [20 y 150 y+24]);
end

numberOfInputs = size(runtimeInputs,1) + size(constants,1);
functionHeight = max(260, 28*numberOfInputs);

functionBlockPath = [subsystemPath '/' functionName];

add_block( ...
    'simulink/User-Defined Functions/MATLAB Function', ...
    functionBlockPath, ...
    'Position', [245 25 600 25+functionHeight]);

chart = find( ...
    sfroot, ...
    '-isa', 'Stateflow.EMChart', ...
    'Path', functionBlockPath);

if isempty(chart)
    error('StereoIBVS:MATLABFunctionNotFound', ...
        '无法找到 MATLAB Function：%s', functionBlockPath);
end

chart.Script = functionCode;

for outputIndex = 1:size(outputs,1)
    y = 35 + 44*(outputIndex-1);
    addOutport( ...
        subsystemPath, ...
        outputs{outputIndex,1}, ...
        outputs{outputIndex,2}, ...
        outputs{outputIndex,3}, ...
        [700 y 730 y+14]);
end

for inputIndex = 1:size(runtimeInputs,1)
    connect( ...
        subsystemPath, ...
        [runtimeInputs{inputIndex,1} '/1'], ...
        sprintf('%s/%d', functionName, inputIndex));
end

for constantIndex = 1:size(constants,1)
    functionInputIndex = size(runtimeInputs,1) + constantIndex;
    connect( ...
        subsystemPath, ...
        [constants{constantIndex,1} '/1'], ...
        sprintf('%s/%d', functionName, functionInputIndex));
end

for outputIndex = 1:size(outputs,1)
    connect( ...
        subsystemPath, ...
        sprintf('%s/%d', functionName, outputIndex), ...
        [outputs{outputIndex,1} '/1']);
end
end


function addCleanSubsystem(subsystemPath, position)
add_block( ...
    'simulink/Ports & Subsystems/Subsystem', ...
    subsystemPath, ...
    'Position', position, ...
    'BackgroundColor', 'white');

deleteIfExists([subsystemPath '/In1']);
deleteIfExists([subsystemPath '/Out1']);
end


function addInport(parentPath, blockName, portNumber, dimensions, position)
add_block( ...
    'simulink/Sources/In1', ...
    [parentPath '/' blockName], ...
    'Port', num2str(portNumber), ...
    'PortDimensions', dimensions, ...
    'OutDataTypeStr', 'double', ...
    'Position', position);
end


function addOutport(parentPath, blockName, portNumber, dimensions, position)
add_block( ...
    'simulink/Sinks/Out1', ...
    [parentPath '/' blockName], ...
    'Port', num2str(portNumber), ...
    'PortDimensions', dimensions, ...
    'Position', position);
end


function connect(parentPath, sourcePort, destinationPort)
add_line( ...
    parentPath, ...
    sourcePort, ...
    destinationPort, ...
    'autorouting', 'on');
end


function deleteIfExists(blockPath)
if getSimulinkBlockHandle(blockPath) ~= -1
    delete_block(blockPath);
end
end


function warnMissingConfigurationFields(requiredFields)
if evalin('base', 'exist(''cfg'',''var'')') ~= 1
    warning('StereoIBVS:ConfigurationNotLoaded', ...
        ['基础工作区中还没有 cfg。08仍会创建，' ...
         '但编译模型前必须先运行配置脚本。']);
    return;
end

cfgValue = evalin('base', 'cfg');
missingFields = {};

for fieldIndex = 1:numel(requiredFields)
    if ~isfield(cfgValue, requiredFields{fieldIndex})
        missingFields{end+1} = requiredFields{fieldIndex}; %#ok<AGROW>
    end
end

if ~isempty(missingFields)
    warning('StereoIBVS:MissingConfigurationFields', ...
        'cfg缺少以下字段：%s', ...
        strjoin(missingFields, ', '));
end
end


function code = visibilityGuardCode()
code = strjoin({
    'function [ ...';
    '    uRightPredicted, ...';
    '    rightVisibilityMarginActualPx, ...';
    '    fRightMaxVisibleMm, ...';
    '    fRightMaxEffectiveMm, ...';
    '    rightScaleDesiredEffective, ...';
    '    rightVisibilityActive, ...';
    '    rightVisibilityInfeasible, ...';
    '    scaleDesiredEffective, ...';
    '    rightScaleDesiredNominal, ...';
    '    fRightMeasuredMm] = fcn( ...';
    '    xLeftMeasured, ...';
    '    rhoHatSafe, ...';
    '    focalLengthMeasuredMm, ...';
    '    focalLengthDataValid, ...';
    '    cameraModelValid, ...';
    '    validLeft, ...';
    '    validRight, ...';
    '    ekfPredictionValid, ...';
    '    rightReacquireActive, ...';
    '    resetSafe, ...';
    '    scaleDesired, ...';
    '    numericalEpsilon, ...';
    '    baseline, ...';
    '    outputPixelPitchXmm, ...';
    '    cxR, ...';
    '    focalLengthHardwareMaxMm, ...';
    '    visibilityEpsilon, ...';
    '    imageWidthPx, ...';
    '    rightVisibilityMarginPx, ...';
    '    focalLengthWorkingMaxMm, ...';
    '    focalLengthWorkingMinMm, ...';
    '    targetCharacteristicSize, ...';
    '    rightVisibilityHysteresisPx)';
    '%#codegen';
    '% 右相机可见性保护。';
    '% 所有配置参数均由外部 Constant 块输入。';
    '';
    'persistent active';
    '';
    'if isempty(active)';
    '    active = 1;';
    'end';
    '';
    'if resetSafe > 0.5';
    '    active = 1;';
    'end';
    '';
    '%% 默认安全输出';
    'uRightPredicted = cxR;';
    'rightVisibilityMarginActualPx = 0;';
    '';
    'fRightMaxVisibleMm = 0;';
    'fRightMaxEffectiveMm = ...';
    '    focalLengthWorkingMinMm(2);';
    '';
    'rightScaleDesiredNominal = ...';
    '    scaleDesired(2);';
    '';
    'rightScaleDesiredEffective = ...';
    '    rightScaleDesiredNominal;';
    '';
    'scaleDesiredEffective = ...';
    '    scaleDesired;';
    '';
    'rightVisibilityActive = 1;';
    'rightVisibilityInfeasible = 1;';
    '';
    'fRightMeasuredMm = 0;';
    '';
    '%% 输入检查';
    'inputFinite = ...';
    '    isfinite(xLeftMeasured) && ...';
    '    isfinite(rhoHatSafe) && ...';
    '    all(isfinite(focalLengthMeasuredMm)) && ...';
    '    all(isfinite(scaleDesired)) && ...';
    '    isfinite(numericalEpsilon) && ...';
    '    isfinite(baseline) && ...';
    '    isfinite(outputPixelPitchXmm) && ...';
    '    isfinite(cxR) && ...';
    '    all(isfinite(focalLengthHardwareMaxMm)) && ...';
    '    isfinite(visibilityEpsilon) && ...';
    '    isfinite(imageWidthPx) && ...';
    '    isfinite(rightVisibilityMarginPx) && ...';
    '    all(isfinite(focalLengthWorkingMaxMm)) && ...';
    '    all(isfinite(focalLengthWorkingMinMm)) && ...';
    '    isfinite(targetCharacteristicSize) && ...';
    '    isfinite(rightVisibilityHysteresisPx);';
    '';
    'modelReady = ...';
    '    focalLengthDataValid > 0.5 && ...';
    '    cameraModelValid > 0.5 && ...';
    '    outputPixelPitchXmm > numericalEpsilon;';
    '';
    'if ~inputFinite || ~modelReady';
    '    active = 1;';
    '    return;';
    'end';
    '';
    '%% 右相机预测位置';
    'fRightMeasuredMm = ...';
    '    focalLengthMeasuredMm(2);';
    '';
    'rho = max( ...';
    '    rhoHatSafe, ...';
    '    numericalEpsilon);';
    '';
    'xRightPredicted = ...';
    '    xLeftMeasured - ...';
    '    baseline * rho;';
    '';
    'fRightPx = ...';
    '    focalLengthMeasuredMm(2) / ...';
    '    outputPixelPitchXmm;';
    '';
    'uRightPredicted = ...';
    '    cxR + ...';
    '    fRightPx * xRightPredicted;';
    '';
    'rightVisibilityMarginActualPx = min( ...';
    '    uRightPredicted, ...';
    '    imageWidthPx - uRightPredicted);';
    '';
    '%% 可见焦距上限';
    'hardwareMaxPx = ...';
    '    focalLengthHardwareMaxMm(2) / ...';
    '    outputPixelPitchXmm;';
    '';
    'if xRightPredicted < -visibilityEpsilon';
    '';
    '    fMaxPx = ...';
    '        (cxR - rightVisibilityMarginPx) / ...';
    '        (-xRightPredicted);';
    '';
    'elseif xRightPredicted > visibilityEpsilon';
    '';
    '    fMaxPx = ...';
    '        (imageWidthPx - ...';
    '        rightVisibilityMarginPx - ...';
    '        cxR) / ...';
    '        xRightPredicted;';
    '';
    'else';
    '    fMaxPx = hardwareMaxPx;';
    'end';
    '';
    'fMaxPx = max(fMaxPx, 0);';
    '';
    'fRightMaxVisibleMm = ...';
    '    fMaxPx * outputPixelPitchXmm;';
    '';
    'rawEffectiveMaxMm = min([';
    '    focalLengthWorkingMaxMm(2)';
    '    focalLengthHardwareMaxMm(2)';
    '    fRightMaxVisibleMm';
    ']);';
    '';
    'rightVisibilityInfeasible = double( ...';
    '    rawEffectiveMaxMm < ...';
    '    focalLengthWorkingMinMm(2));';
    '';
    'fRightMaxEffectiveMm = max( ...';
    '    focalLengthWorkingMinMm(2), ...';
    '    rawEffectiveMaxMm);';
    '';
    '%% 调整右相机期望尺度';
    'scaleRightMaxVisible = ...';
    '    targetCharacteristicSize * ...';
    '    fMaxPx * rho;';
    '';
    'rightScaleDesiredEffective = min( ...';
    '    rightScaleDesiredNominal, ...';
    '    max(scaleRightMaxVisible, numericalEpsilon));';
    '';
    'scaleDesiredEffective = [';
    '    scaleDesired(1)';
    '    rightScaleDesiredEffective';
    '];';
    '';
    '%% 滞回状态';
    'enterGuard = ...';
    '    validLeft <= 0.5 || ...';
    '    validRight <= 0.5 || ...';
    '    ekfPredictionValid <= 0.5 || ...';
    '    rightReacquireActive > 0.5 || ...';
    '    rightVisibilityInfeasible > 0.5 || ...';
    '    uRightPredicted < rightVisibilityMarginPx || ...';
    '    uRightPredicted > ...';
    '        imageWidthPx - rightVisibilityMarginPx || ...';
    '    focalLengthMeasuredMm(2) > ...';
    '        fRightMaxEffectiveMm + numericalEpsilon;';
    '';
    'exitGuard = ...';
    '    validLeft > 0.5 && ...';
    '    validRight > 0.5 && ...';
    '    ekfPredictionValid > 0.5 && ...';
    '    rightReacquireActive <= 0.5 && ...';
    '    rightVisibilityInfeasible <= 0.5 && ...';
    '    uRightPredicted >= ...';
    '        rightVisibilityMarginPx + ...';
    '        rightVisibilityHysteresisPx && ...';
    '    uRightPredicted <= ...';
    '        imageWidthPx - ...';
    '        rightVisibilityMarginPx - ...';
    '        rightVisibilityHysteresisPx && ...';
    '    focalLengthMeasuredMm(2) <= ...';
    '        fRightMaxEffectiveMm;';
    '';
    'if enterGuard';
    '    active = 1;';
    'elseif exitGuard';
    '    active = 0;';
    'end';
    '';
    'rightVisibilityActive = ...';
    '    double(active > 0.5);';
    '';
    '%% 输出保护';
    'if any(~isfinite([';
    '        uRightPredicted';
    '        rightVisibilityMarginActualPx';
    '        fRightMaxVisibleMm';
    '        fRightMaxEffectiveMm';
    '        rightScaleDesiredEffective';
    '        scaleDesiredEffective';
    '        fRightMeasuredMm]))';
    '';
    '    active = 1;';
    '';
    '    uRightPredicted = cxR;';
    '    rightVisibilityMarginActualPx = 0;';
    '';
    '    fRightMaxVisibleMm = 0;';
    '    fRightMaxEffectiveMm = ...';
    '        focalLengthWorkingMinMm(2);';
    '';
    '    rightScaleDesiredEffective = ...';
    '        numericalEpsilon;';
    '';
    '    scaleDesiredEffective = [';
    '        scaleDesired(1)';
    '        rightScaleDesiredEffective';
    '    ];';
    '';
    '    rightVisibilityActive = 1;';
    '    rightVisibilityInfeasible = 1;';
    '    fRightMeasuredMm = 0;';
    'end';
    'end';
}, sprintf('\n'));
end
function code = zoomControllerCode()
code = strjoin({
    'function [ ...';
    '    fDotMmCmd, ...';
    '    fDotMmLimited, ...';
    '    scaleError, ...';
    '    gDotCmd, ...';
    '    focalCommandOutwardAtWorkingLimit, ...';
    '    focalFeasible, ...';
    '    focalRateUsageGuaranteedCmd, ...';
    '    focalRateUsageHardCmd] = fcn( ...';
    '    focalLengthMeasuredMm, ...';
    '    focalLengthFresh, ...';
    '    focalLengthDataValid, ...';
    '    cameraModelValid, ...';
    '    scaleMeasured, ...';
    '    rhoHatSafe, ...';
    '    rhoDotHat, ...';
    '    scaleDesiredEffective, ...';
    '    validLeft, ...';
    '    ekfPredictionValid, ...';
    '    focalAtWorkingLowerLimit, ...';
    '    focalAtWorkingUpperLimit, ...';
    '    fRightMaxEffectiveMm, ...';
    '    rightVisibilityActive, ...';
    '    rightReacquireActive, ...';
    '    numericalEpsilon, ...';
    '    focalLengthWorkingMaxMm, ...';
    '    robustEnable, ...';
    '    betaF, ...';
    '    epsilonF, ...';
    '    Kf, ...';
    '    targetCharacteristicSize, ...';
    '    outputPixelPitchXmm, ...';
    '    focalLengthWorkingMinMm, ...';
    '    rightReacquireZoomRateMmPerSec, ...';
    '    focalRateDesignMmPerSec, ...';
    '    Ts, ...';
    '    focalRateGuaranteedMmPerSec, ...';
    '    focalRateAbsoluteMaxMmPerSec, ...';
    '    zoomControlEnable)';
    '%#codegen';
    '% V2 Zoom控制器。';
    '% 焦距单位mm，速度命令单位mm/s。';
    '% 所有配置参数均由外部 Constant 块输入。';
    '';
    '%% 固定尺寸输出';
    'fDotMmCmd = zeros(2,1);';
    'fDotMmLimited = zeros(2,1);';
    '';
    'scaleError = zeros(2,1);';
    'gDotCmd = zeros(2,1);';
    '';
    'focalCommandOutwardAtWorkingLimit = ...';
    '    zeros(2,1);';
    '';
    'focalFeasible = zeros(2,1);';
    '';
    'focalRateUsageGuaranteedCmd = ...';
    '    zeros(2,1);';
    '';
    'focalRateUsageHardCmd = ...';
    '    zeros(2,1);';
    '';
    '%% 输入检查';
    'inputFinite = ...';
    '    all(isfinite(focalLengthMeasuredMm)) && ...';
    '    all(isfinite(scaleMeasured)) && ...';
    '    isfinite(rhoHatSafe) && ...';
    '    all(isfinite(rhoDotHat)) && ...';
    '    all(isfinite(scaleDesiredEffective)) && ...';
    '    all(isfinite(focalAtWorkingLowerLimit)) && ...';
    '    all(isfinite(focalAtWorkingUpperLimit)) && ...';
    '    isfinite(fRightMaxEffectiveMm) && ...';
    '    isfinite(numericalEpsilon) && ...';
    '    all(isfinite(focalLengthWorkingMaxMm)) && ...';
    '    isfinite(robustEnable) && ...';
    '    all(isfinite(betaF)) && ...';
    '    all(isfinite(epsilonF)) && ...';
    '    all(isfinite(Kf(:))) && ...';
    '    isfinite(targetCharacteristicSize) && ...';
    '    isfinite(outputPixelPitchXmm) && ...';
    '    all(isfinite(focalLengthWorkingMinMm)) && ...';
    '    isfinite(rightReacquireZoomRateMmPerSec) && ...';
    '    isfinite(focalRateDesignMmPerSec) && ...';
    '    isfinite(Ts) && ...';
    '    isfinite(focalRateGuaranteedMmPerSec) && ...';
    '    isfinite(focalRateAbsoluteMaxMmPerSec) && ...';
    '    isfinite(zoomControlEnable);';
    '';
    'if ~inputFinite || ...';
    '        numericalEpsilon <= 0 || ...';
    '        outputPixelPitchXmm <= numericalEpsilon || ...';
    '        Ts <= 0';
    '';
    '    return;';
    'end';
    '';
    'rho = max(rhoHatSafe, numericalEpsilon);';
    '';
    'upperEffectiveMm = [';
    '    focalLengthWorkingMaxMm(1)';
    '    fRightMaxEffectiveMm';
    '];';
    '';
    '%% V2控制律';
    'for cameraIndex = 1:2';
    '';
    '    scaleCurrent = max( ...';
    '        scaleMeasured(cameraIndex), ...';
    '        numericalEpsilon);';
    '';
    '    scaleDesiredCurrent = max( ...';
    '        scaleDesiredEffective(cameraIndex), ...';
    '        numericalEpsilon);';
    '';
    '    scaleError(cameraIndex) = ...';
    '        log(scaleCurrent / scaleDesiredCurrent);';
    '';
    '    depthRateEstimate = ...';
    '        rhoDotHat(cameraIndex) / rho;';
    '';
    '    robustTerm = 0;';
    '';
    '    if robustEnable > 0.5';
    '        robustTerm = ...';
    '            betaF(cameraIndex) * ...';
    '            scaleError(cameraIndex) / ...';
    '            (abs(scaleError(cameraIndex)) + ...';
    '            epsilonF(cameraIndex));';
    '    end';
    '';
    '    gDotCmd(cameraIndex) = ...';
    '        -Kf(cameraIndex,cameraIndex) * ...';
    '        scaleError(cameraIndex) - ...';
    '        depthRateEstimate - ...';
    '        robustTerm;';
    '';
    '    fDotMmCmd(cameraIndex) = ...';
    '        focalLengthMeasuredMm(cameraIndex) * ...';
    '        gDotCmd(cameraIndex);';
    '';
    '    focalRequiredMm = ...';
    '        scaleDesiredEffective(cameraIndex) / ...';
    '        (targetCharacteristicSize * rho) * ...';
    '        outputPixelPitchXmm;';
    '';
    '    focalFeasible(cameraIndex) = double( ...';
    '        focalRequiredMm >= ...';
    '            focalLengthWorkingMinMm(cameraIndex) && ...';
    '        focalRequiredMm <= ...';
    '            upperEffectiveMm(cameraIndex));';
    'end';
    '';
    '%% 焦距反馈超时或模型失效：Zoom立即停止';
    'zoomInputValid = ...';
    '    focalLengthFresh > 0.5 && ...';
    '    focalLengthDataValid > 0.5 && ...';
    '    cameraModelValid > 0.5;';
    '';
    'if ~zoomInputValid || ...';
    '        zoomControlEnable <= 0.5';
    '';
    '    fDotMmCmd = zeros(2,1);';
    '    fDotMmLimited = zeros(2,1);';
    '    gDotCmd = zeros(2,1);';
    '    return;';
    'end';
    '';
    '%% 右相机可见性保护';
    'if rightVisibilityActive > 0.5 || ...';
    '        rightReacquireActive > 0.5';
    '';
    '    if focalLengthMeasuredMm(2) > ...';
    '            fRightMaxEffectiveMm + numericalEpsilon';
    '';
    '        fDotMmCmd(2) = ...';
    '            -rightReacquireZoomRateMmPerSec;';
    '    else';
    '        fDotMmCmd(2) = ...';
    '            min(fDotMmCmd(2), 0);';
    '    end';
    'end';
    '';
    '%% 左相机或EKF无效：保留V2安全缩焦';
    'if validLeft <= 0.5 || ...';
    '        ekfPredictionValid <= 0.5';
    '';
    '    fDotMmCmd = zeros(2,1);';
    '';
    '    for cameraIndex = 1:2';
    '        if focalLengthMeasuredMm(cameraIndex) > ...';
    '                focalLengthWorkingMinMm(cameraIndex)';
    '';
    '            fDotMmCmd(cameraIndex) = ...';
    '                -rightReacquireZoomRateMmPerSec;';
    '        end';
    '    end';
    'end';
    '';
    '%% 工作范围方向诊断';
    'for cameraIndex = 1:2';
    '';
    '    focalCommandOutwardAtWorkingLimit(cameraIndex) = ...';
    '        double( ...';
    '        (focalAtWorkingLowerLimit(cameraIndex) > 0.5 && ...';
    '        fDotMmCmd(cameraIndex) < 0) || ...';
    '        (focalAtWorkingUpperLimit(cameraIndex) > 0.5 && ...';
    '        fDotMmCmd(cameraIndex) > 0) || ...';
    '        (focalLengthMeasuredMm(cameraIndex) >= ...';
    '        upperEffectiveMm(cameraIndex) && ...';
    '        fDotMmCmd(cameraIndex) > 0));';
    'end';
    '';
    '%% V2设计速度限制';
    'fDotMmLimited = min( ...';
    '    max( ...';
    '        fDotMmCmd, ...';
    '        -focalRateDesignMmPerSec), ...';
    '    focalRateDesignMmPerSec);';
    '';
    '%% 防止下一周期越过右相机可见焦距上限';
    'nextRightFocalLengthMm = ...';
    '    focalLengthMeasuredMm(2) + ...';
    '    Ts * fDotMmLimited(2);';
    '';
    'if focalLengthMeasuredMm(2) <= ...';
    '        fRightMaxEffectiveMm && ...';
    '        nextRightFocalLengthMm > ...';
    '        fRightMaxEffectiveMm';
    '';
    '    fDotMmLimited(2) = min( ...';
    '        fDotMmLimited(2), ...';
    '        (fRightMaxEffectiveMm - ...';
    '        focalLengthMeasuredMm(2)) / Ts);';
    'end';
    '';
    '%% 速度使用率';
    'if focalRateGuaranteedMmPerSec > numericalEpsilon';
    '    focalRateUsageGuaranteedCmd = ...';
    '        abs(fDotMmLimited) / ...';
    '        focalRateGuaranteedMmPerSec;';
    'end';
    '';
    'if focalRateAbsoluteMaxMmPerSec > numericalEpsilon';
    '    focalRateUsageHardCmd = ...';
    '        abs(fDotMmLimited) / ...';
    '        focalRateAbsoluteMaxMmPerSec;';
    'end';
    '';
    '%% 输出有限值保护';
    'if any(~isfinite(fDotMmCmd)) || ...';
    '        any(~isfinite(fDotMmLimited)) || ...';
    '        any(~isfinite(scaleError)) || ...';
    '        any(~isfinite(gDotCmd)) || ...';
    '        any(~isfinite(focalFeasible)) || ...';
    '        any(~isfinite(focalRateUsageGuaranteedCmd)) || ...';
    '        any(~isfinite(focalRateUsageHardCmd))';
    '';
    '    fDotMmCmd = zeros(2,1);';
    '    fDotMmLimited = zeros(2,1);';
    '';
    '    scaleError = zeros(2,1);';
    '    gDotCmd = zeros(2,1);';
    '';
    '    focalCommandOutwardAtWorkingLimit = ...';
    '        zeros(2,1);';
    '';
    '    focalFeasible = zeros(2,1);';
    '';
    '    focalRateUsageGuaranteedCmd = ...';
    '        zeros(2,1);';
    '';
    '    focalRateUsageHardCmd = ...';
    '        zeros(2,1);';
    'end';
    'end';
}, sprintf('\n'));
end
function code = zoomPrioritySupervisorCode()
code = strjoin({
    'function [ ...';
    '    depthTaskWeight, ...';
    '    schedulerMode, ...';
    '    zoomPriorityActive, ...';
    '    zoomPriorityTimer, ...';
    '    scaleSettledTimer, ...';
    '    armRecoveryReason, ...';
    '    newDisturbanceDetected] = fcn( ...';
    '    scaleError, ...';
    '    depthErrorDelayed, ...';
    '    focalLengthMeasuredMm, ...';
    '    focalLengthFresh, ...';
    '    focalLengthDataValid, ...';
    '    cameraModelValid, ...';
    '    focalAtWorkingLowerLimit, ...';
    '    focalAtWorkingUpperLimit, ...';
    '    focalHeadroomMm, ...';
    '    focalFeasible, ...';
    '    validLeft, ...';
    '    validStereoQualified, ...';
    '    ekfPredictionValid, ...';
    '    resetSafe, ...';
    '    focalCommandOutwardAtWorkingLimit, ...';
    '    focalRateUsageGuaranteedCmd, ...';
    '    focalRateUsageHardCmd, ...';
    '    rightReacquireActive, ...';
    '    Ts, ...';
    '    zoomPriorityEnable, ...';
    '    armDepthRampTime, ...';
    '    scaleErrorEnterThreshold, ...';
    '    scaleErrorExitThreshold, ...';
    '    scaleSettledHoldTime, ...';
    '    disturbanceConfirmTime, ...';
    '    zoomOnlyMaxTime)';
    '%#codegen';
    '% V2 Zoom优先级状态机。';
    '% 所有配置参数均由外部 Constant 块输入。';
    '%';
    '% schedulerMode:';
    '% 0 INITIALIZE';
    '% 1 ZOOM_FIRST';
    '% 2 ARM_RECOVERY';
    '% 3 FULL_TRACK';
    '% 4 SAFE_HOLD';
    '% 5 RIGHT_REACQUIRE';
    '';
    'persistent mode';
    'persistent zoomTimer';
    'persistent settledTimer';
    'persistent rampTimer';
    'persistent reason';
    'persistent disturbanceTimer';
    'persistent validLast';
    'persistent rateTimer';
    '';
    'if isempty(mode)';
    '    mode = 0;';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = 0;';
    '    reason = 0;';
    '    disturbanceTimer = 0;';
    '    validLast = 0;';
    '    rateTimer = 0;';
    'end';
    '';
    'if resetSafe > 0.5';
    '    mode = 0;';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = 0;';
    '    reason = 0;';
    '    disturbanceTimer = 0;';
    '    validLast = 0;';
    '    rateTimer = 0;';
    'end';
    '';
    'depthTaskWeight = 0;';
    'newDisturbanceDetected = 0;';
    '';
    'centerValid = ...';
    '    validLeft > 0.5;';
    '';
    'focalStateValid = ...';
    '    focalLengthDataValid > 0.5 && ...';
    '    cameraModelValid > 0.5;';
    '';
    'depthReady = ...';
    '    validStereoQualified > 0.5 && ...';
    '    ekfPredictionValid > 0.5 && ...';
    '    rightReacquireActive <= 0.5;';
    '';
    'maximumScaleError = ...';
    '    max(abs(scaleError));';
    '';
    '%% 中心任务无效';
    'if ~centerValid';
    '';
    '    mode = 4;';
    '    depthTaskWeight = 0;';
    '';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = 0;';
    '    disturbanceTimer = 0;';
    '    rateTimer = 0;';
    '    validLast = 0;';
    '';
    '%% 焦距数据或相机模型无效';
    'elseif ~focalStateValid';
    '';
    '    mode = 4;';
    '    depthTaskWeight = 0;';
    '';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = 0;';
    '    disturbanceTimer = 0;';
    '    rateTimer = 0;';
    '    validLast = 0;';
    '';
    '%% 双目深度暂不可用';
    'elseif ~depthReady';
    '';
    '    mode = 5;';
    '    depthTaskWeight = 0;';
    '';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = 0;';
    '    disturbanceTimer = 0;';
    '    rateTimer = 0;';
    '    validLast = 0;';
    '';
    '%% 焦距仅不新鲜：Zoom暂停，深度继续';
    'elseif focalLengthFresh <= 0.5';
    '';
    '    mode = 3;';
    '    depthTaskWeight = 1;';
    '';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = armDepthRampTime;';
    '    disturbanceTimer = 0;';
    '    rateTimer = 0;';
    '    reason = 0;';
    '    validLast = 1;';
    '';
    '%% 禁用Zoom优先级';
    'elseif zoomPriorityEnable <= 0.5';
    '';
    '    mode = 3;';
    '    depthTaskWeight = 1;';
    '';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = armDepthRampTime;';
    '    disturbanceTimer = 0;';
    '    rateTimer = 0;';
    '    reason = 0;';
    '    validLast = 1;';
    '';
    'else';
    '';
    '    if validLast < 0.5 && ...';
    '            (mode == 4 || mode == 5)';
    '';
    '        mode = 1;';
    '';
    '        zoomTimer = 0;';
    '        settledTimer = 0;';
    '        rampTimer = 0;';
    '        disturbanceTimer = 0;';
    '        rateTimer = 0;';
    '        reason = 0;';
    '    end';
    '';
    '    if mode == 0';
    '';
    '        if maximumScaleError > ...';
    '                scaleErrorEnterThreshold';
    '';
    '            mode = 1;';
    '            depthTaskWeight = 0;';
    '';
    '        else';
    '            mode = 3;';
    '            depthTaskWeight = 1;';
    '        end';
    '';
    '    elseif mode == 1';
    '';
    '        depthTaskWeight = 0;';
    '        zoomTimer = zoomTimer + Ts;';
    '';
    '        if maximumScaleError <= ...';
    '                scaleErrorExitThreshold';
    '';
    '            settledTimer = ...';
    '                settledTimer + Ts;';
    '        else';
    '            settledTimer = 0;';
    '        end';
    '';
    '        if any( ...';
    '                focalRateUsageGuaranteedCmd >= 1)';
    '';
    '            rateTimer = ...';
    '                rateTimer + Ts;';
    '        else';
    '            rateTimer = 0;';
    '        end';
    '';
    '        if any(focalRateUsageHardCmd >= 1)';
    '';
    '            mode = 2;';
    '            reason = 6;';
    '            rampTimer = 0;';
    '            rateTimer = 0;';
    '';
    '        elseif rateTimer >= ...';
    '                disturbanceConfirmTime && ...';
    '                maximumScaleError > ...';
    '                scaleErrorExitThreshold';
    '';
    '            mode = 2;';
    '            reason = 5;';
    '            rampTimer = 0;';
    '            rateTimer = 0;';
    '';
    '        elseif settledTimer >= ...';
    '                scaleSettledHoldTime';
    '';
    '            mode = 2;';
    '            reason = 1;';
    '            rampTimer = 0;';
    '            rateTimer = 0;';
    '';
    '        elseif any( ...';
    '                focalCommandOutwardAtWorkingLimit > 0.5)';
    '';
    '            mode = 2;';
    '            reason = 7;';
    '            rampTimer = 0;';
    '            rateTimer = 0;';
    '';
    '        elseif zoomTimer >= ...';
    '                disturbanceConfirmTime && ...';
    '                any(focalFeasible < 0.5)';
    '';
    '            mode = 2;';
    '            reason = 3;';
    '            rampTimer = 0;';
    '            rateTimer = 0;';
    '';
    '        elseif zoomTimer >= zoomOnlyMaxTime';
    '';
    '            mode = 2;';
    '            reason = 4;';
    '            rampTimer = 0;';
    '            rateTimer = 0;';
    '        end';
    '';
    '    elseif mode == 2';
    '';
    '        rampTimer = min( ...';
    '            rampTimer + Ts, ...';
    '            armDepthRampTime);';
    '';
    '        rampFraction = min(max( ...';
    '            rampTimer / max( ...';
    '                armDepthRampTime, ...';
    '                Ts), ...';
    '            0), 1);';
    '';
    '        depthTaskWeight = ...';
    '            rampFraction * ...';
    '            rampFraction * ...';
    '            (3 - 2*rampFraction);';
    '';
    '        if depthTaskWeight >= 1 - 1e-6';
    '            mode = 3;';
    '            depthTaskWeight = 1;';
    '            disturbanceTimer = 0;';
    '        end';
    '';
    '    elseif mode == 3';
    '';
    '        depthTaskWeight = 1;';
    '';
    '        if maximumScaleError >= ...';
    '                scaleErrorEnterThreshold';
    '';
    '            depthTaskWeight = 0;';
    '';
    '            disturbanceTimer = ...';
    '                disturbanceTimer + Ts;';
    '        else';
    '            disturbanceTimer = 0;';
    '        end';
    '';
    '        if disturbanceTimer >= ...';
    '                disturbanceConfirmTime';
    '';
    '            mode = 1;';
    '            depthTaskWeight = 0;';
    '';
    '            zoomTimer = 0;';
    '            settledTimer = 0;';
    '            rampTimer = 0;';
    '            disturbanceTimer = 0;';
    '            rateTimer = 0;';
    '            reason = 0;';
    '';
    '            newDisturbanceDetected = 1;';
    '        end';
    '';
    '    else';
    '';
    '        mode = 1;';
    '        depthTaskWeight = 0;';
    '';
    '        zoomTimer = 0;';
    '        settledTimer = 0;';
    '        rampTimer = 0;';
    '        disturbanceTimer = 0;';
    '        rateTimer = 0;';
    '        reason = 0;';
    '    end';
    '';
    '    validLast = 1;';
    'end';
    '';
    'schedulerMode = mode;';
    '';
    'zoomPriorityActive = ...';
    '    double(mode == 1);';
    '';
    'zoomPriorityTimer = ...';
    '    zoomTimer;';
    '';
    'scaleSettledTimer = ...';
    '    settledTimer;';
    '';
    'armRecoveryReason = ...';
    '    reason;';
    '';
    '%% 有限值保护';
    'diagnosticFinite = all(isfinite([';
    '    depthTaskWeight';
    '    schedulerMode';
    '    zoomPriorityTimer';
    '    scaleSettledTimer';
    '    depthErrorDelayed';
    '    focalLengthMeasuredMm';
    '    focalHeadroomMm';
    '    focalAtWorkingLowerLimit';
    '    focalAtWorkingUpperLimit';
    '    focalLengthFresh';
    '    focalLengthDataValid';
    '    cameraModelValid';
    '    Ts';
    '    zoomPriorityEnable';
    '    armDepthRampTime';
    '    scaleErrorEnterThreshold';
    '    scaleErrorExitThreshold';
    '    scaleSettledHoldTime';
    '    disturbanceConfirmTime';
    '    zoomOnlyMaxTime';
    ']));';
    '';
    'if ~diagnosticFinite';
    '';
    '    mode = 4;';
    '    zoomTimer = 0;';
    '    settledTimer = 0;';
    '    rampTimer = 0;';
    '    reason = 0;';
    '    disturbanceTimer = 0;';
    '    validLast = 0;';
    '    rateTimer = 0;';
    '';
    '    depthTaskWeight = 0;';
    '    schedulerMode = 4;';
    '    zoomPriorityActive = 0;';
    '    zoomPriorityTimer = 0;';
    '    scaleSettledTimer = 0;';
    '    armRecoveryReason = 0;';
    '    newDisturbanceDetected = 0;';
    'end';
    'end';
}, sprintf('\n'));
end
