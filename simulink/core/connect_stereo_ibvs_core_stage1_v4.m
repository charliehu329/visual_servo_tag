function connect_stereo_ibvs_core_stage1_v4(modelFile)
% 代码作用：
% 根据当前stereo_ibvs_core.slx中的实际子系统端口顺序，
% 自动删除并重建Stage 1模型的全部顶层连线。
%
% 本函数只修改顶层连线，并创建少量顶层Constant和Terminator。
% 不会修改01～10子系统内部的模块和连线。
%
% 输入参数含义：
% modelFile：
% 模型名称或模型完整路径。
% 可以省略，默认使用当前目录下的stereo_ibvs_core.slx。
%
% 输出参数含义：
% 本函数没有输出参数。
% 执行完成后会更新并保存模型。

if nargin < 1 || isempty(modelFile)
    modelFile = 'stereo_ibvs_core.slx';
end

modelFile = char(modelFile);
[~, modelName, ~] = fileparts(modelFile);

if isempty(modelName)
    error('Stage1:InvalidModelName', '模型名称不能为空。');
end

%% 加载模型
load_system(modelFile);

% 获取模型实际完整路径。
modelFullPath = get_param(modelName, 'FileName');

if isempty(modelFullPath)
    error('Stage1:ModelPathUnavailable', ...
        '无法获取模型%s的完整文件路径。', modelName);
end

modelDirectory = fileparts(modelFullPath);

%% 查找并运行配置文件
% 优先支持以下两种工程结构：
% 1. 配置文件和模型位于同一目录；
% 2. 模型位于simulink_new/core，配置位于simulink_new/config。
configCandidates = {
    fullfile(modelDirectory, 'stereo_ibvs_config.m')
    fullfile(fileparts(modelDirectory), ...
        'config', 'stereo_ibvs_config.m')
};

configFromPath = which('stereo_ibvs_config');

if ~isempty(configFromPath)
    configCandidates{end+1,1} = configFromPath;
end

configFile = '';

for candidateIndex = 1:numel(configCandidates)
    if isfile(configCandidates{candidateIndex})
        configFile = configCandidates{candidateIndex};
        break;
    end
end

if isempty(configFile)
    error('Stage1:ConfigNotFound', ...
        ['找不到stereo_ibvs_config.m。\n' ...
         '建议路径：%s'], ...
        fullfile(fileparts(modelDirectory), ...
        'config', 'stereo_ibvs_config.m'));
end

%% 修复模型InitFcn
% 不再使用容易受当前工作目录影响的裸命令：
% stereo_ibvs_config
%
% 新回调根据模型自身路径自动寻找配置文件，工程移动后仍可使用。
initCallback = [ ...
    'modelFile__ = get_param(bdroot,''FileName''); ' ...
    'modelDir__ = fileparts(modelFile__); ' ...
    'config1__ = fullfile(modelDir__,''stereo_ibvs_config.m''); ' ...
    'config2__ = fullfile(fileparts(modelDir__),''config'',''stereo_ibvs_config.m''); ' ...
    'if isfile(config1__), ' ...
    'run(config1__); ' ...
    'elseif isfile(config2__), ' ...
    'run(config2__); ' ...
    'else, ' ...
    'error(''StereoIBVS:ConfigNotFound'',' ...
    '''找不到stereo_ibvs_config.m。''); ' ...
    'end; ' ...
    'clear modelFile__ modelDir__ config1__ config2__;'];

set_param(modelName, 'InitFcn', initCallback);

%% 修复当前模型中的信号维度
% 当前上传的SLX中存在两个明确问题：
% 1. 01_Input_Validity/controllerEnableRaw被误设为2维；
% 2. 多个列向量被Simulink按一维向量解释。
%
% 统一使用Nx1列向量，避免MATLAB Function中的persistent变量
% 在一维向量和二维列矩阵之间发生尺寸冲突。
configureStage1Dimensions(modelName);

%% 检查当前模型顶层模块
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
};

for i = 1:numel(requiredBlocks)
    blockPath = [modelName '/' requiredBlocks{i}];

    if getSimulinkBlockHandle(blockPath) == -1
        error('Stage1:MissingBlock', ...
            '当前模型顶层缺少模块：%s', blockPath);
    end
end

%% 检查当前SLX中的实际端口数量
% 这些数量与用户当前上传的stereo_ibvs_core.slx完全对应。
checkPortCount(modelName, '01_Input_Validity',          5,  6);
checkPortCount(modelName, '02_Camera_Model',            1,  3);
checkPortCount(modelName, '03_Feature_Processing',      2,  5);
checkPortCount(modelName, '04_FR3_Camera_Kinematics',   1,  3);
checkPortCount(modelName, '05_Target_EKF',              5,  5);
checkPortCount(modelName, '06_Inverse_Depth_Filter',    5,  4);
checkPortCount(modelName, '07_Arm_Priority_Controller', 6,  4);
checkPortCount(modelName, '08_Zoom_Controller',         7,  3);
checkPortCount(modelName, '09_Safety_Supervisor',       5,  4);
checkPortCount(modelName, '10_Diagnostics',            12,  3);

%% 删除全部顶层旧连线
% SearchDepth=1只搜索顶层，不会删除子系统内部连线。
topLevelLines = find_system( ...
    modelName, ...
    'FindAll', 'on', ...
    'SearchDepth', 1, ...
    'Type', 'line');

for i = 1:numel(topLevelLines)
    try
        delete_line(topLevelLines(i));
    catch
        % 删除主线后，其分支可能已经自动消失。
    end
end

%% 创建或更新顶层辅助模块

% Stage 1真实运动许可。
% 必须同时完成相机安装外参和相机内参标定，才允许安全模块放行。
ensureBlock( ...
    modelName, ...
    'simulink/Sources/Constant', ...
    'Stage1 Calibration Ready', ...
    [1010 1050 1185 1085]);

set_param( ...
    [modelName '/Stage1 Calibration Ready'], ...
    'Value', 'double(cfg.stage1CalibrationReady)', ...
    'SampleTime', '-1');

% 当前Stage 1未使用的信号接入Terminator，避免未连接端口警告。
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
    'Term_10_systemReady'
    'Term_10_faultCode'
};

terminatorPositions = [
     75 1405   95 1425
   1135 1710 1155 1730
   1135 1780 1155 1800
   1135 1850 1155 1870
   1135 1920 1155 1940
    590 2010  610 2030
    975 1100  995 1120
    120 1900  140 1920
   1600  720 1620  740
   1600  790 1620  810
    585 1510  605 1530
    585 1580  605 1600
];

for i = 1:numel(terminatorNames)
    ensureBlock( ...
        modelName, ...
        'simulink/Sinks/Terminator', ...
        terminatorNames{i}, ...
        terminatorPositions(i,:));
end

%% 顶层输入 -> 01_Input_Validity

connectBlocks(modelName, ...
    'qRaw', 1, ...
    '01_Input_Validity', 1);

connectBlocks(modelName, ...
    'visionFeatureRaw', 1, ...
    '01_Input_Validity', 2);

connectBlocks(modelName, ...
    'zoomPositionStepsRaw', 1, ...
    '01_Input_Validity', 3);

connectBlocks(modelName, ...
    'controllerEnableRaw', 1, ...
    '01_Input_Validity', 4);

connectBlocks(modelName, ...
    'resetRaw', 1, ...
    '01_Input_Validity', 5);

%% 01_Input_Validity输出

% 输出1：q
connectBlocks(modelName, ...
    '01_Input_Validity', 1, ...
    '04_FR3_Camera_Kinematics', 1);

connectBlocks(modelName, ...
    '01_Input_Validity', 1, ...
    '09_Safety_Supervisor', 1);

% 输出2：visionFeature
connectBlocks(modelName, ...
    '01_Input_Validity', 2, ...
    '03_Feature_Processing', 1);

% 输出3：zoomPositionSteps
connectBlocks(modelName, ...
    '01_Input_Validity', 3, ...
    '02_Camera_Model', 1);

connectBlocks(modelName, ...
    '01_Input_Validity', 3, ...
    '08_Zoom_Controller', 1);

% 输出4：controllerEnableSafe
connectBlocks(modelName, ...
    '01_Input_Validity', 4, ...
    '07_Arm_Priority_Controller', 6);

connectBlocks(modelName, ...
    '01_Input_Validity', 4, ...
    '08_Zoom_Controller', 6);

connectBlocks(modelName, ...
    '01_Input_Validity', 4, ...
    '09_Safety_Supervisor', 4);

connectBlocks(modelName, ...
    '01_Input_Validity', 4, ...
    '10_Diagnostics', 11);

% 输出5：resetSafe
connectBlocks(modelName, ...
    '01_Input_Validity', 5, ...
    '05_Target_EKF', 5);

connectBlocks(modelName, ...
    '01_Input_Validity', 5, ...
    '06_Inverse_Depth_Filter', 5);

connectBlocks(modelName, ...
    '01_Input_Validity', 5, ...
    '08_Zoom_Controller', 7);

% 输出6：inputDataValid
connectBlocks(modelName, ...
    '01_Input_Validity', 6, ...
    '10_Diagnostics', 1);

%% 02_Camera_Model输出

% 输出1：cameraIntrinsics
connectBlocks(modelName, ...
    '02_Camera_Model', 1, ...
    '03_Feature_Processing', 2);

% 输出2：focalLengthPx，Stage 1暂未使用
connectBlocks(modelName, ...
    '02_Camera_Model', 2, ...
    'Term_02_focalLengthPx', 1);

% 输出3：cameraModelValid
connectBlocks(modelName, ...
    '02_Camera_Model', 3, ...
    '07_Arm_Priority_Controller', 4);

connectBlocks(modelName, ...
    '02_Camera_Model', 3, ...
    '10_Diagnostics', 2);

%% 03_Feature_Processing输出

% 输出1：cLMeasured
connectBlocks(modelName, ...
    '03_Feature_Processing', 1, ...
    '05_Target_EKF', 1);

connectBlocks(modelName, ...
    '03_Feature_Processing', 1, ...
    '06_Inverse_Depth_Filter', 1);

connectBlocks(modelName, ...
    '03_Feature_Processing', 1, ...
    '07_Arm_Priority_Controller', 1);

% 输出2：cRMeasured
connectBlocks(modelName, ...
    '03_Feature_Processing', 2, ...
    '06_Inverse_Depth_Filter', 2);

% 输出3：scaleMeasured
connectBlocks(modelName, ...
    '03_Feature_Processing', 3, ...
    '08_Zoom_Controller', 2);

% 输出4：validLeft
connectBlocks(modelName, ...
    '03_Feature_Processing', 4, ...
    '06_Inverse_Depth_Filter', 3);

connectBlocks(modelName, ...
    '03_Feature_Processing', 4, ...
    '07_Arm_Priority_Controller', 3);

connectBlocks(modelName, ...
    '03_Feature_Processing', 4, ...
    '08_Zoom_Controller', 4);

connectBlocks(modelName, ...
    '03_Feature_Processing', 4, ...
    '10_Diagnostics', 4);

% 输出5：validRight
connectBlocks(modelName, ...
    '03_Feature_Processing', 5, ...
    '06_Inverse_Depth_Filter', 4);

connectBlocks(modelName, ...
    '03_Feature_Processing', 5, ...
    '08_Zoom_Controller', 5);

connectBlocks(modelName, ...
    '03_Feature_Processing', 5, ...
    '10_Diagnostics', 5);

%% 04_FR3_Camera_Kinematics输出

% 输出1：JL
connectBlocks(modelName, ...
    '04_FR3_Camera_Kinematics', 1, ...
    '07_Arm_Priority_Controller', 2);

% 输出2：T_CL2B
connectBlocks(modelName, ...
    '04_FR3_Camera_Kinematics', 2, ...
    '05_Target_EKF', 4);

% 输出3：kinematicsValid
connectBlocks(modelName, ...
    '04_FR3_Camera_Kinematics', 3, ...
    '07_Arm_Priority_Controller', 5);

connectBlocks(modelName, ...
    '04_FR3_Camera_Kinematics', 3, ...
    '10_Diagnostics', 3);

%% 05_Target_EKF输出

connectBlocks(modelName, ...
    '05_Target_EKF', 1, ...
    'Term_05_pHatB', 1);

connectBlocks(modelName, ...
    '05_Target_EKF', 2, ...
    'Term_05_vHatB', 1);

connectBlocks(modelName, ...
    '05_Target_EKF', 3, ...
    'Term_05_aHatB', 1);

connectBlocks(modelName, ...
    '05_Target_EKF', 4, ...
    'Term_05_vHatCL', 1);

% 输出5：ekfValid
connectBlocks(modelName, ...
    '05_Target_EKF', 5, ...
    '10_Diagnostics', 9);

%% 06_Inverse_Depth_Filter输出

% 当前07_Arm_Priority_Controller仍然使用内部固定rho常量，
% 因此rhoForControl只连接08_Zoom_Controller。
% Stage 2再为07增加rhoForControl输入端口。
connectBlocks(modelName, ...
    '06_Inverse_Depth_Filter', 1, ...
    '08_Zoom_Controller', 3);

% 输出2：rhoMeasured
connectBlocks(modelName, ...
    '06_Inverse_Depth_Filter', 2, ...
    '05_Target_EKF', 2);

% 输出3：disparityNormalized
connectBlocks(modelName, ...
    '06_Inverse_Depth_Filter', 3, ...
    'Term_06_disparityNormalized', 1);

% 输出4：depthMeasurementValid
connectBlocks(modelName, ...
    '06_Inverse_Depth_Filter', 4, ...
    '05_Target_EKF', 3);

connectBlocks(modelName, ...
    '06_Inverse_Depth_Filter', 4, ...
    '10_Diagnostics', 6);

%% 07_Arm_Priority_Controller输出

% 输出1：qDotCenter
% 当前09_Safety_Supervisor的实际端口顺序：
% 输入1=q，输入2=qDotRaw。
connectBlocks(modelName, ...
    '07_Arm_Priority_Controller', 1, ...
    '09_Safety_Supervisor', 2);

% 输出2：centerError
connectBlocks(modelName, ...
    '07_Arm_Priority_Controller', 2, ...
    'Term_07_centerError', 1);

% 输出3：centerTaskValid
connectBlocks(modelName, ...
    '07_Arm_Priority_Controller', 3, ...
    '09_Safety_Supervisor', 3);

connectBlocks(modelName, ...
    '07_Arm_Priority_Controller', 3, ...
    '10_Diagnostics', 7);

% 输出4：jConditionMetric
connectBlocks(modelName, ...
    '07_Arm_Priority_Controller', 4, ...
    '10_Diagnostics', 12);

%% 08_Zoom_Controller输出

% 输出1：zoomStepRateCmd
connectBlocks(modelName, ...
    '08_Zoom_Controller', 1, ...
    'zoomStepRateCmd', 1);

% 输出2：scaleError
connectBlocks(modelName, ...
    '08_Zoom_Controller', 2, ...
    'Term_08_scaleError', 1);

% 输出3：zoomControllerValid
connectBlocks(modelName, ...
    '08_Zoom_Controller', 3, ...
    '10_Diagnostics', 10);

%% 09_Safety_Supervisor输入和输出

% 输入5：Stage 1标定许可
connectBlocks(modelName, ...
    'Stage1 Calibration Ready', 1, ...
    '09_Safety_Supervisor', 5);

% 输出1：qDotSafe
connectBlocks(modelName, ...
    '09_Safety_Supervisor', 1, ...
    'jointVelocityCmd', 1);

% 输出2：jointLimitWarning
connectBlocks(modelName, ...
    '09_Safety_Supervisor', 2, ...
    'Term_09_jointLimitWarning', 1);

% 输出3：velocitySaturationFlag
connectBlocks(modelName, ...
    '09_Safety_Supervisor', 3, ...
    'Term_09_velocitySaturationFlag', 1);

% 输出4：safetyValid
connectBlocks(modelName, ...
    '09_Safety_Supervisor', 4, ...
    '10_Diagnostics', 8);

%% 10_Diagnostics输出

% 当前模型顶层只有三个输出：
% jointVelocityCmd、zoomStepRateCmd、controllerStatus。
connectBlocks(modelName, ...
    '10_Diagnostics', 1, ...
    'controllerStatus', 1);

% systemReady和faultCode暂时不增加顶层端口。
connectBlocks(modelName, ...
    '10_Diagnostics', 2, ...
    'Term_10_systemReady', 1);

connectBlocks(modelName, ...
    '10_Diagnostics', 3, ...
    'Term_10_faultCode', 1);

%% 更新并保存模型
% 先保存InitFcn和顶层连线，再更新模型。
save_system(modelName);

try
    set_param(modelName, 'SimulationCommand', 'update');
catch updateError
    fprintf(2, '\n模型更新失败，完整错误如下：\n');
    fprintf(2, '%s\n', ...
        getReport(updateError, 'extended', ...
        'hyperlinks', 'off'));
    rethrow(updateError);
end

save_system(modelName);

fprintf('\nStage 1顶层连线完成并保存：%s\n', modelName);
fprintf('配置文件：%s\n', configFile);
fprintf('模型InitFcn已经更新为相对路径查找方式。\n');
fprintf('Stage 1向量信号已经统一为Nx1列向量。\n');
fprintf('当前07控制器继续使用内部固定逆深度cfg.rhoD。\n');
fprintf('当前顶层输出：jointVelocityCmd、zoomStepRateCmd、controllerStatus。\n');

end


function configureStage1Dimensions(modelName)
% 代码作用：
% 修复Stage 1模型中已知的端口尺寸和Constant向量解释方式。
% 所有向量统一采用Nx1列向量，标量统一采用1。
%
% 输入参数含义：
% modelName：
% 顶层模型名称。
%
% 输出参数含义：
% 本函数没有输出参数。

%% 顶层输入端口
setPortDimensions(modelName, 'qRaw', '[7 1]');
setPortDimensions(modelName, 'visionFeatureRaw', '[8 1]');
setPortDimensions(modelName, 'zoomPositionStepsRaw', '[2 1]');
setPortDimensions(modelName, 'controllerEnableRaw', '1');
setPortDimensions(modelName, 'resetRaw', '1');

% 控制使能信号统一使用double，避免single和double混合。
set_param( ...
    [modelName '/controllerEnableRaw'], ...
    'OutDataTypeStr', 'double');

%% 01_Input_Validity内部输入端口
inputValidity = [modelName '/01_Input_Validity'];

setPortDimensions(inputValidity, 'qRaw', '[7 1]');
setPortDimensions(inputValidity, 'visionFeatureRaw', '[8 1]');
setPortDimensions(inputValidity, 'zoomPositionStepsRaw', '[2 1]');
setPortDimensions(inputValidity, 'controllerEnableRaw', '1');
setPortDimensions(inputValidity, 'resetRaw', '1');

% Constant块默认会把向量参数解释成一维向量。
% 关闭该选项后，cfg中的7x1和2x1形状会被保留。
set_param( ...
    [inputValidity '/Initial Joint Position'], ...
    'VectorParams1D', 'off');

set_param( ...
    [inputValidity '/Initial Zoom Position'], ...
    'VectorParams1D', 'off');

%% 所有子系统中的Constant统一保留参数原始矩阵形状
subsystemNames = {
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
};

for subsystemIndex = 1:numel(subsystemNames)
    subsystemPath = ...
        [modelName '/' subsystemNames{subsystemIndex}];

    constantBlocks = find_system( ...
        subsystemPath, ...
        'SearchDepth', 1, ...
        'BlockType', 'Constant');

    for constantIndex = 1:numel(constantBlocks)
        try
            set_param( ...
                constantBlocks{constantIndex}, ...
                'VectorParams1D', 'off');
        catch
            % 某些特殊Constant没有该参数时不影响后续处理。
        end
    end
end
end


function setPortDimensions(parentPath, blockName, dimensions)
% 代码作用：
% 设置指定Inport或Outport模块的端口维度。
%
% 输入参数含义：
% parentPath：
% 端口所在系统路径。
%
% blockName：
% 端口模块名称。
%
% dimensions：
% Simulink端口维度字符串，例如'[7 1]'或'1'。
%
% 输出参数含义：
% 本函数没有输出参数。

blockPath = [parentPath '/' blockName];

if getSimulinkBlockHandle(blockPath) == -1
    error('Stage1:DimensionBlockMissing', ...
        '设置维度时找不到模块：%s', blockPath);
end

set_param(blockPath, 'PortDimensions', dimensions);
end


function checkPortCount(modelName, blockName, expectedInputs, expectedOutputs)
% 代码作用：
% 检查子系统当前输入输出端口数量是否与连线脚本一致。
%
% 输入参数含义：
% modelName：
% 顶层模型名称。
%
% blockName：
% 需要检查的子系统名称。
%
% expectedInputs：
% 期望输入端口数量。
%
% expectedOutputs：
% 期望输出端口数量。
%
% 输出参数含义：
% 本函数没有输出参数。
% 端口数量不一致时直接报错并停止连线。

blockPath = [modelName '/' blockName];
portHandles = get_param(blockPath, 'PortHandles');

actualInputs = numel(portHandles.Inport);
actualOutputs = numel(portHandles.Outport);

if actualInputs ~= expectedInputs || actualOutputs ~= expectedOutputs
    error('Stage1:PortCountMismatch', ...
        ['模块%s端口数量与当前连线脚本不一致。\n' ...
         '实际：输入%d，输出%d。\n' ...
         '脚本要求：输入%d，输出%d。'], ...
        blockPath, ...
        actualInputs, actualOutputs, ...
        expectedInputs, expectedOutputs);
end
end


function ensureBlock(modelName, libraryBlock, blockName, position)
% 代码作用：
% 如果辅助模块不存在则创建；如果已经存在则更新其位置。
%
% 输入参数含义：
% modelName：
% 顶层模型名称。
%
% libraryBlock：
% Simulink库模块路径。
%
% blockName：
% 创建后的顶层模块名称。
%
% position：
% 模块位置坐标[x1 y1 x2 y2]。
%
% 输出参数含义：
% 本函数没有输出参数。

blockPath = [modelName '/' blockName];

if getSimulinkBlockHandle(blockPath) == -1
    add_block( ...
        libraryBlock, ...
        blockPath, ...
        'Position', position);
else
    set_param(blockPath, 'Position', position);
end
end


function connectBlocks(modelName, ...
    sourceBlock, sourcePort, destinationBlock, destinationPort)
% 代码作用：
% 使用顶层模块端口句柄建立连线。
% 同一个输出连接多个输入时，Simulink会自动创建信号分支。
%
% 输入参数含义：
% modelName：
% 顶层模型名称。
%
% sourceBlock：
% 源模块名称。
%
% sourcePort：
% 源模块输出端口号。
%
% destinationBlock：
% 目标模块名称。
%
% destinationPort：
% 目标模块输入端口号。
%
% 输出参数含义：
% 本函数没有输出参数。

sourcePath = [modelName '/' sourceBlock];
destinationPath = [modelName '/' destinationBlock];

sourceHandles = get_param(sourcePath, 'PortHandles');
destinationHandles = get_param(destinationPath, 'PortHandles');

if numel(sourceHandles.Outport) < sourcePort
    error('Stage1:InvalidSourcePort', ...
        '模块%s不存在输出端口%d。', ...
        sourcePath, sourcePort);
end

if numel(destinationHandles.Inport) < destinationPort
    error('Stage1:InvalidDestinationPort', ...
        '模块%s不存在输入端口%d。', ...
        destinationPath, destinationPort);
end

sourceHandle = sourceHandles.Outport(sourcePort);
destinationHandle = destinationHandles.Inport(destinationPort);

% 目标端口如有旧线，则先删除旧线。
oldLine = get_param(destinationHandle, 'Line');

if oldLine ~= -1
    delete_line(oldLine);
end

add_line( ...
    modelName, ...
    sourceHandle, ...
    destinationHandle, ...
    'autorouting', 'on');
end
