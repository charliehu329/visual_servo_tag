function build_arm_stereo_ibvs_ekf_v1
%BUILD_ARM_STEREO_IBVS_EKF_V1 Reproducibly build the requested Simulink model.
% Public Simulink and Stateflow APIs are used; SLX internals are not edited.

sourceModel = 'active_stereo_ibvs';
model = 'arm_stereo_ibvs_ekf_v1';
backupFile = fullfile(pwd, 'active_stereo_ibvs_original_backup.slx');
logFile = fullfile(pwd, 'model_build_log.txt');

if ~isfile([sourceModel '.slx'])
    error('Missing source model: %s.slx', sourceModel);
end
init_arm_stereo_ibvs_ekf_v1;

logId = fopen(logFile, 'a');
if logId < 0
    error('Cannot open build log: %s', logFile);
end
cleanupLog = onCleanup(@()fclose(logId)); %#ok<NASGU>
logmsg(logId, 'BUILD START');

if ~isfile(backupFile)
    copyfile([sourceModel '.slx'], backupFile);
    logmsg(logId, 'Created byte-for-byte source backup.');
else
    logmsg(logId, 'Existing source backup retained.');
end

if bdIsLoaded(model)
    close_system(model, 0);
end
if bdIsLoaded(sourceModel)
    close_system(sourceModel, 0);
end

% First create the required copy, then modify only the copy.
load_system(sourceModel);
save_system(sourceModel, model);
close_system(sourceModel, 0);
load_system(model);
clearSystemContents(model);

set_param(model, ...
    'Name', model, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'cfg.Ts', ...
    'StartTime', '0', ...
    'StopTime', 'cfg.stopTime', ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'on', ...
    'TimeSaveName', 'tout', ...
    'InitFcn', 'init_arm_stereo_ibvs_ekf_v1', ...
    'AlgebraicLoopMsg', 'error', ...
    'UnconnectedInputMsg', 'warning', ...
    'UnconnectedOutputMsg', 'warning', ...
    'DefaultParameterBehavior', 'Tunable');

add_block('simulink/Sources/Clock', [model '/Simulation Time'], ...
    'Position', [25 58 55 82]);
add_block('simulink/Sources/Constant', [model '/Experiment Mode'], ...
    'Value', 'cfg.experimentMode', 'OutDataTypeStr', 'double', ...
    'Position', [25 103 95 127]);
add_block('simulink/Sources/Constant', [model '/Reset'], ...
    'Value', '0', 'OutDataTypeStr', 'double', ...
    'Position', [25 148 95 172]);
add_block('simulink/Sources/Constant', [model '/Camera Mount Placeholder'], ...
    'Value', 'double(cfg.cameraMountIsPlaceholder)', 'OutDataTypeStr', 'double', ...
    'Position', [25 190 120 214]);
add_block('simulink/Sources/Constant', [model '/Camera Intrinsics Placeholder'], ...
    'Value', 'double(cfg.cameraIntrinsicsArePlaceholder)', 'OutDataTypeStr', 'double', ...
    'Position', [25 225 120 249]);
add_block('simulink/Discrete/Unit Delay', [model '/qDotApplied Delay'], ...
    'SampleTime', 'cfg.Ts', 'InitialCondition', 'zeros(7,1)', ...
    'Position', [1935 495 1985 535]);
add_block('simulink/Discrete/Unit Delay', [model '/Depth Error Delay'], ...
    'SampleTime','cfg.Ts','InitialCondition','0','Position',[2515 245 2565 275]);
add_block('simulink/Sources/Constant', [model '/Center Task Enable'], ...
    'Value', 'cfg.centerTaskEnable', 'Position', [2200 205 2265 229]);
add_block('simulink/Sources/Constant', [model '/Depth Task Enable'], ...
    'Value', 'cfg.depthTaskEnable', 'Position', [2200 240 2265 264]);
add_block('simulink/Sources/Constant', [model '/Arm Control Enable'], ...
    'Value', 'cfg.armControlEnable', 'Position', [2200 275 2265 299]);
add_block('simulink/Sinks/Terminator', [model '/Depth True Terminator'], ...
    'Position', [1300 240 1320 260]);
add_block('simulink/Signal Routing/Demux', [model '/Depth True Demux'], ...
    'Outputs','2','Position',[1300 265 1305 315]);
add_block('simulink/Sinks/Terminator', [model '/gDot Terminator'], ...
    'Position', [2510 475 2530 495]);

addMFunctionSubsystem(model, 'Target Truth Generator', targetTruthCode(), ...
    {'t','mode'}, {'pTargetTrueW','vTargetTrueW','aTargetTrueW'}, [130 35 315 145]);
addJointStateSubsystem(model, [130 190 315 260]);
addMFunctionSubsystem(model, 'FR3 Camera Kinematics', fr3KinematicsCode(), ...
    {'q'}, {'pWCL','RWCL','pWCR','RWCR','JL','JR','rankJL','rcondJL', ...
    'rotationOrthogonalityError','baselineLength','baselineError','stereoRotationError','pWLink8','RWLink8'}, ...
    [350 150 590 410]);
addZoomStateSubsystem(model, [130 560 315 630]);
addReferenceSubsystem(model, [350 595 525 685]);

addMFunctionSubsystem(model, 'Stereo Projection and Measurement', projectionCode(), ...
    {'pTargetTrueW','pWCL','RWCL','pWCR','RWCR','fMeasured'}, ...
    {'pixelMeasurementTrue','normalizedFeatureTrue','scaleTrue','rhoTrue','depthTrue','validStereo', ...
    'leftCameraVisible','rightCameraVisible','leftImageMargin','rightImageMargin','targetDepthRangeViolation', ...
    'desiredFocalLengthEstimate','focalFeasibilityFlag','focalVisibilityLimited','focalScaleLimited','focalEffectiveUpperLimitMm','focalRequiredMm'}, ...
    [1055 35 1285 235]);
addMFunctionSubsystem(model, 'Pixel Measurement Noise', noiseCode(), ...
    {'pixelMeasurementTrue','validLeft','validRight'}, {'pixelMeasurement'}, [1325 50 1505 145]);
addMFunctionSubsystem(model, 'Measured Feature Extraction', measuredFeatureCode(), ...
    {'pixelMeasurement','fMeasured','scaleTrue','validLeft','validRight'}, ...
    {'normalizedFeatureMeasured','scaleMeasured','zMeasured','xLeftMeasured','yLeftMeasured','xRightMeasured', ...
    'disparityNormalized','rhoMeasured','validDisparity','uRightMeasured'}, [1545 40 1775 240]);
addMFunctionSubsystem(model, 'Stereo Validity Manager', validityManagerCode(), ...
    {'validLeft','validRight','validDisparity','reset'}, ...
    {'stereoMeasurementEnable','rightReacquireActive','rightReacquireCount','validStereoQualified'}, [1795 300 1995 430]);
addMFunctionSubsystem(model, 'Target EKF', ekfCode(), ...
    {'zMeasured','measurementEnable','pWCL','RWCL','fMeasured','reset'}, ...
    {'pHatW','vHatW','aHatW','pHatLeft','vHatLeft','rhoHatRaw','rhoHatSafe','innovation','rhoInnovation', ...
    'innovationNorm','traceP','ekfPredictionValid','ekfMeasurementUpdated','ekfStateFinite','minEigenvalueP','symmetryErrorP'}, ...
    [2035 20 2255 285]);
addMFunctionSubsystem(model, 'Estimated Target Motion Transform', motionTransformCode(), ...
    {'vHatW','RWCR'}, {'vHatCR'}, [2290 285 2470 365]);
addMFunctionSubsystem(model, 'Arm Priority Controller', armControllerCode(), ...
    {'q','cLMeasured','rhoHatL','vHatLeft','JL','cLd','rhoD','validLeft','ekfPredictionValid','centerEnable','depthEnable','armEnable','depthTaskWeight'}, ...
    {'qDotCmd','centerError','depthError','rcondJc','rcondArho','qDotCenter','qDotDepthRaw','qDotDepthWeighted','qDotNullUsed'}, [2290 25 2510 250]);
addMFunctionSubsystem(model, 'Estimated Rho Dynamics', rhoDynamicsCode(), ...
    {'normalizedFeatureMeasured','rhoHat','vHatCL','vHatCR','JL','JR','qDotApplied'}, ...
    {'rhoDotHat'}, [2035 340 2250 475]);
addMFunctionSubsystem(model, 'Right Visibility Guard', rightVisibilityGuardCode(), ...
    {'xLeftMeasured','rhoHatSafe','focalLengthMeasuredMm','validLeft','validRight','ekfPredictionValid','rightReacquireActive','scaleDesiredNominal','reset'}, ...
    {'uRightPredicted','rightVisibilityMarginActualPx','fRightMaxVisibleMm','fRightMaxEffectiveMm','rightScaleDesiredEffective', ...
    'rightVisibilityActive','rightVisibilityInfeasible','scaleDesiredEffective','rightScaleDesiredNominal','fRightMeasuredMm'}, [2290 560 2520 805]);
addMFunctionSubsystem(model, 'Zoom Controller', zoomControllerCode(), ...
    {'fMeasuredMm','scaleMeasured','rhoHat','rhoDotHat','scaleDesiredEffective','validLeft','ekfPredictionValid', ...
    'focalAtWorkingLowerLimit','focalAtWorkingUpperLimit','fRightMaxEffectiveMm','rightVisibilityActive','rightReacquireActive'}, ...
    {'fDotMmCmd','fDotMmLimited','scaleError','gDotCmd','focalCommandOutwardAtWorkingLimit','focalFeasible','focalRateUsageGuaranteedCmd','focalRateUsageHardCmd'}, [2290 335 2510 540]);
addMFunctionSubsystem(model, 'Zoom Priority Supervisor', zoomSupervisorCode(), ...
    {'scaleError','depthError','focalLengthMeasuredMm','focalAtWorkingLowerLimit','focalAtWorkingUpperLimit','focalHeadroomMm','focalFeasible', ...
    'validLeft','validStereoQualified','ekfPredictionValid','reset','focalCommandOutwardAtWorkingLimit','focalRateUsageGuaranteed','focalRateUsageHard','rightReacquireActive'}, ...
    {'depthTaskWeight','schedulerMode','zoomPriorityActive','zoomPriorityTimer','scaleSettledTimer','armRecoveryReason','newDisturbanceDetected'}, ...
    [2550 20 2780 230]);
addMFunctionSubsystem(model, 'Safety and Saturation', safetyCode(), ...
    {'qDotCmd','fDotMmLimited','q','focalLengthMeasuredMm','validLeft','JL'}, ...
    {'qDotApplied','fDotMmApplied','qSaturationFlag','focalRateSaturationFlag','jointVelocitySaturationFlag', ...
    'jointVelocitySaturationAny','jointLimitWarning','cartesianSpeed','cartesianSpeedScale','cartesianSpeedSaturationFlag','focalLengthCommandMm','focalRateUsageGuaranteed','focalRateUsageHard','focalRateHardViolation'}, ...
    [2820 250 3070 520]);
addMFunctionSubsystem(model, 'Compatibility Log Aliases', compatibilityLogAliasCode(), ...
    {'pHatWorld','vHatWorld','aHatWorld','rhoHatSafe','ekfPredictionValid','validLeft','validRight'}, ...
    {'targetPositionHatW','targetVelocityHatW','targetAccelerationHatW','rhoHat','ekfValid','leftCameraVisible','rightCameraVisible'}, ...
    [2820 560 3070 800]);

logNames = {'q','qDotCmd','qDotApplied','pWCL','RWCL','pWCR','RWCR', ...
    'baselineError','rotationOrthogonalityError','pixelMeasurementTrue', ...
    'pixelMeasurement','normalizedFeatureTrue','normalizedFeatureMeasured', ...
    'scaleMeasured','centerError','depthError','scaleError','fxMeasuredPx', ...
    'fDotMmCmd','fDotMmAppliedPreview','targetPositionTrueW','targetVelocityTrueW', ...
    'targetAccelerationTrueW','targetPositionHatW','targetVelocityHatW', ...
    'targetAccelerationHatW','rhoTrue','rhoHat','rhoDotHat','ekfInnovation','ekfInnovationNorm', ...
    'ekfTraceP','ekfValid','rcondJc','rcondArho','rankJL','rcondJL', ...
    'validStereo','qSaturationFlag','focalRateSaturationFlag','minEigenvalueP','ekfSymmetryErrorP', ...
    'pWLink8','RWLink8','JL','JR','baselineLength','stereoRotationError', ...
    'cartesianSpeed','cartesianSpeedScale','cartesianSpeedSaturationFlag','jointLimitWarning', ...
    'jointVelocitySaturationFlag','jointVelocitySaturationAny','targetDepthLeftTrue','targetDepthRightTrue', ...
    'targetDepthRangeViolation','leftCameraVisible','rightCameraVisible','leftImageMargin','rightImageMargin', ...
    'cameraMountIsPlaceholder','cameraIntrinsicsArePlaceholder','desiredFocalLengthEstimate','focalFeasibilityFlag', ...
    'focalLengthMeasuredMm','focalLengthCommandMm','fDotMmLimited','fDotMmApplied','fyMeasuredPx','focalRateUsageGuaranteed','focalRateUsageHard', ...
    'focalAtWorkingLowerLimit','focalAtWorkingUpperLimit','focalHeadroomMm','focalFeasible','focalCommandOutwardAtWorkingLimit', ...
    'schedulerMode','depthTaskWeight','zoomPriorityActive','zoomPriorityTimer','scaleSettledTimer','armRecoveryReason','newDisturbanceDetected', ...
    'qDotCenter','qDotDepthRaw','qDotDepthWeighted','qDotNullUsed', ...
    'focalAtHardwareLowerLimit','focalAtHardwareUpperLimit','focalOutsideWorkingRange','focalHardwareLimitViolation','focalRangeUsage','focalRateHardViolation', ...
    'focalVisibilityLimited','focalScaleLimited','focalLengthEffectiveMaxMm','focalRequiredMm', ...
    'focalLengthWorkingMinMm','focalLengthWorkingMaxMm','focalLengthHardwareMinMm','focalLengthHardwareMaxMm', ...
    'xLeftMeasured','yLeftMeasured','xRightMeasured','disparityNormalized','rhoMeasured','validDisparity','uRightMeasured', ...
    'targetPositionHatLeft','targetVelocityHatLeft','rhoHatRaw','rhoHatSafe','rhoInnovation', ...
    'ekfPredictionValid','ekfMeasurementUpdated','ekfStateFinite','stereoMeasurementEnable','rightReacquireActive','rightReacquireCount','validStereoQualified', ...
    'uRightPredicted','rightVisibilityMarginActualPx','fRightMaxVisibleMm','fRightMaxEffectiveMm','rightScaleDesiredNominal','rightScaleDesiredEffective', ...
    'rightVisibilityActive','rightVisibilityInfeasible','fRightMeasuredMm','validLeft','validRight','pHatWorld','vHatWorld','aHatWorld'};
addLoggingSubsystem(model, logNames, [3110 25 3350 900]);

% Functional signal graph.
wire(model, 'Simulation Time',1,'Target Truth Generator',1,'time',false);
wire(model, 'Experiment Mode',1,'Target Truth Generator',2,'experimentMode',false);
wire(model, 'Safety and Saturation',1,'Arm Joint States',1,'qDotApplied',true);
wire(model, 'Arm Joint States',1,'FR3 Camera Kinematics',1,'q',true);
wire(model, 'Safety and Saturation',2,'Zoom States',1,'fDotMmApplied',true);

wire(model, 'Target Truth Generator',1,'Stereo Projection and Measurement',1,'targetPositionTrueW',true);
wire(model, 'FR3 Camera Kinematics',1,'Stereo Projection and Measurement',2,'pWCL',true);
wire(model, 'FR3 Camera Kinematics',2,'Stereo Projection and Measurement',3,'RWCL',true);
wire(model, 'FR3 Camera Kinematics',3,'Stereo Projection and Measurement',4,'pWCR',true);
wire(model, 'FR3 Camera Kinematics',4,'Stereo Projection and Measurement',5,'RWCR',true);
wire(model, 'Zoom States',2,'Stereo Projection and Measurement',6,'fxMeasuredPx',true);
wire(model, 'Stereo Projection and Measurement',1,'Pixel Measurement Noise',1,'pixelMeasurementTrue',true);
wire(model, 'Stereo Projection and Measurement',7,'Pixel Measurement Noise',2,'validLeft',true);
wire(model, 'Stereo Projection and Measurement',8,'Pixel Measurement Noise',3,'validRight',true);
wire(model, 'Pixel Measurement Noise',1,'Measured Feature Extraction',1,'pixelMeasurement',true);
wire(model, 'Zoom States',2,'Measured Feature Extraction',2,'',false);
wire(model, 'Stereo Projection and Measurement',3,'Measured Feature Extraction',3,'',false);
wire(model, 'Stereo Projection and Measurement',7,'Measured Feature Extraction',4,'',false);
wire(model, 'Stereo Projection and Measurement',8,'Measured Feature Extraction',5,'',false);
wire(model, 'Stereo Projection and Measurement',5,'Depth True Terminator',1,'depthTrue',false);
wire(model, 'Stereo Projection and Measurement',5,'Depth True Demux',1,'',false);

wire(model, 'Stereo Projection and Measurement',7,'Stereo Validity Manager',1,'',false);
wire(model, 'Stereo Projection and Measurement',8,'Stereo Validity Manager',2,'',false);
wire(model, 'Measured Feature Extraction',9,'Stereo Validity Manager',3,'',false);
wire(model, 'Reset',1,'Stereo Validity Manager',4,'',false);

wire(model, 'Measured Feature Extraction',3,'Target EKF',1,'zMeasured',true);
wire(model, 'Stereo Validity Manager',1,'Target EKF',2,'stereoMeasurementEnable',true);
wire(model, 'FR3 Camera Kinematics',1,'Target EKF',3,'',false);
wire(model, 'FR3 Camera Kinematics',2,'Target EKF',4,'',false);
wire(model, 'Zoom States',2,'Target EKF',5,'',false);
wire(model, 'Reset',1,'Target EKF',6,'',false);
wire(model, 'Target EKF',2,'Estimated Target Motion Transform',1,'targetVelocityHatW',true);
wire(model, 'FR3 Camera Kinematics',4,'Estimated Target Motion Transform',2,'',false);

wire(model, 'Arm Joint States',1,'Arm Priority Controller',1,'',false);
wire(model, 'Measured Feature Extraction',1,'Arm Priority Controller',2,'normalizedFeatureMeasured',true);
wire(model, 'Target EKF',7,'Arm Priority Controller',3,'rhoHatSafe',true);
wire(model, 'Target EKF',5,'Arm Priority Controller',4,'vHatLeft',true);
wire(model, 'FR3 Camera Kinematics',5,'Arm Priority Controller',5,'',false);
wire(model, 'References',1,'Arm Priority Controller',6,'',false);
wire(model, 'References',2,'Arm Priority Controller',7,'',false);
wire(model, 'Stereo Projection and Measurement',7,'Arm Priority Controller',8,'',false);
wire(model, 'Target EKF',12,'Arm Priority Controller',9,'ekfPredictionValid',true);
wire(model, 'Center Task Enable',1,'Arm Priority Controller',10,'',false);
wire(model, 'Depth Task Enable',1,'Arm Priority Controller',11,'',false);
wire(model, 'Arm Control Enable',1,'Arm Priority Controller',12,'',false);
wire(model, 'Zoom Priority Supervisor',1,'Arm Priority Controller',13,'depthTaskWeight',true);

wire(model, 'Measured Feature Extraction',1,'Estimated Rho Dynamics',1,'',false);
wire(model, 'Target EKF',7,'Estimated Rho Dynamics',2,'',false);
wire(model, 'Target EKF',5,'Estimated Rho Dynamics',3,'',false);
wire(model, 'Estimated Target Motion Transform',1,'Estimated Rho Dynamics',4,'',false);
wire(model, 'FR3 Camera Kinematics',5,'Estimated Rho Dynamics',5,'',false);
wire(model, 'FR3 Camera Kinematics',6,'Estimated Rho Dynamics',6,'',false);
wire(model, 'Safety and Saturation',1,'qDotApplied Delay',1,'',false);
wire(model, 'qDotApplied Delay',1,'Estimated Rho Dynamics',7,'',false);

wire(model, 'Measured Feature Extraction',4,'Right Visibility Guard',1,'xLeftMeasured',true);
wire(model, 'Target EKF',7,'Right Visibility Guard',2,'',false);
wire(model, 'Zoom States',1,'Right Visibility Guard',3,'',false);
wire(model, 'Stereo Projection and Measurement',7,'Right Visibility Guard',4,'',false);
wire(model, 'Stereo Projection and Measurement',8,'Right Visibility Guard',5,'',false);
wire(model, 'Target EKF',12,'Right Visibility Guard',6,'',false);
wire(model, 'Stereo Validity Manager',2,'Right Visibility Guard',7,'rightReacquireActive',true);
wire(model, 'References',3,'Right Visibility Guard',8,'',false);
wire(model, 'Reset',1,'Right Visibility Guard',9,'',false);

wire(model, 'Zoom States',1,'Zoom Controller',1,'focalLengthMeasuredMm',true);
wire(model, 'Measured Feature Extraction',2,'Zoom Controller',2,'scaleMeasured',true);
wire(model, 'Target EKF',7,'Zoom Controller',3,'',false);
wire(model, 'Estimated Rho Dynamics',1,'Zoom Controller',4,'rhoDotHat',true);
wire(model, 'Right Visibility Guard',8,'Zoom Controller',5,'scaleDesiredEffective',true);
wire(model, 'Stereo Projection and Measurement',7,'Zoom Controller',6,'',false);
wire(model, 'Target EKF',12,'Zoom Controller',7,'',false);
wire(model, 'Zoom States',4,'Zoom Controller',8,'',false);
wire(model, 'Zoom States',5,'Zoom Controller',9,'',false);
wire(model, 'Right Visibility Guard',4,'Zoom Controller',10,'fRightMaxEffectiveMm',true);
wire(model, 'Right Visibility Guard',6,'Zoom Controller',11,'rightVisibilityActive',true);
wire(model, 'Stereo Validity Manager',2,'Zoom Controller',12,'',false);
wire(model, 'Zoom Controller',4,'gDot Terminator',1,'gDotCmd',false);

wire(model, 'Zoom Controller',3,'Zoom Priority Supervisor',1,'scaleError',true);
wire(model, 'Arm Priority Controller',3,'Depth Error Delay',1,'depthError',true);
wire(model, 'Depth Error Delay',1,'Zoom Priority Supervisor',2,'depthErrorDelayed',true);
wire(model, 'Zoom States',1,'Zoom Priority Supervisor',3,'focalLengthMeasuredMm',true);
wire(model, 'Zoom States',4,'Zoom Priority Supervisor',4,'',false);
wire(model, 'Zoom States',5,'Zoom Priority Supervisor',5,'',false);
wire(model, 'Zoom States',11,'Zoom Priority Supervisor',6,'',false);
wire(model, 'Zoom Controller',6,'Zoom Priority Supervisor',7,'focalFeasible',true);
wire(model, 'Stereo Projection and Measurement',7,'Zoom Priority Supervisor',8,'',false);
wire(model, 'Stereo Validity Manager',4,'Zoom Priority Supervisor',9,'',false);
wire(model, 'Target EKF',12,'Zoom Priority Supervisor',10,'',false);
wire(model, 'Reset',1,'Zoom Priority Supervisor',11,'',false);
wire(model, 'Zoom Controller',5,'Zoom Priority Supervisor',12,'',false);
wire(model, 'Zoom Controller',7,'Zoom Priority Supervisor',13,'',false);
wire(model, 'Zoom Controller',8,'Zoom Priority Supervisor',14,'',false);
wire(model, 'Stereo Validity Manager',2,'Zoom Priority Supervisor',15,'',false);

wire(model, 'Arm Priority Controller',1,'Safety and Saturation',1,'qDotCmd',true);
wire(model, 'Zoom Controller',2,'Safety and Saturation',2,'fDotMmLimited',true);
wire(model, 'Arm Joint States',1,'Safety and Saturation',3,'',false);
wire(model, 'Zoom States',1,'Safety and Saturation',4,'',false);
wire(model, 'Stereo Projection and Measurement',7,'Safety and Saturation',5,'',false);
wire(model, 'FR3 Camera Kinematics',5,'Safety and Saturation',6,'JL',true);

wire(model, 'Target EKF',1,'Compatibility Log Aliases',1,'',false);
wire(model, 'Target EKF',2,'Compatibility Log Aliases',2,'',false);
wire(model, 'Target EKF',3,'Compatibility Log Aliases',3,'',false);
wire(model, 'Target EKF',7,'Compatibility Log Aliases',4,'',false);
wire(model, 'Target EKF',12,'Compatibility Log Aliases',5,'',false);
wire(model, 'Stereo Projection and Measurement',7,'Compatibility Log Aliases',6,'',false);
wire(model, 'Stereo Projection and Measurement',8,'Compatibility Log Aliases',7,'',false);

% Logging branches. Truth signals are deliberately connected only to plant/logging.
logMap = {
    'Arm Joint States',1;
    'Arm Priority Controller',1;
    'Safety and Saturation',1;
    'FR3 Camera Kinematics',1;
    'FR3 Camera Kinematics',2;
    'FR3 Camera Kinematics',3;
    'FR3 Camera Kinematics',4;
    'FR3 Camera Kinematics',11;
    'FR3 Camera Kinematics',9;
    'Stereo Projection and Measurement',1;
    'Pixel Measurement Noise',1;
    'Stereo Projection and Measurement',2;
    'Measured Feature Extraction',1;
    'Measured Feature Extraction',2;
    'Arm Priority Controller',2;
    'Arm Priority Controller',3;
    'Zoom Controller',3;
    'Zoom States',2;
    'Zoom Controller',1;
    'Safety and Saturation',2;
    'Target Truth Generator',1;
    'Target Truth Generator',2;
    'Target Truth Generator',3;
    'Compatibility Log Aliases',1;
    'Compatibility Log Aliases',2;
    'Compatibility Log Aliases',3;
    'Stereo Projection and Measurement',4;
    'Compatibility Log Aliases',4;
    'Estimated Rho Dynamics',1;
    'Target EKF',8;
    'Target EKF',10;
    'Target EKF',11;
    'Compatibility Log Aliases',5;
    'Arm Priority Controller',4;
    'Arm Priority Controller',5;
    'FR3 Camera Kinematics',7;
    'FR3 Camera Kinematics',8;
    'Stereo Projection and Measurement',6;
    'Safety and Saturation',3;
    'Safety and Saturation',4;
    'Target EKF',15;
    'Target EKF',16;
    'FR3 Camera Kinematics',13;
    'FR3 Camera Kinematics',14;
    'FR3 Camera Kinematics',5;
    'FR3 Camera Kinematics',6;
    'FR3 Camera Kinematics',10;
    'FR3 Camera Kinematics',12;
    'Safety and Saturation',8;
    'Safety and Saturation',9;
    'Safety and Saturation',10;
    'Safety and Saturation',7;
    'Safety and Saturation',5;
    'Safety and Saturation',6;
    'Depth True Demux',1;
    'Depth True Demux',2;
    'Stereo Projection and Measurement',11;
    'Compatibility Log Aliases',6;
    'Compatibility Log Aliases',7;
    'Stereo Projection and Measurement',9;
    'Stereo Projection and Measurement',10;
    'Camera Mount Placeholder',1;
    'Camera Intrinsics Placeholder',1;
    'Stereo Projection and Measurement',12;
    'Stereo Projection and Measurement',13;
    'Zoom States',1;
    'Safety and Saturation',11;
    'Zoom Controller',2;
    'Safety and Saturation',2;
    'Zoom States',3;
    'Safety and Saturation',12;
    'Safety and Saturation',13;
    'Zoom States',4;
    'Zoom States',5;
    'Zoom States',11;
    'Zoom Controller',6;
    'Zoom Controller',5;
    'Zoom Priority Supervisor',2;
    'Zoom Priority Supervisor',1;
    'Zoom Priority Supervisor',3;
    'Zoom Priority Supervisor',4;
    'Zoom Priority Supervisor',5;
    'Zoom Priority Supervisor',6;
    'Zoom Priority Supervisor',7;
    'Arm Priority Controller',6;
    'Arm Priority Controller',7;
    'Arm Priority Controller',8;
    'Arm Priority Controller',9;
    'Zoom States',6;
    'Zoom States',7;
    'Zoom States',8;
    'Zoom States',9;
    'Zoom States',10;
    'Safety and Saturation',14;
    'Stereo Projection and Measurement',14;
    'Stereo Projection and Measurement',15;
    'Stereo Projection and Measurement',16;
    'Stereo Projection and Measurement',17;
    'Zoom States',12;
    'Zoom States',13;
    'Zoom States',14;
    'Zoom States',15;
    'Measured Feature Extraction',4;
    'Measured Feature Extraction',5;
    'Measured Feature Extraction',6;
    'Measured Feature Extraction',7;
    'Measured Feature Extraction',8;
    'Measured Feature Extraction',9;
    'Measured Feature Extraction',10;
    'Target EKF',4;
    'Target EKF',5;
    'Target EKF',6;
    'Target EKF',7;
    'Target EKF',9;
    'Target EKF',12;
    'Target EKF',13;
    'Target EKF',14;
    'Stereo Validity Manager',1;
    'Stereo Validity Manager',2;
    'Stereo Validity Manager',3;
    'Stereo Validity Manager',4;
    'Right Visibility Guard',1;
    'Right Visibility Guard',2;
    'Right Visibility Guard',3;
    'Right Visibility Guard',4;
    'Right Visibility Guard',9;
    'Right Visibility Guard',5;
    'Right Visibility Guard',6;
    'Right Visibility Guard',7;
    'Right Visibility Guard',10;
    'Stereo Projection and Measurement',7;
    'Stereo Projection and Measurement',8;
    'Target EKF',1;
    'Target EKF',2;
    'Target EKF',3};
for k = 1:numel(logNames)
    wire(model, logMap{k,1}, logMap{k,2}, 'Logging and Diagnostics', k, logNames{k}, true);
end

% Replace the simulation plant and truth sources with the production ROS 2
% interface.  The estimator/controllers below are retained unchanged.
convertModelToROS2(model);

% Helpful annotation and layout.
annotationText = sprintf(['URDF-derived FR3 kinematics + rigid stereo + pixel EKF + damped approximate-priority IBVS\n' ...
    'V=[vx vy vz wx wy wz]^T; RWCi maps camera coordinates to world.\n' ...
    'Production control uses EKF velocity/depth only; truth paths terminate at plant/logging.']);
ann = Simulink.Annotation(model, string(annotationText));
ann.Position = [1055 265 2000 325];

save_system(model);
set_param(model, 'SimulationCommand', 'update');
logmsg(logId, 'Update Diagram successful after complete signal graph.');

% Explicit positions above preserve the intended left-to-right architecture.
% Automatic whole-model arrangement makes the many diagnostic branches less readable.
set_param(model, 'SimulationCommand', 'update');
save_system(model);

resultDir = fullfile(pwd, 'test_results');
if ~isfolder(resultDir)
    mkdir(resultDir);
end
try
    print(['-s' model], '-dpng', '-r150', ...
        fullfile(resultDir, 'arm_stereo_ibvs_ekf_v1_overview.png'));
    logmsg(logId, 'Model overview image exported.');
catch ME
    logmsg(logId, ['Overview export warning: ' ME.message]);
end

close_system(model, 0);
logmsg(logId, 'BUILD COMPLETE');
fprintf('Built %s.slx successfully.\n', model);
end

function addMFunctionSubsystem(model, name, code, inNames, outNames, position)
path = [model '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
clearSystemContents(path);
nIn = numel(inNames);
nOut = numel(outNames);
height = max(80, 35*max(nIn,nOut));
fnPath = [path '/MATLAB Function'];
add_block('simulink/User-Defined Functions/MATLAB Function', fnPath, ...
    'Position', [135 30 310 30+height]);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', fnPath);
[code, parameterNames] = flattenConfigParameters(code);
chart.Script = code;
chart.SupportVariableSizing = false;
for p = 1:numel(parameterNames)
    data = Stateflow.Data(chart);
    data.Name = parameterNames{p};
    data.Scope = 'Parameter';
    data.DataType = 'double';
    parameterValue = evalin('base', parameterNames{p});
    data.Props.Array.Size = mat2str(size(parameterValue));
end
for k = 1:nIn
    add_block('simulink/Ports & Subsystems/In1', [path '/' inNames{k}], ...
        'Port', num2str(k), 'Position', [25 35*k 55 35*k+14]);
    add_line(path, [inNames{k} '/1'], sprintf('MATLAB Function/%d',k), ...
        'autorouting', 'on');
end
for k = 1:nOut
    add_block('simulink/Ports & Subsystems/Out1', [path '/' outNames{k}], ...
        'Port', num2str(k), 'Position', [390 35*k 420 35*k+14]);
    add_line(path, sprintf('MATLAB Function/%d',k), [outNames{k} '/1'], ...
        'autorouting', 'on');
end
end

function convertModelToROS2(model)
% Convert the closed-loop simulation into the deployable ROS 2 controller.
load_system('ros2lib');

deleteNames = {
    'Simulation Time','Experiment Mode','Target Truth Generator', ...
    'Stereo Projection and Measurement','Pixel Measurement Noise', ...
    'Arm Joint States','Zoom States','Depth True Terminator', ...
    'Depth True Demux','Camera Mount Placeholder', ...
    'Camera Intrinsics Placeholder','Compatibility Log Aliases', ...
    'Logging and Diagnostics'};
for k = 1:numel(deleteNames)
    path = [model '/' deleteNames{k}];
    if getSimulinkBlockHandle(path) ~= -1
        delete_block(path);
    end
end

% ROS 2 subscribers.
add_block('ros2lib/Subscribe',[model '/ROS Vision Subscribe'], ...
    'topic','cfg.ros2VisionTopic','messageType','std_msgs/Float64MultiArray', ...
    'sampleTime','cfg.Ts','Position',[25 35 185 95]);
add_block('simulink/Signal Routing/Bus Selector',[model '/Vision Data'], ...
    'OutputSignals','Data','Position',[215 50 220 80]);
add_block('simulink/Signal Routing/Selector',[model '/Vision First 8'], ...
    'IndexOptionArray',{'Index vector (dialog)'},'Indices','1:8', ...
    'InputPortWidth','128','Position',[255 50 315 80]);
addMFunctionSubsystem(model,'ROS Vision Decoder',rosVisionDecoderCode(), ...
    {'features','isNew'},{'pixelMeasurement','scaleMeasured','validLeft','validRight','visionFresh'}, ...
    [350 25 555 160]);

add_block('ros2lib/Subscribe',[model '/ROS Joint State Subscribe'], ...
    'topic','cfg.ros2JointStateTopic','messageType','sensor_msgs/JointState', ...
    'sampleTime','cfg.Ts','Position',[25 200 185 260]);
add_block('simulink/Signal Routing/Bus Selector',[model '/Joint Position'], ...
    'OutputSignals','Position','Position',[215 215 220 245]);
add_block('simulink/Signal Routing/Selector',[model '/Joint First 7'], ...
    'IndexOptionArray',{'Index vector (dialog)'},'Indices','1:7', ...
    'InputPortWidth','128','Position',[255 215 315 245]);
addMFunctionSubsystem(model,'ROS Joint Decoder',rosJointDecoderCode(), ...
    {'position','isNew'},{'q','jointFresh'},[350 190 555 275]);

add_block('ros2lib/Subscribe',[model '/ROS Focal State Subscribe'], ...
    'topic','cfg.ros2FocalStateTopic','messageType','std_msgs/Float64MultiArray', ...
    'sampleTime','cfg.Ts','Position',[25 355 185 415]);
add_block('simulink/Signal Routing/Bus Selector',[model '/Focal Data'], ...
    'OutputSignals','Data','Position',[215 370 220 400]);
add_block('simulink/Signal Routing/Selector',[model '/Focal First 2'], ...
    'IndexOptionArray',{'Index vector (dialog)'},'Indices','1:2', ...
    'InputPortWidth','128','Position',[255 370 315 400]);
addMFunctionSubsystem(model,'ROS Focal Decoder',rosFocalDecoderCode(), ...
    {'focalMm','isNew'},{'focalLengthMeasuredMm','focalFresh'},[350 345 555 430]);
addMFunctionSubsystem(model,'ROS Input Validity',rosValidityCode(), ...
    {'validLeftRaw','validRightRaw','visionFresh','jointFresh','focalFresh'}, ...
    {'validLeft','validRight'},[610 175 790 300]);
addMFunctionSubsystem(model,'Focal State Calibration',zoomStateCode(), ...
    {'fMm'},{'focalLengthMeasuredMm','fxMeasuredPx','fyMeasuredPx', ...
    'focalAtWorkingLowerLimit','focalAtWorkingUpperLimit', ...
    'focalAtHardwareLowerLimit','focalAtHardwareUpperLimit', ...
    'focalOutsideWorkingRange','focalHardwareLimitViolation','focalRangeUsage', ...
    'focalHeadroomMm','focalLengthWorkingMinMm','focalLengthWorkingMaxMm', ...
    'focalLengthHardwareMinMm','focalLengthHardwareMaxMm'},[825 330 1040 700]);

% Subscriber wiring.
wire(model,'ROS Vision Subscribe',2,'Vision Data',1,'',false);
wire(model,'Vision Data',1,'Vision First 8',1,'',false);
wire(model,'Vision First 8',1,'ROS Vision Decoder',1,'',false);
wire(model,'ROS Vision Subscribe',1,'ROS Vision Decoder',2,'',false);
wire(model,'ROS Joint State Subscribe',2,'Joint Position',1,'',false);
wire(model,'Joint Position',1,'Joint First 7',1,'',false);
wire(model,'Joint First 7',1,'ROS Joint Decoder',1,'',false);
wire(model,'ROS Joint State Subscribe',1,'ROS Joint Decoder',2,'',false);
wire(model,'ROS Focal State Subscribe',2,'Focal Data',1,'',false);
wire(model,'Focal Data',1,'Focal First 2',1,'',false);
wire(model,'Focal First 2',1,'ROS Focal Decoder',1,'',false);
wire(model,'ROS Focal State Subscribe',1,'ROS Focal Decoder',2,'',false);
wire(model,'ROS Vision Decoder',3,'ROS Input Validity',1,'',false);
wire(model,'ROS Vision Decoder',4,'ROS Input Validity',2,'',false);
wire(model,'ROS Vision Decoder',5,'ROS Input Validity',3,'',false);
wire(model,'ROS Joint Decoder',2,'ROS Input Validity',4,'',false);
wire(model,'ROS Focal Decoder',2,'ROS Input Validity',5,'',false);
wire(model,'ROS Focal Decoder',1,'Focal State Calibration',1,'',false);

% Real measurements replace simulation truth/noise paths.
wireReplace(model,'ROS Joint Decoder',1,'FR3 Camera Kinematics',1,'q');
wireReplace(model,'ROS Joint Decoder',1,'Arm Priority Controller',1,'');
wireReplace(model,'ROS Joint Decoder',1,'Safety and Saturation',3,'');
wireReplace(model,'ROS Vision Decoder',1,'Measured Feature Extraction',1,'');
wireReplace(model,'Focal State Calibration',2,'Measured Feature Extraction',2,'');
wireReplace(model,'ROS Vision Decoder',2,'Measured Feature Extraction',3,'');
wireReplace(model,'ROS Input Validity',1,'Measured Feature Extraction',4,'');
wireReplace(model,'ROS Input Validity',2,'Measured Feature Extraction',5,'');
wireReplace(model,'ROS Input Validity',1,'Stereo Validity Manager',1,'');
wireReplace(model,'ROS Input Validity',2,'Stereo Validity Manager',2,'');
wireReplace(model,'ROS Input Validity',1,'Arm Priority Controller',8,'');
wireReplace(model,'ROS Input Validity',1,'Right Visibility Guard',4,'');
wireReplace(model,'ROS Input Validity',2,'Right Visibility Guard',5,'');
wireReplace(model,'ROS Input Validity',1,'Zoom Controller',6,'');
wireReplace(model,'ROS Input Validity',1,'Zoom Priority Supervisor',8,'');
wireReplace(model,'ROS Input Validity',1,'Safety and Saturation',5,'');

% Focal-state fanout (same port contract as the removed simulation state).
wireReplace(model,'Focal State Calibration',1,'Right Visibility Guard',3,'');
wireReplace(model,'Focal State Calibration',1,'Zoom Controller',1,'');
wireReplace(model,'Focal State Calibration',4,'Zoom Controller',8,'');
wireReplace(model,'Focal State Calibration',5,'Zoom Controller',9,'');
wireReplace(model,'Focal State Calibration',1,'Zoom Priority Supervisor',3,'');
wireReplace(model,'Focal State Calibration',4,'Zoom Priority Supervisor',4,'');
wireReplace(model,'Focal State Calibration',5,'Zoom Priority Supervisor',5,'');
wireReplace(model,'Focal State Calibration',11,'Zoom Priority Supervisor',6,'');
wireReplace(model,'Focal State Calibration',1,'Safety and Saturation',4,'');
wireReplace(model,'Focal State Calibration',2,'Target EKF',5,'');

% Convert the retained joint-space law to the left-camera twist expected by
% the existing Python mapper: Vc = JL*qDotApplied.
addMFunctionSubsystem(model,'Camera Velocity Output',cameraVelocityCode(), ...
    {'qDotApplied','JL','validLeft'},{'cameraVelocity'},[3100 160 3290 255]);
wire(model,'Safety and Saturation',1,'Camera Velocity Output',1,'',false);
wire(model,'FR3 Camera Kinematics',5,'Camera Velocity Output',2,'',false);
wire(model,'ROS Input Validity',1,'Camera Velocity Output',3,'',false);

addROSFloat64Publisher(model,'Camera Velocity Output',1, ...
    'ROS Camera Velocity Message','ROS Camera Velocity Publish', ...
    'cfg.ros2CameraVelocityTopic',[3340 130 3650 255]);
addROSFloat64Publisher(model,'Safety and Saturation',2, ...
    'ROS Zoom Velocity Message','ROS Zoom Velocity Publish', ...
    'cfg.ros2ZoomVelocityTopic',[3340 315 3650 440]);

set_param(model,'StopTime','inf','SignalLogging','off');
end

function addROSFloat64Publisher(model,src,srcPort,msgName,pubName,topic,position)
blank=[model '/' msgName ' Blank']; assign=[model '/' msgName]; pub=[model '/' pubName];
add_block('ros2lib/Blank Message',blank,'messageType','std_msgs/Float64MultiArray', ...
    'SampleTime','cfg.Ts','Position',[position(1) position(2) position(1)+120 position(2)+45]);
add_block('simulink/Signal Routing/Bus Assignment',assign,'AssignedSignals','Data', ...
    'Position',[position(1)+155 position(2) position(1)+225 position(2)+70]);
add_block('ros2lib/Publish',pub,'topic',topic,'messageType','std_msgs/Float64MultiArray', ...
    'Position',[position(1)+260 position(2) position(1)+410 position(2)+60]);
wire(model,[msgName ' Blank'],1,msgName,1,'',false);
wire(model,src,srcPort,msgName,2,'',false);
wire(model,msgName,1,pubName,1,'',false);
end

function code=rosVisionDecoderCode()
code=sprintf(['function [pixelMeasurement,scaleMeasured,validLeft,validRight,visionFresh]=fcn(features,isNew)\n' ...
    '%%#codegen\npersistent age last\nif isempty(age), age=inf; last=zeros(8,1); end\n' ...
    'if isNew, last=reshape(features,8,1); age=0; else, age=age+cfg.Ts; end\n' ...
    'visionFresh=double(age<=cfg.ros2InputTimeoutSec);\n' ...
    'validLeft=double(visionFresh>0.5 && last(1)>0.5); validRight=double(visionFresh>0.5 && last(2)>0.5);\n' ...
    'pixelMeasurement=[last(3);last(4);last(5);last(6)]; scaleMeasured=[last(7);last(8)];\n' ...
    'if any(~isfinite([pixelMeasurement;scaleMeasured])), pixelMeasurement=zeros(4,1); scaleMeasured=zeros(2,1); validLeft=0; validRight=0; end\nend']);
end

function code=rosJointDecoderCode()
code=sprintf(['function [q,jointFresh]=fcn(position,isNew)\n%%#codegen\npersistent age last\n' ...
    'if isempty(age), age=inf; last=cfg.q0; end\nif isNew, candidate=reshape(position,7,1); if all(isfinite(candidate)), last=candidate; age=0; end, else, age=age+cfg.Ts; end\n' ...
    'q=last; jointFresh=double(age<=cfg.ros2InputTimeoutSec);\nend']);
end

function code=rosFocalDecoderCode()
code=sprintf(['function [focalLengthMeasuredMm,focalFresh]=fcn(focalMm,isNew)\n%%#codegen\npersistent age last\n' ...
    'if isempty(age), age=inf; last=cfg.focalLength0Mm; end\nif isNew, candidate=reshape(focalMm,2,1); if all(isfinite(candidate)) && all(candidate>0), last=candidate; age=0; end, else, age=age+cfg.Ts; end\n' ...
    'focalLengthMeasuredMm=last; focalFresh=double(age<=cfg.ros2InputTimeoutSec);\nend']);
end

function code=rosValidityCode()
code=sprintf(['function [validLeft,validRight]=fcn(validLeftRaw,validRightRaw,visionFresh,jointFresh,focalFresh)\n%%#codegen\n' ...
    'ready=visionFresh>0.5 && jointFresh>0.5 && focalFresh>0.5; validLeft=double(ready && validLeftRaw>0.5); validRight=double(ready && validRightRaw>0.5);\nend']);
end

function code=cameraVelocityCode()
code=sprintf(['function cameraVelocity=fcn(qDotApplied,JL,validLeft)\n%%#codegen\n' ...
    'cameraVelocity=zeros(6,1); if validLeft>0.5 && all(isfinite(qDotApplied)) && all(isfinite(JL(:))), cameraVelocity=JL*qDotApplied; end\n' ...
    'if any(~isfinite(cameraVelocity)), cameraVelocity=zeros(6,1); end\nend']);
end

function addJointStateSubsystem(model, position)
path = [model '/Arm Joint States'];
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
clearSystemContents(path);
add_block('simulink/Ports & Subsystems/In1', [path '/qDotApplied'], ...
    'Position', [25 53 55 67]);
add_block('simulink/Discrete/Discrete-Time Integrator', [path '/q integrator'], ...
    'gainval', '1', 'SampleTime', 'cfg.Ts', 'InitialCondition', 'cfg.q0', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'cfg.qMax', ...
    'LowerSaturationLimit', 'cfg.qMin', 'Position', [110 38 160 82]);
add_block('simulink/Ports & Subsystems/Out1', [path '/q'], ...
    'Position', [225 53 255 67]);
add_line(path, 'qDotApplied/1', 'q integrator/1');
add_line(path, 'q integrator/1', 'q/1');
end

function addZoomStateSubsystem(model, position)
path = [model '/Zoom States'];
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
clearSystemContents(path);
add_block('simulink/Ports & Subsystems/In1', [path '/fDotMmApplied'], ...
    'Position', [25 53 55 67]);
add_block('simulink/Discrete/Discrete-Time Integrator', [path '/focal length mm integrator'], ...
    'gainval', '1', 'SampleTime', 'cfg.Ts', 'InitialCondition', 'cfg.focalLength0Mm', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'cfg.focalLengthHardwareMaxMm', ...
    'LowerSaturationLimit', 'cfg.focalLengthHardwareMinMm', 'Position', [90 38 150 82]);
fn=[path '/Zoom Calibration'];
add_block('simulink/User-Defined Functions/MATLAB Function',fn,'Position',[185 25 345 120]);
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',fn);
[code,params]=flattenConfigParameters(zoomStateCode()); chart.Script=code; chart.SupportVariableSizing=false;
for p=1:numel(params), d=Stateflow.Data(chart); d.Name=params{p}; d.Scope='Parameter'; d.DataType='double'; v=evalin('base',params{p}); d.Props.Array.Size=mat2str(size(v)); end
outs={'focalLengthMeasuredMm','fxMeasuredPx','fyMeasuredPx','focalAtWorkingLowerLimit','focalAtWorkingUpperLimit', ...
    'focalAtHardwareLowerLimit','focalAtHardwareUpperLimit','focalOutsideWorkingRange','focalHardwareLimitViolation','focalRangeUsage','focalHeadroomMm', ...
    'focalLengthWorkingMinMm','focalLengthWorkingMaxMm','focalLengthHardwareMinMm','focalLengthHardwareMaxMm'};
for k=1:numel(outs), add_block('simulink/Ports & Subsystems/Out1',[path '/' outs{k}], ...
        'Port',num2str(k),'Position',[400 20+28*k 430 34+28*k]); add_line(path,sprintf('Zoom Calibration/%d',k),[outs{k} '/1'],'autorouting','on'); end
add_line(path,'fDotMmApplied/1','focal length mm integrator/1');
add_line(path,'focal length mm integrator/1','Zoom Calibration/1','autorouting','on');
end

function addReferenceSubsystem(model, position)
path = [model '/References'];
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
clearSystemContents(path);
names = {'cLd','rhoD','scaleDesired'};
values = {'[0;0]','cfg.rhoD','cfg.scaleDesired'};
for k = 1:3
    add_block('simulink/Sources/Constant',[path '/' names{k} ' value'], ...
        'Value',values{k},'Position',[35 35*k 100 35*k+20]);
    add_block('simulink/Ports & Subsystems/Out1',[path '/' names{k}], ...
        'Port',num2str(k),'Position',[180 35*k 210 35*k+14]);
    add_line(path,[names{k} ' value/1'],[names{k} '/1']);
end
end

function addLoggingSubsystem(model, names, position)
path = [model '/Logging and Diagnostics'];
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
clearSystemContents(path);
for k = 1:numel(names)
    y = 20 + 25*k;
    add_block('simulink/Ports & Subsystems/In1', [path '/' names{k}], ...
        'Port',num2str(k),'Position',[20 y 50 y+14]);
    add_block('simulink/Sinks/Terminator', [path '/term_' names{k}], ...
        'Position',[120 y 140 y+14]);
    add_line(path,[names{k} '/1'],['term_' names{k} '/1']);
end
end

function wire(model, src, srcPort, dst, dstPort, signalName, logSignal)
h = add_line(model, sprintf('%s/%d',src,srcPort), sprintf('%s/%d',dst,dstPort), ...
    'autorouting','on');
if ~isempty(signalName)
    set_param(h,'Name',signalName);
end
if logSignal
    ph = get_param([model '/' src], 'PortHandles');
    outHandle = ph.Outport(srcPort);
    set_param(outHandle,'DataLogging','on');
    if ~isempty(signalName)
        set_param(outHandle,'DataLoggingNameMode','Custom', ...
            'DataLoggingName',signalName);
    end
end
end

function clearSystemContents(systemPath)
blocks = find_system(systemPath, 'SearchDepth', 1, 'Type', 'Block');
blocks(strcmp(blocks, systemPath)) = [];
for k = numel(blocks):-1:1
    delete_block(blocks{k});
end
annotations = find_system(systemPath, 'SearchDepth', 1, 'FindAll', 'on', ...
    'Type', 'Annotation');
for k = 1:numel(annotations)
    delete(annotations(k));
end
end

function [code, parameterNames] = flattenConfigParameters(code)
tokens = regexp(code, 'cfg\.([A-Za-z][A-Za-z0-9_]*)', 'tokens');
parameterNames = cell(size(tokens));
for k = 1:numel(tokens)
    parameterNames{k} = ['cfg_' tokens{k}{1}];
end
parameterNames = unique(parameterNames, 'stable');
code = regexprep(code, 'cfg\.([A-Za-z][A-Za-z0-9_]*)', 'cfg_$1');
end

function logmsg(fid, message)
fprintf(fid, '[%s] %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), message);
end

function code=zoomStateCode
code=strjoin({
    'function [focalLengthMeasuredMm,fxMeasuredPx,fyMeasuredPx,focalAtWorkingLowerLimit,focalAtWorkingUpperLimit,focalAtHardwareLowerLimit,focalAtHardwareUpperLimit,focalOutsideWorkingRange,focalHardwareLimitViolation,focalRangeUsage,focalHeadroomMm,focalLengthWorkingMinMm,focalLengthWorkingMaxMm,focalLengthHardwareMinMm,focalLengthHardwareMaxMm]=fcn(fMm)'
    '%#codegen'
    'focalLengthMeasuredMm=min(max(fMm,cfg.focalLengthHardwareMinMm),cfg.focalLengthHardwareMaxMm);'
    'fxMeasuredPx=focalLengthMeasuredMm/cfg.outputPixelPitchXmm; fyMeasuredPx=focalLengthMeasuredMm/cfg.outputPixelPitchYmm;'
    'wspan=cfg.focalLengthWorkingMaxMm-cfg.focalLengthWorkingMinMm; hspan=cfg.focalLengthHardwareMaxMm-cfg.focalLengthHardwareMinMm; margin=cfg.zoomLimitMarginFraction*wspan;'
    'focalAtWorkingLowerLimit=double(focalLengthMeasuredMm<=cfg.focalLengthWorkingMinMm+margin); focalAtWorkingUpperLimit=double(focalLengthMeasuredMm>=cfg.focalLengthWorkingMaxMm-margin);'
    'focalAtHardwareLowerLimit=double(focalLengthMeasuredMm<=cfg.focalLengthHardwareMinMm+cfg.numericalEpsilon); focalAtHardwareUpperLimit=double(focalLengthMeasuredMm>=cfg.focalLengthHardwareMaxMm-cfg.numericalEpsilon);'
    'focalOutsideWorkingRange=double(focalLengthMeasuredMm<cfg.focalLengthWorkingMinMm|focalLengthMeasuredMm>cfg.focalLengthWorkingMaxMm); focalHardwareLimitViolation=double(focalLengthMeasuredMm<cfg.focalLengthHardwareMinMm|focalLengthMeasuredMm>cfg.focalLengthHardwareMaxMm);'
    'focalRangeUsage=(focalLengthMeasuredMm-cfg.focalLengthWorkingMinMm)./max(wspan,cfg.numericalEpsilon*ones(2,1)); focalHeadroomMm=min(focalLengthMeasuredMm-cfg.focalLengthWorkingMinMm,cfg.focalLengthWorkingMaxMm-focalLengthMeasuredMm);'
    'focalLengthWorkingMinMm=cfg.focalLengthWorkingMinMm; focalLengthWorkingMaxMm=cfg.focalLengthWorkingMaxMm; focalLengthHardwareMinMm=cfg.focalLengthHardwareMinMm; focalLengthHardwareMaxMm=cfg.focalLengthHardwareMaxMm;'
    'end'},newline);
end

function code=compatibilityLogAliasCode
code=strjoin({
    'function [targetPositionHatW,targetVelocityHatW,targetAccelerationHatW,rhoHat,ekfValid,leftCameraVisible,rightCameraVisible]=fcn(pHatWorld,vHatWorld,aHatWorld,rhoHatSafe,ekfPredictionValid,validLeft,validRight)'
    '%#codegen'
    '% Compatibility-only aliases. These outputs do not feed any controller.'
    'targetPositionHatW=pHatWorld; targetVelocityHatW=vHatWorld; targetAccelerationHatW=aHatWorld; rhoHat=rhoHatSafe; ekfValid=ekfPredictionValid; leftCameraVisible=validLeft; rightCameraVisible=validRight;'
    'end'},newline);
end

function code = targetTruthCode
code = strjoin({
    'function [pTargetTrueW,vTargetTrueW,aTargetTrueW] = fcn(t,mode)'
    '%#codegen'
    '% Trajectory is defined in the fixed initial left-camera frame, then mapped to world.'
    'pCL=zeros(3,1); vCL=zeros(3,1); aCL=zeros(3,1);'
    'm = floor(mode + 0.5);'
    'mid=0.5*cfg.baseline;'
    'if m == 0'
    '    pCL=[mid;0;0.75];'
    'elseif m == 1'
    '    pCL=[mid+0.015*sin(0.8*t);0.01*sin(0.6*t);0.75];'
    '    vCL=[0.012*cos(0.8*t);0.006*cos(0.6*t);0];'
    '    aCL=[-0.0096*sin(0.8*t);-0.0036*sin(0.6*t);0];'
    'elseif m == 2'
    '    pCL=[mid;0;0.75+0.20*sin(0.4*t)];'
    '    vCL=[0;0;0.08*cos(0.4*t)];'
    '    aCL=[0;0;-0.032*sin(0.4*t)];'
    'elseif m == 3'
    '    pCL=[mid+0.015*sin(0.8*t);0.01*sin(0.6*t);0.75+0.20*sin(0.4*t)];'
    '    vCL=[0.012*cos(0.8*t);0.006*cos(0.6*t);0.08*cos(0.4*t)];'
    '    aCL=[-0.0096*sin(0.8*t);-0.0036*sin(0.6*t);-0.032*sin(0.4*t)];'
    'else'
    '    if t<3, z=0.75; else, z=0.90; end'
    '    pCL=[mid;0;z]; vCL=zeros(3,1); aCL=zeros(3,1);'
    'end'
    'sid=floor(cfg.gainTuningScenarioId+0.5);'
    'if sid==1, pCL=[0.08;0.04;0.75]; vCL=zeros(3,1); aCL=zeros(3,1);'
    'elseif sid==2, pCL=[0.10*sin(0.8*t);0;0.75]; vCL=[0.08*cos(0.8*t);0;0]; aCL=[-0.064*sin(0.8*t);0;0];'
    'elseif sid==3, pCL=[0;0.08*sin(0.7*t);0.75]; vCL=[0;0.056*cos(0.7*t);0]; aCL=[0;-0.0392*sin(0.7*t);0];'
    'elseif sid==4, pCL=[0.08*sin(0.8*t);0.06*sin(0.6*t);0.75]; vCL=[0.064*cos(0.8*t);0.036*cos(0.6*t);0]; aCL=[-0.0512*sin(0.8*t);-0.0216*sin(0.6*t);0];'
    'elseif sid==5, if t<2,pCL=[0;0;0.75];vCL=zeros(3,1);else,pCL=[0.06*(t-2);0;0.75];vCL=[0.06;0;0];end;aCL=zeros(3,1);'
    'elseif sid==6, pCL=[0.10*sin(t);0;0.75];vCL=[0.10*cos(t);0;0];aCL=[-0.10*sin(t);0;0];'
    'elseif sid==7, pCL=[0.10*sin(0.8*t);0.06*sin(0.6*t);0.75+0.20*sin(0.4*t)];vCL=[0.08*cos(0.8*t);0.036*cos(0.6*t);0.08*cos(0.4*t)];aCL=[-0.064*sin(0.8*t);-0.0216*sin(0.6*t);-0.032*sin(0.4*t)];'
    'elseif sid==8, pCL=[0.04;0.02;0.50];vCL=zeros(3,1);aCL=zeros(3,1);'
    'elseif sid==9, pCL=[0.04;0.02;1.00];vCL=zeros(3,1);aCL=zeros(3,1);'
    'elseif sid==10, pCL=[0;0;0.75+0.20*sin(0.4*t)];vCL=[0;0;0.08*cos(0.4*t)];aCL=[0;0;-0.032*sin(0.4*t)];'
    'elseif sid==11, if t<3,z=0.75;else,z=0.90;end;pCL=[0.08;0.04;z];vCL=zeros(3,1);aCL=zeros(3,1);'
    'elseif sid==12, pCL=[0.18;0.10;0.75];vCL=zeros(3,1);aCL=zeros(3,1);'
    'elseif sid==13, pCL=[0.30*sin(0.5*t);0.18*sin(0.4*t);0.75];vCL=[0.15*cos(0.5*t);0.072*cos(0.4*t);0];aCL=[-0.075*sin(0.5*t);-0.0288*sin(0.4*t);0];'
    'elseif sid==14, if t<2||t>=3.5,pCL=[mid;0;0.75];else,pCL=[-0.03;0;0.75];end;vCL=zeros(3,1);aCL=zeros(3,1);'
    'elseif sid==15, pCL=[mid+0.035*sin(0.55*t);0.02*sin(0.7*t);0.75+0.18*sin(0.32*t)];vCL=[0.01925*cos(0.55*t);0.014*cos(0.7*t);0.0576*cos(0.32*t)];aCL=[-0.0105875*sin(0.55*t);-0.0098*sin(0.7*t);-0.018432*sin(0.32*t)];'
    'elseif sid==16, pCL=[0;0;0.50];vCL=zeros(3,1);aCL=zeros(3,1);'
    'elseif sid==17, pCL=[0;0;0.75];vCL=zeros(3,1);aCL=zeros(3,1);'
    'elseif sid==18, pCL=[0;0;1.00];vCL=zeros(3,1);aCL=zeros(3,1); end'
    'pTargetTrueW=cfg.pWCL0+cfg.RWCL0*pCL;'
    'vTargetTrueW=cfg.RWCL0*vCL; aTargetTrueW=cfg.RWCL0*aCL;'
    'end'}, newline);
end

function code = fr3KinematicsCode
code = strjoin({
    'function [pWCL,RWCL,pWCR,RWCR,JL,JR,rankJL,rcondJL,rotationOrthogonalityError,baselineLength,baselineError,stereoRotationError,pWLink8,RWLink8] = fcn(q)'
    '%#codegen'
    '% URDF-derived FK and optical-center Jacobians; no pose integration.'
    'pWCL=zeros(3,1); RWCL=eye(3); pWCR=zeros(3,1); RWCR=eye(3); JL=zeros(6,7); JR=zeros(6,7); rankJL=0; rcondJL=0; rotationOrthogonalityError=0; baselineLength=0; baselineError=0; stereoRotationError=0; pWLink8=zeros(3,1); RWLink8=eye(3);'
    'T=cfg.T_W_B; jointOrigins=zeros(3,7); jointAxes=zeros(3,7);'
    'for i=1:7'
    '    xyz=cfg.fr3OriginXYZ(:,i); rpy=cfg.fr3OriginRPY(:,i);'
    '    cr=cos(rpy(1));sr=sin(rpy(1));cp=cos(rpy(2));sp=sin(rpy(2));cy=cos(rpy(3));sy=sin(rpy(3));'
    '    Rx=[1 0 0;0 cr -sr;0 sr cr]; Ry=[cp 0 sp;0 1 0;-sp 0 cp]; Rz=[cy -sy 0;sy cy 0;0 0 1]; To=eye(4); To(1:3,1:3)=Rz*Ry*Rx; To(1:3,4)=xyz; T=T*To;'
    '    jointOrigins(:,i)=T(1:3,4); axis=cfg.fr3Axis(:,i); jointAxes(:,i)=T(1:3,1:3)*axis;'
    '    u=axis/max(norm(axis),eps); K=[0 -u(3) u(2);u(3) 0 -u(1);-u(2) u(1) 0]; Tr=eye(4); Tr(1:3,1:3)=eye(3)+sin(q(i))*K+(1-cos(q(i)))*(K*K); T=T*Tr;'
    'end'
    'xyz=cfg.fr3Joint8OriginXYZ; rpy=cfg.fr3Joint8OriginRPY; cr=cos(rpy(1));sr=sin(rpy(1));cp=cos(rpy(2));sp=sin(rpy(2));cy=cos(rpy(3));sy=sin(rpy(3)); Rx=[1 0 0;0 cr -sr;0 sr cr]; Ry=[cp 0 sp;0 1 0;-sp 0 cp]; Rz=[cy -sy 0;sy cy 0;0 0 1]; To=eye(4); To(1:3,1:3)=Rz*Ry*Rx; To(1:3,4)=xyz; T=T*To;'
    'pWLink8=T(1:3,4); RWLink8=T(1:3,1:3); TCL=T*cfg.T_link8_CL; pWCL=TCL(1:3,4); RWCL=TCL(1:3,1:3); TCR=TCL*cfg.T_CL_CR; pWCR=TCR(1:3,4); RWCR=TCR(1:3,1:3);'
    'JW=zeros(6,7); for i=1:7, JW(1:3,i)=cross(jointAxes(:,i),pWCL-jointOrigins(:,i)); JW(4:6,i)=jointAxes(:,i); end'
    'RCLW=RWCL''; JL=[RCLW zeros(3,3);zeros(3,3) RCLW]*JW; R=cfg.R_CL_CR; p=cfg.p_CL_CR; S=[0 -p(3) p(2);p(3) 0 -p(1);-p(2) p(1) 0]; JR=[R'' -R''*S;zeros(3,3) R'']*JL;'
    'rankJL=rank(JL); rcondJL=rcond(JL*JL''); rotationOrthogonalityError=norm(RWCL''*RWCL-eye(3),''fro''); baselineLength=norm(pWCR-pWCL); baselineError=abs(baselineLength-norm(cfg.p_CL_CR)); stereoRotationError=norm(RWCR-RWCL*cfg.R_CL_CR,''fro'');'
    'end'}, newline);
end

function code = projectionCode
code = strjoin({
    'function [pixelMeasurementTrue,normalizedFeatureTrue,scaleTrue,rhoTrue,depthTrue,validStereo,leftCameraVisible,rightCameraVisible,leftImageMargin,rightImageMargin,targetDepthRangeViolation,desiredFocalLengthEstimate,focalFeasibilityFlag,focalVisibilityLimited,focalScaleLimited,focalEffectiveUpperLimitMm,focalRequiredMm] = fcn(pTargetTrueW,pWCL,RWCL,pWCR,RWCR,fMeasured)'
    '%#codegen'
    '% Fixed sizes: world points [3x1], rotations [3x3], focal length [2x1].'
    'pixelMeasurementTrue=zeros(4,1); normalizedFeatureTrue=zeros(4,1); scaleTrue=zeros(2,1); rhoTrue=zeros(2,1); depthTrue=zeros(2,1); validStereo=0; leftCameraVisible=0; rightCameraVisible=0; leftImageMargin=-inf; rightImageMargin=-inf; targetDepthRangeViolation=0; desiredFocalLengthEstimate=zeros(2,1); focalFeasibilityFlag=0; focalVisibilityLimited=0; focalScaleLimited=0; focalEffectiveUpperLimitMm=cfg.focalLengthWorkingMaxMm; focalRequiredMm=zeros(2,1);'
    'pL = RWCL''*(pTargetTrueW-pWCL); pR = RWCR''*(pTargetTrueW-pWCR);'
    'zL=pL(3); zR=pR(3); safeZL=max(zL,cfg.visibilityZMin); safeZR=max(zR,cfg.visibilityZMin);'
    'xL=pL(1)/safeZL; yL=pL(2)/safeZL; xR=pR(1)/safeZR; yR=pR(2)/safeZR;'
    'uL=cfg.cxL+fMeasured(1)*xL; vL=cfg.cyL+fMeasured(1)*yL;'
    'uR=cfg.cxR+fMeasured(2)*xR; vR=cfg.cyR+fMeasured(2)*yR;'
    'pixelMeasurementTrue=[uL;vL;uR;vR]; normalizedFeatureTrue=[xL;yL;xR;yR];'
    'depthTrue=[safeZL;safeZR]; rhoTrue=[1/safeZL;1/safeZR];'
    'scaleTrue=cfg.targetCharacteristicSize*fMeasured.*rhoTrue;'
    'leftImageMargin=min([uL;cfg.imageWidth-uL;vL;cfg.imageHeight-vL]); rightImageMargin=min([uR;cfg.imageWidth-uR;vR;cfg.imageHeight-vR]);'
    'leftCameraVisible=double((zL>cfg.visibilityZMin)&&(leftImageMargin>=0)); rightCameraVisible=double((zR>cfg.visibilityZMin)&&(rightImageMargin>=0)); validStereo=double(leftCameraVisible>0.5&&rightCameraVisible>0.5);'
    'targetDepthRangeViolation=double(zL<cfg.targetDepthMin||zL>cfg.targetDepthMax); desiredFocalLengthEstimate=cfg.scaleDesired.*depthTrue/cfg.targetCharacteristicSize; focalRequiredMm=desiredFocalLengthEstimate*cfg.outputPixelPitchXmm;'
    'mxL=min(cfg.cxL/max(abs(xL),cfg.numericalEpsilon),(cfg.imageWidth-cfg.cxL)/max(abs(xL),cfg.numericalEpsilon)); myL=min(cfg.cyL/max(abs(yL),cfg.numericalEpsilon),(cfg.imageHeight-cfg.cyL)/max(abs(yL),cfg.numericalEpsilon)); mxR=min(cfg.cxR/max(abs(xR),cfg.numericalEpsilon),(cfg.imageWidth-cfg.cxR)/max(abs(xR),cfg.numericalEpsilon)); myR=min(cfg.cyR/max(abs(yR),cfg.numericalEpsilon),(cfg.imageHeight-cfg.cyR)/max(abs(yR),cfg.numericalEpsilon)); fVis=max(cfg.focalLengthHardwareMinMm(1),0.9*min([mxL;myL;mxR;myR])*cfg.outputPixelPitchXmm); fScale=0.8*min(cfg.imageWidth,cfg.imageHeight)*min(depthTrue)*cfg.outputPixelPitchXmm/cfg.targetCharacteristicSize; eff=min([cfg.focalLengthWorkingMaxMm(1);fVis;fScale]); focalEffectiveUpperLimitMm=eff*ones(2,1); focalVisibilityLimited=double(fVis<cfg.focalLengthWorkingMaxMm(1)); focalScaleLimited=double(fScale<cfg.focalLengthWorkingMaxMm(1)); focalFeasibilityFlag=double(all(focalRequiredMm>=cfg.focalLengthWorkingMinMm)&all(focalRequiredMm<=focalEffectiveUpperLimitMm));'
    'if any(~isfinite(pixelMeasurementTrue)), pixelMeasurementTrue=zeros(4,1); normalizedFeatureTrue=zeros(4,1); scaleTrue=zeros(2,1); rhoTrue=zeros(2,1); depthTrue=zeros(2,1); validStereo=0; leftCameraVisible=0; rightCameraVisible=0; leftImageMargin=-1; rightImageMargin=-1; targetDepthRangeViolation=1; desiredFocalLengthEstimate=zeros(2,1); focalRequiredMm=zeros(2,1); focalFeasibilityFlag=0; focalVisibilityLimited=1; focalScaleLimited=1; focalEffectiveUpperLimitMm=cfg.focalLengthWorkingMinMm; end'
    'end'}, newline);
end

function code = noiseCode
code = strjoin({
    'function pixelMeasurement = fcn(pixelMeasurementTrue,validLeft,validRight)'
    '%#codegen'
    '% Deterministic fixed-size pseudo-random pixel noise, repeatable per simulation.'
    'pixelMeasurement=zeros(4,1);'
    'persistent state'
    'if isempty(state), state=double(cfg.randomSeed); end'
    'n=zeros(4,1);'
    'for i=1:4'
    '    s=0;'
    '    for j=1:12, state=mod(1664525*state+1013904223,4294967296); s=s+state/4294967296; end'
    '    n(i)=s-6;'
    'end'
    'pixelMeasurement=pixelMeasurementTrue;'
    'if validLeft>0.5, pixelMeasurement(1:2)=pixelMeasurementTrue(1:2)+cfg.pixelNoiseStd*n(1:2); end'
    'if validRight>0.5, pixelMeasurement(3:4)=pixelMeasurementTrue(3:4)+cfg.pixelNoiseStd*n(3:4); end'
    'if any(~isfinite(pixelMeasurement)), pixelMeasurement=zeros(4,1); end'
    'end'}, newline);
end

function code = measuredFeatureCode
code = strjoin({
    'function [normalizedFeatureMeasured,scaleMeasured,zMeasured,xLeftMeasured,yLeftMeasured,xRightMeasured,disparityNormalized,rhoMeasured,validDisparity,uRightMeasured] = fcn(pixelMeasurement,fMeasured,scaleTrue,validLeft,validRight)'
    '%#codegen'
    '% Pixel-to-normalized feature conversion; scale is ideal detector output here.'
    'normalizedFeatureMeasured=zeros(4,1); scaleMeasured=zeros(2,1); zMeasured=zeros(3,1); xLeftMeasured=0; yLeftMeasured=0; xRightMeasured=0; disparityNormalized=0; rhoMeasured=0; validDisparity=0; uRightMeasured=pixelMeasurement(3);'
    'fL=max(fMeasured(1),cfg.numericalEpsilon); fR=max(fMeasured(2),cfg.numericalEpsilon);'
    'normalizedFeatureMeasured=[(pixelMeasurement(1)-cfg.cxL)/fL;(pixelMeasurement(2)-cfg.cyL)/fL;(pixelMeasurement(3)-cfg.cxR)/fR;(pixelMeasurement(4)-cfg.cyR)/fR];'
    'xLeftMeasured=normalizedFeatureMeasured(1); yLeftMeasured=normalizedFeatureMeasured(2); xRightMeasured=normalizedFeatureMeasured(3); disparityNormalized=xLeftMeasured-xRightMeasured;'
    'candidateRho=0; if cfg.baseline>0, candidateRho=disparityNormalized/cfg.baseline; end'
    'validDisparity=double(validLeft>0.5&&validRight>0.5&&isfinite(xLeftMeasured)&&isfinite(yLeftMeasured)&&isfinite(xRightMeasured)&&isfinite(candidateRho)&&cfg.baseline>0&&disparityNormalized>cfg.disparityMin&&candidateRho>=cfg.rhoMin&&candidateRho<=cfg.rhoMax);'
    'if validDisparity>0.5, rhoMeasured=candidateRho; end'
    'zMeasured=[xLeftMeasured;yLeftMeasured;rhoMeasured];'
    'scaleMeasured=max(scaleTrue,cfg.numericalEpsilon*ones(2,1));'
    'if any(~isfinite(normalizedFeatureMeasured)), normalizedFeatureMeasured=zeros(4,1); zMeasured=zeros(3,1); xLeftMeasured=0; yLeftMeasured=0; xRightMeasured=0; disparityNormalized=0; rhoMeasured=0; validDisparity=0; end'
    'end'}, newline);
end

function code = motionTransformCode
code = strjoin({
    'function vHatCR = fcn(vHatW,RWCR)'
    '%#codegen'
    '% Estimated target translational velocity expressed in each camera frame.'
    'vHatCR=zeros(3,1);'
    'vHatCR=RWCR''*vHatW;'
    'if any(~isfinite(vHatCR)), vHatCR=zeros(3,1); end'
    'end'}, newline);
end

function code = validityManagerCode
code = strjoin({
    'function [stereoMeasurementEnable,rightReacquireActive,rightReacquireCount,validStereoQualified] = fcn(validLeft,validRight,validDisparity,reset)'
    '%#codegen'
    '% Require consecutive valid right/disparity frames before stereo EKF correction resumes.'
    'persistent consecutiveValid reacquire'
    'if isempty(consecutiveValid), consecutiveValid=0; reacquire=1; end'
    'if reset>0.5, consecutiveValid=0; reacquire=1; end'
    'rawStereo=validLeft>0.5&&validRight>0.5&&validDisparity>0.5;'
    'if rawStereo, consecutiveValid=min(consecutiveValid+1,cfg.rightReacquireValidSamples); else, consecutiveValid=0; end'
    'if ~rawStereo, reacquire=1; elseif consecutiveValid>=cfg.rightReacquireValidSamples, reacquire=0; end'
    'validStereoQualified=double(rawStereo&&consecutiveValid>=cfg.rightReacquireValidSamples);'
    'stereoMeasurementEnable=validStereoQualified; rightReacquireActive=double(validLeft>0.5&&reacquire>0.5); rightReacquireCount=consecutiveValid;'
    'end'},newline);
end

function code = rightVisibilityGuardCode
code = strjoin({
    'function [uRightPredicted,rightVisibilityMarginActualPx,fRightMaxVisibleMm,fRightMaxEffectiveMm,rightScaleDesiredEffective,rightVisibilityActive,rightVisibilityInfeasible,scaleDesiredEffective,rightScaleDesiredNominal,fRightMeasuredMm] = fcn(xLeftMeasured,rhoHatSafe,focalLengthMeasuredMm,validLeft,validRight,ekfPredictionValid,rightReacquireActive,scaleDesiredNominal,reset)'
    '%#codegen'
    '% CURRENT_STATE_VISIBILITY_GUARD; interface retained for future one-step prediction.'
    'persistent active'
    'if isempty(active), active=1; end'
    'if reset>0.5, active=1; end'
    'rightScaleDesiredNominal=scaleDesiredNominal(2); fRightMeasuredMm=focalLengthMeasuredMm(2); rho=max(rhoHatSafe,cfg.numericalEpsilon); xR=xLeftMeasured-cfg.baseline*rho; fRightPx=focalLengthMeasuredMm(2)/cfg.outputPixelPitchXmm;'
    'uRightPredicted=cfg.cxR+fRightPx*xR; rightVisibilityMarginActualPx=min(uRightPredicted,cfg.imageWidth-uRightPredicted);'
    'hardwareMaxPx=cfg.focalLengthHardwareMaxMm(2)/cfg.outputPixelPitchXmm;'
    'if xR < -cfg.visibilityEpsilon, fMaxPx=(cfg.cxR-cfg.rightVisibilityMarginPx)/(-xR); elseif xR > cfg.visibilityEpsilon, fMaxPx=(cfg.imageWidth-cfg.rightVisibilityMarginPx-cfg.cxR)/xR; else, fMaxPx=hardwareMaxPx; end'
    'fMaxPx=max(fMaxPx,0); fRightMaxVisibleMm=fMaxPx*cfg.outputPixelPitchXmm; rawEffective=min([cfg.focalLengthWorkingMaxMm(2);cfg.focalLengthHardwareMaxMm(2);fRightMaxVisibleMm]);'
    'rightVisibilityInfeasible=double(rawEffective<cfg.focalLengthWorkingMinMm(2)); fRightMaxEffectiveMm=max(cfg.focalLengthWorkingMinMm(2),rawEffective);'
    'scaleRightMaxVisible=cfg.targetCharacteristicSize*fMaxPx*rho; rightScaleDesiredEffective=min(scaleDesiredNominal(2),max(scaleRightMaxVisible,cfg.numericalEpsilon)); scaleDesiredEffective=[scaleDesiredNominal(1);rightScaleDesiredEffective];'
    'enter=(validLeft<=0.5)||(ekfPredictionValid<=0.5)||(validRight<=0.5)||(rightReacquireActive>0.5)||(rightVisibilityInfeasible>0.5)||(uRightPredicted<cfg.rightVisibilityMarginPx)||(uRightPredicted>cfg.imageWidth-cfg.rightVisibilityMarginPx)||(focalLengthMeasuredMm(2)>fRightMaxEffectiveMm+cfg.numericalEpsilon);'
    'exitOK=validLeft>0.5&&validRight>0.5&&ekfPredictionValid>0.5&&rightReacquireActive<=0.5&&rightVisibilityInfeasible<=0.5&&uRightPredicted>=cfg.rightVisibilityMarginPx+cfg.rightVisibilityHysteresisPx&&uRightPredicted<=cfg.imageWidth-cfg.rightVisibilityMarginPx-cfg.rightVisibilityHysteresisPx&&focalLengthMeasuredMm(2)<=fRightMaxEffectiveMm;'
    'if enter, active=1; elseif exitOK, active=0; end; rightVisibilityActive=double(active>0.5);'
    'if any(~isfinite([uRightPredicted;rightVisibilityMarginActualPx;fRightMaxVisibleMm;fRightMaxEffectiveMm;rightScaleDesiredEffective])), uRightPredicted=cfg.cxR; rightVisibilityMarginActualPx=0; fRightMaxVisibleMm=0; fRightMaxEffectiveMm=cfg.focalLengthWorkingMinMm(2); rightScaleDesiredEffective=cfg.numericalEpsilon; scaleDesiredEffective=[scaleDesiredNominal(1);rightScaleDesiredEffective]; rightVisibilityActive=1; rightVisibilityInfeasible=1; end'
    'end'},newline);
end


function code = ekfCode
code = strjoin({
    'function [pHatW,vHatW,aHatW,pHatLeft,vHatLeft,rhoHatRaw,rhoHatSafe,innovation,rhoInnovation,innovationNorm,traceP,ekfPredictionValid,ekfMeasurementUpdated,ekfStateFinite,minEigenvalueP,symmetryErrorP] = fcn(zMeasured,measurementEnable,pWCL,RWCL,fMeasured,reset)'
    '%#codegen'
    '% Nine-state world-frame CA EKF; measurement is [xLeft;yLeft;rhoStereo].'
    'pHatW=zeros(3,1); vHatW=zeros(3,1); aHatW=zeros(3,1); pHatLeft=zeros(3,1); vHatLeft=zeros(3,1); rhoHatRaw=0; rhoHatSafe=0; innovation=zeros(3,1); rhoInnovation=0; innovationNorm=0; traceP=0; ekfPredictionValid=0; ekfMeasurementUpdated=0; ekfStateFinite=0; minEigenvalueP=0; symmetryErrorP=0;'
    'persistent x P'
    'if isempty(x), x=cfg.ekfX0; P=cfg.ekfP0; end'
    'if reset>0.5, x=cfg.ekfX0; P=cfg.ekfP0; end'
    'I3=eye(3); Z3=zeros(3,3); I9=eye(9); Ts=cfg.Ts;'
    'F=[I3 Ts*I3 0.5*Ts*Ts*I3;Z3 I3 Ts*I3;Z3 Z3 I3];'
    's2=cfg.sigmaJerk*cfg.sigmaJerk;'
    'Q=s2*[(Ts^5/20)*I3 (Ts^4/8)*I3 (Ts^3/6)*I3;(Ts^4/8)*I3 (Ts^3/3)*I3 (Ts^2/2)*I3;(Ts^3/6)*I3 (Ts^2/2)*I3 Ts*I3];'
    'xPred=F*x; PPred=F*P*F''+Q; PPred=0.5*(PPred+PPred'');'
    'pPredW=xPred(1:3); pPredLeft=RWCL''*(pPredW-pWCL);'
    'predictionFinite=all(isfinite(xPred))&&all(isfinite(PPred(:)))&&all(isfinite(pPredLeft))&&all(isfinite(RWCL(:)))&&all(isfinite(pWCL)); ekfPredictionValid=double(predictionFinite&&pPredLeft(3)>cfg.visibilityZMin);'
    'h=zeros(3,1); H=zeros(3,9); measurementOK=measurementEnable>0.5&&ekfPredictionValid>0.5&&all(isfinite(zMeasured))&&all(isfinite(fMeasured))&&all(fMeasured>cfg.numericalEpsilon);'
    'if measurementOK'
    '    Z=pPredLeft(3); rho=1/Z; xLeftPred=pPredLeft(1)/Z; yLeftPred=pPredLeft(2)/Z; h=[xLeftPred;yLeftPred;rho];'
    '    HLeft=[rho 0 -xLeftPred*rho;0 rho -yLeftPred*rho;0 0 -rho*rho]; HWorld=HLeft*RWCL''; H=[HWorld zeros(3,3) zeros(3,3)];'
    'end'
    'x=xPred; P=PPred;'
    'if measurementOK && all(isfinite(h)) && all(isfinite(H(:)))'
    '    innovation=zMeasured-h; rhoInnovation=innovation(3);'
    '    pv=cfg.pixelNoiseStd*cfg.pixelNoiseStd; fL=max(fMeasured(1),cfg.numericalEpsilon); fR=max(fMeasured(2),cfg.numericalEpsilon); b=max(cfg.baseline,cfg.numericalEpsilon);'
    '    varXL=pv/(fL*fL); covXRho=pv/(b*fL*fL); varRho=pv/(b*b)*(1/(fL*fL)+1/(fR*fR)); R=[varXL 0 covXRho;0 varXL 0;covXRho 0 varRho]+1e-12*eye(3);'
    '    S=H*PPred*H''+R;'
    '    if all(isfinite(S(:))) && rcond(S)>cfg.ekfSConditionMin'
    '        K=(PPred*H'')/S;'
    '        x=xPred+K*innovation;'
    '        A=I9-K*H; P=A*PPred*A''+K*R*K''; P=0.5*(P+P'');'
    '        if cfg.ekfCovarianceJitter>0, P=P+cfg.ekfCovarianceJitter*I9; end'
    '        ekfMeasurementUpdated=1;'
    '    end'
    'end'
    'pHatW=x(1:3); vHatW=x(4:6); aHatW=x(7:9);'
    'pHatLeft=RWCL''*(pHatW-pWCL); vHatLeft=RWCL''*vHatW;'
    'ekfStateFinite=double(all(isfinite(x))&&all(isfinite(P(:)))&&all(isfinite(pHatLeft))&&all(isfinite(vHatLeft)));'
    'if ekfStateFinite>0.5&&pHatLeft(3)>cfg.visibilityZMin, rhoHatRaw=1/pHatLeft(3); rhoHatSafe=min(max(rhoHatRaw,cfg.rhoMin),cfg.rhoMax); else, rhoHatRaw=0; rhoHatSafe=0; ekfPredictionValid=0; end'
    'innovationNorm=sqrt(innovation''*innovation); symmetryErrorP=norm(P-P'',''fro''); Psym=0.5*(P+P''); traceP=trace(Psym);'
    'ev=eig(Psym); minEigenvalueP=min(real(ev));'
    'if ekfStateFinite<0.5, x=cfg.ekfX0; P=cfg.ekfP0; pHatW=x(1:3); vHatW=x(4:6); aHatW=x(7:9); pHatLeft=zeros(3,1); vHatLeft=zeros(3,1); rhoHatRaw=0; rhoHatSafe=0; innovation=zeros(3,1); rhoInnovation=0; innovationNorm=0; symmetryErrorP=0; traceP=trace(P); minEigenvalueP=0; ekfPredictionValid=0; ekfMeasurementUpdated=0; end'
    'end'}, newline);
end

function code = armControllerCode
code = strjoin({
    'function [qDotCmd,centerError,depthError,rcondJc,rcondArho,qDotCenter,qDotDepthRaw,qDotDepthWeighted,qDotNullUsed] = fcn(q,cLMeasured,rhoHatL,vHatLeft,JL,cLd,rhoD,validLeft,ekfPredictionValid,centerEnable,depthEnable,armEnable,depthTaskWeight)'
    '%#codegen'
    '% Damped approximate priority: left normalized center primary, inverse depth secondary.'
    'qDotCmd=zeros(7,1); centerError=zeros(2,1); depthError=0; rcondJc=0; rcondArho=0; qDotCenter=zeros(7,1); qDotDepthRaw=zeros(7,1); qDotDepthWeighted=zeros(7,1); qDotNullUsed=zeros(7,1);'
    'x=cLMeasured(1); y=cLMeasured(2); rho=rhoD; vFeed=zeros(3,1); if ekfPredictionValid>0.5&&isfinite(rhoHatL)&&rhoHatL>0, rho=max(rhoHatL,cfg.numericalEpsilon); vFeed=vHatLeft; end'
    'centerError=[x;y]-cLd; depthError=rho-rhoD;'
    'Lc=[-rho 0 x*rho x*y -(1+x*x) y;0 -rho y*rho 1+y*y -x*y -x];'
    'Hc=rho*[1 0 -x;0 1 -y]; bhatC=Hc*vFeed; Jc=Lc*JL;'
    'rc=zeros(2,1);'
    'if cfg.robustEnable>0.5, rc=cfg.betaC*centerError/(sqrt(centerError''*centerError)+cfg.epsilonC); end'
    'nuC=-cfg.Kc*centerError-bhatC-rc;'
    'Gc=Jc*Jc''+(cfg.lambdaC^2)*eye(2); JcSharp=Jc''/Gc;'
    'qC=JcSharp*nuC; Nc=eye(7)-JcSharp*Jc; rcondJc=det(Gc)/(trace(Gc)*trace(Gc)+cfg.numericalEpsilon); qDotCenter=qC;'
    'Lrho=[0 0 rho*rho rho*y -rho*x 0]; Hrho=[0 0 -rho*rho];'
    'Jrho=Lrho*JL; bhatRho=Hrho*vFeed;'
    'rr=0; if cfg.robustEnable>0.5, rr=cfg.betaRho*depthError/(abs(depthError)+cfg.epsilonRho); end'
    'nuRho=-cfg.kRho*depthError-bhatRho-rr; A=Jrho*Nc; denom=A*A''+cfg.lambdaRho^2; rcondArho=denom/(1+denom);'
    'w=min(max(depthTaskWeight,0),1)*double(ekfPredictionValid>0.5); qDotDepthRaw=Nc*A''*((nuRho-Jrho*qC)/denom); qDotDepthWeighted=w*qDotDepthRaw;'
    'qDot0=zeros(7,1); if cfg.nullspaceEnable>0.5, qDot0=-cfg.kNull*(q-cfg.qMid); end'
    'ArhoDagger=A''/denom; NcRho=Nc*(eye(7)-ArhoDagger*A); Nnull=(1-w)*Nc+w*NcRho; qDotNullUsed=Nnull*qDot0;'
    'if centerEnable>0.5, qDotCmd=qDotCenter+qDotNullUsed; end'
    'if (centerEnable>0.5)&&(depthEnable>0.5), qDotCmd=qDotCenter+qDotDepthWeighted+qDotNullUsed; end'
    'if (centerEnable<=0.5)&&(depthEnable>0.5), d=Jrho*Jrho''+cfg.lambdaRho^2; qDotDepthRaw=Jrho''*(nuRho/d); qDotDepthWeighted=w*qDotDepthRaw; qDotCenter=zeros(7,1); qDotNullUsed=zeros(7,1); qDotCmd=qDotDepthWeighted; end'
    'qDotCmd=qDotCmd+0*q;'
    'centerAllowed=validLeft>0.5&&cfg.leftOnlyCenterControlEnable>0.5; if (~centerAllowed)||(armEnable<=0.5), qDotCmd=zeros(7,1); qDotCenter=zeros(7,1); qDotDepthWeighted=zeros(7,1); qDotNullUsed=zeros(7,1); end'
    'if any(~isfinite(qDotCmd))'
    '    qDotCmd=zeros(7,1);'
    'end'
    'end'}, newline);
end

function code = rhoDynamicsCode
code = strjoin({
    'function rhoDotHat = fcn(normalizedFeatureMeasured,rhoHat,vHatCL,vHatCR,JL,JR,qDotApplied)'
    '%#codegen'
    '% Predicted inverse-depth rate in each camera using EKF target velocity.'
    'rhoDotHat=zeros(2,1);'
    'for i=1:2'
    '    if i==1, x=normalizedFeatureMeasured(1); y=normalizedFeatureMeasured(2); rho=max(rhoHat,cfg.numericalEpsilon); J=JL; vt=vHatCL;'
    '    else, x=normalizedFeatureMeasured(3); y=normalizedFeatureMeasured(4); rho=max(rhoHat,cfg.numericalEpsilon); J=JR; vt=vHatCR; end'
    '    Lrho=[0 0 rho*rho rho*y -rho*x 0]; Hrho=[0 0 -rho*rho];'
    '    rhoDotHat(i)=Lrho*(J*qDotApplied)+Hrho*vt;'
    'end'
    'if any(~isfinite(rhoDotHat)), rhoDotHat=zeros(2,1); end'
    'end'}, newline);
end

function code = zoomControllerCode
code = strjoin({
    'function [fDotMmCmd,fDotMmLimited,scaleError,gDotCmd,focalCommandOutwardAtWorkingLimit,focalFeasible,focalRateUsageGuaranteedCmd,focalRateUsageHardCmd] = fcn(fMeasuredMm,scaleMeasured,rhoHat,rhoDotHat,scaleDesiredEffective,validLeft,ekfPredictionValid,focalAtWorkingLowerLimit,focalAtWorkingUpperLimit,fRightMaxEffectiveMm,rightVisibilityActive,rightReacquireActive)'
    '%#codegen'
    '% Independent log-focal-length control; focal length is measured, not estimated.'
    'fDotMmCmd=zeros(2,1); fDotMmLimited=zeros(2,1); scaleError=zeros(2,1); gDotCmd=zeros(2,1); focalCommandOutwardAtWorkingLimit=zeros(2,1); focalFeasible=ones(2,1); focalRateUsageGuaranteedCmd=zeros(2,1); focalRateUsageHardCmd=zeros(2,1);'
    'upperEffective=[cfg.focalLengthWorkingMaxMm(1);fRightMaxEffectiveMm]; rho=max(rhoHat,cfg.numericalEpsilon);'
    'for i=1:2'
    '    r=max(scaleMeasured(i),cfg.numericalEpsilon); rd=max(scaleDesiredEffective(i),cfg.numericalEpsilon);'
    '    scaleError(i)=log(r/rd); dHat=rhoDotHat(i)/rho; rf=0;'
    '    if cfg.robustEnable>0.5, rf=cfg.betaF(i)*scaleError(i)/(abs(scaleError(i))+cfg.epsilonF(i)); end'
    '    gDotCmd(i)=-cfg.Kf(i,i)*scaleError(i)-dHat-rf; fDotMmCmd(i)=fMeasuredMm(i)*gDotCmd(i);'
    '    fRequiredMm=scaleDesiredEffective(i)/(cfg.targetCharacteristicSize*rho)*cfg.outputPixelPitchXmm; if fRequiredMm<cfg.focalLengthWorkingMinMm(i)||fRequiredMm>upperEffective(i), focalFeasible(i)=0; end'
    '    focalCommandOutwardAtWorkingLimit(i)=double((focalAtWorkingLowerLimit(i)>0.5&&fDotMmCmd(i)<0)||(focalAtWorkingUpperLimit(i)>0.5&&fDotMmCmd(i)>0)||(fMeasuredMm(i)>=upperEffective(i)&&fDotMmCmd(i)>0));'
    'end'
    'if rightVisibilityActive>0.5||rightReacquireActive>0.5, if fMeasuredMm(2)>fRightMaxEffectiveMm+cfg.numericalEpsilon, fDotMmCmd(2)=-cfg.rightReacquireZoomRateMmPerSec; else, fDotMmCmd(2)=min(fDotMmCmd(2),0); end; end'
    'if validLeft<=0.5||ekfPredictionValid<=0.5, fDotMmCmd(1)=0; if fMeasuredMm(1)>cfg.focalLengthWorkingMinMm(1), fDotMmCmd(1)=-cfg.rightReacquireZoomRateMmPerSec; end; fDotMmCmd(2)=0; if fMeasuredMm(2)>cfg.focalLengthWorkingMinMm(2), fDotMmCmd(2)=-cfg.rightReacquireZoomRateMmPerSec; end; end'
    'fDotMmLimited=min(max(fDotMmCmd,-cfg.focalRateDesignMmPerSec),cfg.focalRateDesignMmPerSec); nextRight=fMeasuredMm(2)+cfg.Ts*fDotMmLimited(2); if fMeasuredMm(2)<=fRightMaxEffectiveMm&&nextRight>fRightMaxEffectiveMm, fDotMmLimited(2)=min(fDotMmLimited(2),(fRightMaxEffectiveMm-fMeasuredMm(2))/cfg.Ts); end'
    'focalRateUsageGuaranteedCmd=abs(fDotMmLimited)/cfg.focalRateGuaranteedMmPerSec; focalRateUsageHardCmd=abs(fDotMmLimited)/cfg.focalRateAbsoluteMaxMmPerSec;'
    'if cfg.zoomControlEnable<=0.5 || any(~isfinite(fDotMmCmd))||any(~isfinite(fDotMmLimited)), fDotMmCmd=zeros(2,1); fDotMmLimited=zeros(2,1); gDotCmd=zeros(2,1); focalRateUsageGuaranteedCmd=zeros(2,1); focalRateUsageHardCmd=zeros(2,1); end'
    'end'}, newline);
end

function code=zoomSupervisorCode
code=strjoin({
    'function [depthTaskWeight,schedulerMode,zoomPriorityActive,zoomPriorityTimer,scaleSettledTimer,armRecoveryReason,newDisturbanceDetected]=fcn(scaleError,depthError,focalLengthMeasuredMm,focalAtWorkingLowerLimit,focalAtWorkingUpperLimit,focalHeadroomMm,focalFeasible,validLeft,validStereoQualified,ekfPredictionValid,reset,focalCommandOutwardAtWorkingLimit,focalRateUsageGuaranteed,focalRateUsageHard,rightReacquireActive)'
    '%#codegen'
    '% Modes: 0 INITIALIZE, 1 ZOOM_FIRST, 2 ARM_RECOVERY, 3 FULL_TRACK, 4 SAFE_HOLD, 5 RIGHT_REACQUIRE.'
    'persistent mode zoomTimer settledTimer rampTimer reason disturbanceTimer validLast rateTimer'
    'if isempty(mode), mode=0; zoomTimer=0; settledTimer=0; rampTimer=0; reason=0; disturbanceTimer=0; validLast=0; rateTimer=0; end'
    'if reset>0.5, mode=0; zoomTimer=0; settledTimer=0; rampTimer=0; reason=0; disturbanceTimer=0; validLast=0; rateTimer=0; end'
    'depthTaskWeight=0; newDisturbanceDetected=0; centerValid=validLeft>0.5; depthReady=validStereoQualified>0.5&&ekfPredictionValid>0.5&&rightReacquireActive<=0.5; e=max(abs(scaleError));'
    'if ~centerValid, mode=4; depthTaskWeight=0; validLast=0;'
    'elseif ~depthReady, mode=5; depthTaskWeight=0; validLast=0; rampTimer=0;'
    'elseif cfg.zoomPriorityEnable<=0.5, mode=3; depthTaskWeight=1; zoomTimer=0; settledTimer=0; rampTimer=cfg.armDepthRampTime; reason=0; disturbanceTimer=0; validLast=1;'
    'else'
    '    if validLast<0.5&&(mode==4||mode==5), mode=1; zoomTimer=0; settledTimer=0; rampTimer=0; reason=0; disturbanceTimer=0; end'
    '    if mode==0, if e>cfg.scaleErrorEnterThreshold, mode=1; depthTaskWeight=0; else, mode=3; depthTaskWeight=1; end'
    '    elseif mode==1'
    '        depthTaskWeight=0; zoomTimer=zoomTimer+cfg.Ts; if e<=cfg.scaleErrorExitThreshold, settledTimer=settledTimer+cfg.Ts; else, settledTimer=0; end'
    '        if any(focalRateUsageGuaranteed>=1), rateTimer=rateTimer+cfg.Ts; else, rateTimer=0; end'
    '        if any(focalRateUsageHard>=1), mode=2; reason=6; rampTimer=0;'
    '        elseif rateTimer>=cfg.disturbanceConfirmTime&&e>cfg.scaleErrorExitThreshold, mode=2; reason=5; rampTimer=0;'
    '        elseif settledTimer>=cfg.scaleSettledHoldTime, mode=2; reason=1; rampTimer=0;'
    '        elseif any(focalCommandOutwardAtWorkingLimit>0.5), mode=2; reason=7; rampTimer=0;'
    '        elseif zoomTimer>=cfg.disturbanceConfirmTime&&any(focalFeasible<0.5), mode=2; reason=3; rampTimer=0;'
    '        elseif zoomTimer>=cfg.zoomOnlyMaxTime, mode=2; reason=4; rampTimer=0; end'
    '    elseif mode==2'
    '        rampTimer=min(rampTimer+cfg.Ts,cfg.armDepthRampTime); s=min(max(rampTimer/max(cfg.armDepthRampTime,cfg.Ts),0),1); depthTaskWeight=s*s*(3-2*s); if depthTaskWeight>=1-1e-6, mode=3; depthTaskWeight=1; disturbanceTimer=0; end'
    '    elseif mode==3'
    '        depthTaskWeight=1; if e>=cfg.scaleErrorEnterThreshold, depthTaskWeight=0; disturbanceTimer=disturbanceTimer+cfg.Ts; else, disturbanceTimer=0; end'
    '        if disturbanceTimer>=cfg.disturbanceConfirmTime, mode=1; depthTaskWeight=0; zoomTimer=0; settledTimer=0; rampTimer=0; reason=0; disturbanceTimer=0; newDisturbanceDetected=1; end'
    '    else, mode=1; depthTaskWeight=0; zoomTimer=0; settledTimer=0; rampTimer=0; reason=0; end'
    '    validLast=1;'
    'end'
    'schedulerMode=mode; zoomPriorityActive=double(mode==1); zoomPriorityTimer=zoomTimer; scaleSettledTimer=settledTimer; armRecoveryReason=reason;'
    'if any(~isfinite([depthTaskWeight;schedulerMode;zoomPriorityTimer;scaleSettledTimer;depthError;focalLengthMeasuredMm;focalHeadroomMm;focalAtWorkingLowerLimit;focalAtWorkingUpperLimit])), depthTaskWeight=0; schedulerMode=4; zoomPriorityActive=0; newDisturbanceDetected=0; end'
    'end'},newline);
end

function code = safetyCode
code = strjoin({
    'function [qDotApplied,fDotMmApplied,qSaturationFlag,focalRateSaturationFlag,jointVelocitySaturationFlag,jointVelocitySaturationAny,jointLimitWarning,cartesianSpeed,cartesianSpeedScale,cartesianSpeedSaturationFlag,focalLengthCommandMm,focalRateUsageGuaranteed,focalRateUsageHard,focalRateHardViolation] = fcn(qDotCmd,fDotMmLimited,q,focalLengthMeasuredMm,validLeft,JL)'
    '%#codegen'
    '% Element-wise actuator limits, state-bound projection and finite-value guards.'
    'qDotApplied=zeros(7,1); fDotMmApplied=zeros(2,1); qSaturationFlag=0; focalRateSaturationFlag=0; jointVelocitySaturationFlag=zeros(7,1); jointVelocitySaturationAny=0; jointLimitWarning=zeros(7,1); cartesianSpeed=0; cartesianSpeedScale=1; cartesianSpeedSaturationFlag=0; focalLengthCommandMm=focalLengthMeasuredMm; focalRateUsageGuaranteed=zeros(2,1); focalRateUsageHard=zeros(2,1); focalRateHardViolation=zeros(2,1);'
    'if validLeft>0.5 && all(isfinite(qDotCmd))'
    '    qCandidate=min(max(qDotCmd,-cfg.qDotMax),cfg.qDotMax); jointVelocitySaturationFlag=double(abs(qCandidate-qDotCmd)>1e-12); jointVelocitySaturationAny=double(any(jointVelocitySaturationFlag>0.5));'
    '    for i=1:7, nearLow=q(i)-cfg.qMin(i)<cfg.qLimitSoftMargin; nearHigh=cfg.qMax(i)-q(i)<cfg.qLimitSoftMargin; jointLimitWarning(i)=double(nearLow||nearHigh); if (nearLow&&qCandidate(i)<0)||(nearHigh&&qCandidate(i)>0), qCandidate(i)=0; end, end'
    '    VCL=JL*qCandidate; cartesianSpeed=norm(VCL(1:3)); if cartesianSpeed>cfg.cartesianLinearSpeedMax, cartesianSpeedScale=cfg.cartesianLinearSpeedMax/cartesianSpeed; qCandidate=cartesianSpeedScale*qCandidate; cartesianSpeedSaturationFlag=1; cartesianSpeed=cfg.cartesianLinearSpeedMax; end'
    '    qDotApplied=min(max(qCandidate,-cfg.qDotMax),cfg.qDotMax); for i=1:7, if (q(i)<=cfg.qMin(i)&&qDotApplied(i)<0)||(q(i)>=cfg.qMax(i)&&qDotApplied(i)>0), qDotApplied(i)=0; end, end'
    'end'
    'if all(isfinite(fDotMmLimited))'
    '    fDotMmApplied=min(max(fDotMmLimited,-cfg.focalRateAbsoluteMaxMmPerSec),cfg.focalRateAbsoluteMaxMmPerSec);'
    '    for i=1:2, belowW=focalLengthMeasuredMm(i)<=cfg.focalLengthWorkingMinMm(i); aboveW=focalLengthMeasuredMm(i)>=cfg.focalLengthWorkingMaxMm(i); belowH=focalLengthMeasuredMm(i)<=cfg.focalLengthHardwareMinMm(i); aboveH=focalLengthMeasuredMm(i)>=cfg.focalLengthHardwareMaxMm(i); if ((belowW||belowH)&&fDotMmApplied(i)<0)||((aboveW||aboveH)&&fDotMmApplied(i)>0), fDotMmApplied(i)=0; end; nextF=focalLengthMeasuredMm(i)+cfg.Ts*fDotMmApplied(i); if focalLengthMeasuredMm(i)>=cfg.focalLengthWorkingMinMm(i)&&nextF<cfg.focalLengthWorkingMinMm(i), fDotMmApplied(i)=(cfg.focalLengthWorkingMinMm(i)-focalLengthMeasuredMm(i))/cfg.Ts; elseif focalLengthMeasuredMm(i)<=cfg.focalLengthWorkingMaxMm(i)&&nextF>cfg.focalLengthWorkingMaxMm(i), fDotMmApplied(i)=(cfg.focalLengthWorkingMaxMm(i)-focalLengthMeasuredMm(i))/cfg.Ts; end; end'
    'end'
    'if any(abs(qDotApplied-qDotCmd)>1e-12), qSaturationFlag=1; end'
    'if any(abs(fDotMmApplied-fDotMmLimited)>1e-12), focalRateSaturationFlag=1; end'
    'if any(~isfinite(qDotApplied))||~isfinite(cartesianSpeed), qDotApplied=zeros(7,1); qSaturationFlag=1; cartesianSpeed=0; cartesianSpeedScale=0; end'
    'if any(~isfinite(fDotMmApplied)), fDotMmApplied=zeros(2,1); focalRateSaturationFlag=1; end'
    'focalRateUsageGuaranteed=abs(fDotMmApplied)/cfg.focalRateGuaranteedMmPerSec; focalRateUsageHard=abs(fDotMmApplied)/cfg.focalRateAbsoluteMaxMmPerSec; focalRateHardViolation=double(abs(fDotMmLimited)>cfg.focalRateAbsoluteMaxMmPerSec); focalLengthCommandMm=min(max(focalLengthMeasuredMm+cfg.Ts*fDotMmApplied,cfg.focalLengthHardwareMinMm),cfg.focalLengthHardwareMaxMm);'
    'end'}, newline);
end
