%% START_HERE - initialize and open the packaged simulinkv2 model
packageDir = fileparts(mfilename('fullpath'));
cd(packageDir);
addpath(packageDir);

init_arm_stereo_ibvs_ekf_v1;
model = 'arm_stereo_ibvs_ekf_v1';
load_system(model);
set_param(model,'SimulationCommand','update');

fprintf('simulinkv2 is ready. Model: %s.slx\n',model);
fprintf('Sample time: %.17g s; formal stereo baseline: %.17g m\n',cfg.Ts,cfg.baseline);
fprintf('To simulate: simOut = sim(''%s'');\n',model);
open_system(model);
