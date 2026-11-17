function rep = build_bench_models()
% BUILD_BENCH_MODELS Build the two interactive models from uacs_go's parameters.
%
%   rep = build_bench_models()
%
%   model/uacs_flight_bench.slx   interactive flight bench; PID and LQR run in parallel
%                                 with a rotary switch between them
%   model/uacs_aero_test.slx      aerodynamics page; coefficients and forces come from
%                                 plant/aero_forces.m
%
% Keep the models in <root>/model/: their callbacks derive the project root from the .slx
% location, so a double-click works with no setup. Run this through run_job.

this = mfilename('fullpath');
if isempty(this)
   this = which('build_bench_models');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
rehash path;

% Push parameters, gains and the default scenario to base so the model variables have values.
run(fullfile(root, 'uacs_go.m'));
P = evalin('base', 'P');
js_id = evalin('base', 'sim_js_id');

% Model callbacks: add the project root to the search path, then run uacs_go.
%
% PreLoadFcn and InitFcn both carry this, so one press of Run also recovers when
% PreLoadFcn did not run.
boot = ['p=fileparts(fileparts(get_param(bdroot,''FileName'')));' ...
   'addpath(p,fullfile(p,''params''),fullfile(p,''plant''),fullfile(p,''control''),' ...
   'fullfile(p,''sim''),fullfile(p,''run''),fullfile(p,''verify''),''-begin'');'];
reload = [boot 'uacs_go'];
ensure = [boot 'uacs_ensure'];

fprintf('\n构建交互模型...\n');
bench = sim_build_bench(P, struct('name', 'uacs_flight_bench', ...
   'joystick_id', js_id, 'stop_time', 'sim_bench_t_end', ...
   'reload_fcn', reload, 'ensure_fcn', ensure));
aero = sim_build_aero_test(P, struct('reload_fcn', reload, 'ensure_fcn', ensure));
rep = struct('bench', bench, 'aero', aero, 'reload_fcn', reload, ...
   'joystick_id', js_id, 'bench_stop_s', evalin('base', 'sim_bench_t_end'));
fprintf('  %-20s top %3d total %3d  %s\n', bench.model, bench.census.top_count, ...
   bench.census.total_count, bench.file);
fprintf('  %-20s top %3d total %3d  %s\n', aero.model, aero.census.top_count, ...
   aero.census.total_count, aero.file);
fprintf('台架手柄 JoystickID = %s\n', js_id);
fprintf('构建完成。双击 model 目录下的 .slx 即可使用。\n\n');
end
