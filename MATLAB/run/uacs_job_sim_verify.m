% UACS_JOB_SIM_VERIFY Job entry: run the Simulink acceptance cases.
% Owner: ash-sim (task-3). Reads the saved results and the .slx files from disk;
% it does not regenerate anything, so it is an independent check.
%
% V13 does push the design variables into the base workspace and run the LINEAR
% simulation, because the values the model reads only exist after a push and the
% consistency check has to compare them against the design fields. It never runs the
% model and never rewrites a design file, so the trajectories it judges are still the
% ones the scenario jobs produced.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_verify');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
[P_init, params_src] = sim_init_params(); %#ok<ASGLU>
out = v_sim_all();
for k = 1:numel(out)
   fprintf('%-22s %-4s %s\n', out(k).case, out(k).status, out(k).detail);
end
