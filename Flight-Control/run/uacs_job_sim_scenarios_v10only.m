% UACS_JOB_SIM_SCENARIOS_V10 Job entry: V10 step-size independence only.
% V9 already has data in results/sim_wind_crosswind.mat, so re-running it would
% cost another five minutes for nothing.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_scenarios_v10only');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
R = run_scenarios('which', {'v10'}, 'plots', false);
fprintf('V10 report -> %s\n', R.report_file);
if ~isempty(R.v10)
   fprintf('max_rel_ptp %.6e  max_rel_scale %.6e  worst state %d (%s)\n', ...
      R.v10.max_rel_ptp, R.v10.max_rel_scale, R.v10.worst_state_rel_ptp, ...
      R.v10.state_names{R.v10.worst_state_rel_ptp});
   fprintf('samples ode4 %d  ode45 %d\n', numel(R.v10.t_common), numel(R.v10.t_ode45));
end
fprintf('JOB BODY DONE\n');
