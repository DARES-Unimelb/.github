% UACS_JOB_SIM_SCENARIOS_V910 Job entry: V9 crosswind and V10 step-size independence.
% V6 is excluded because it needs the manual-PID trajectory, which depends on the
% PID design that is not finished yet.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_scenarios_v910');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
R = run_scenarios('which', {'v9', 'v10'}, 'plots', false);
fprintf('scenario run complete, report %s\n', R.report_file);
if ~isempty(R.v9)
   f = fieldnames(R.v9);
   fprintf('V9 fields: %s\n', strjoin(f(:).', ' '));
end
if ~isempty(R.v10)
   f = fieldnames(R.v10);
   fprintf('V10 fields: %s\n', strjoin(f(:).', ' '));
end
fprintf('JOB BODY DONE\n');
