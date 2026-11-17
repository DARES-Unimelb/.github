function uacs_job_sim_v9only()
% UACS_JOB_SIM_V9ONLY Job entry: the V9 crosswind case on its own.
% V10 is skipped because it does not depend on the wind initial condition, and the full
% v9+v10 job costs about three times as long.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_v9only');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions; %#ok<CLFUNC>
rehash path;
R = run_scenarios('which', {'v9'}, 'plots', false);
fprintf('scenario run complete, report %s\n', R.report_file);
if ~isempty(R.v9)
   fprintf('V9 fields: %s\n', strjoin(fieldnames(R.v9).', ' '));
end
fprintf('JOB BODY DONE\n');
end
