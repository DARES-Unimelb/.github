% UACS_JOB_SIM_MANUAL_PID Job entry: simulate the PID cascade scenarios and save their
% trajectories. Build the models in a separate job first.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_manual_pid');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);
R = run_manual_pid();
fprintf('PID scenarios done: %d trajectories\n', numel(R.traj));
for k = 1:numel(R.traj)
   m = R.traj(k).metrics;
   fprintf(['  %-16s phi tr %7.4f os %6.2f err %10.3e | theta tr %7.4f os %6.2f | ' ...
      'r tr %7.4f | V tr %7.4f err %10.3e | h err %10.3e\n'], ...
      R.traj(k).name, m.phi.rise_time_s, m.phi.overshoot_pct, m.phi.error_ss, ...
      m.theta.rise_time_s, m.theta.overshoot_pct, m.r.rise_time_s, ...
      m.V.rise_time_s, m.V.error_ss, m.h.error_ss);
end
