% UACS_JOB_SIM_LQR Job entry: simulate the LQR scenarios from the built model.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_lqr');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);
R = run_lqr();
fprintf('LQR scenarios done: %d trajectories\n', numel(R.traj));
for k = 1:numel(R.traj)
   m = R.traj(k).metrics;
   fprintf(['  %-16s V ss err %10.3e m/s | phi ss err %10.3e rad | ' ...
      'z tail [%9.3e %9.3e] | act max %6.2f deg\n'], ...
      R.traj(k).name, m.V.error_ss, m.phi.error_ss, m.z_long_tail, m.z_lat_tail, ...
      m.act_max_deg);
end
fprintf('reference map linearity residual %.3e\n', R.refmap_linearity);
