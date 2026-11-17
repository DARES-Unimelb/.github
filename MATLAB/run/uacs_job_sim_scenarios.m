% UACS_JOB_SIM_SCENARIOS Job entry: V10 step-size independence and V9 crosswind.
% Owner: ash-sim (task-3). One thing only: the multi-run acceptance scenarios.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_scenarios');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);
R = run_scenarios('which', {'v6', 'v9', 'v10', 'v12'});
if ~isempty(R.v10)
   fprintf('V10 max rel ptp %.6e at %s (linear interp %.6e), rel scale %.6e\n', ...
      R.v10.max_rel_ptp, R.v10.state_names{R.v10.worst_state_rel_ptp}, ...
      R.v10.max_rel_ptp_linear, R.v10.max_rel_scale);
   fprintf('V10 ode4 samples %d, ode45 samples %d\n', ...
      R.v10.meta_ode4.n_samples, R.v10.meta_ode45.n_samples);
end
if ~isempty(R.v9)
   fprintf('V9 phi tail %.4f deg, beta tail %.4f deg, drift %.4f m/s, residual %.4f m\n', ...
      R.v9.phi_tail_deg, R.v9.beta_tail_deg, R.v9.drift_rate_mps, R.v9.residual_max_m);
end
fprintf('V12 %s: %s\n', R.v12.status, R.v12.reason);
