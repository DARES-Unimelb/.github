% UACS_JOB_SIM_ALL Job entry: the whole Simulink chain, stage by stage.
%
% Owner: ash-sim (task-3). Stages run in order, each one's verdict is printed as
% it finishes, and a failure does not stop the later stages. That matters on a
% shared session: if the MCP request is dropped while MATLAB keeps working, the
% job diary still shows exactly which stage reached where. The single-stage jobs
% stay available for reruns of one step.
%
% No local functions in this file on purpose: a script that defines local
% functions cannot be executed through evalin, which is how run_job starts it.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_all');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);
[P_init, params_src] = sim_init_params(); %#ok<ASGLU>

ok = true(1, 5);
ok(1) = sim_stage_run('build', @() build_uacs_models());
ok(2) = sim_stage_run('manual_pid', @() run_manual_pid());
ok(3) = sim_stage_run('lqr', @() run_lqr());
ok(4) = sim_stage_run('scenarios', @() run_scenarios('which', {'v6', 'v9', 'v10', 'v12'}));
ok(5) = sim_stage_run('verify', @() v_sim_all());

fprintf('\n===== SUMMARY =====\n');
names = {'build', 'manual_pid', 'lqr', 'scenarios', 'verify'};
for k = 1:numel(names)
   if ok(k)
      s = 'OK';
   else
      s = 'FAILED';
   end
   fprintf('  %-11s %s\n', names{k}, s);
end
fprintf('stages failed: %d of %d\n', sum(~ok), numel(ok));
