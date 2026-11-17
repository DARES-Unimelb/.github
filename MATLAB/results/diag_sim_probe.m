% diag_sim_probe Replicate sim_run_model's exact setup and dump the sim failure.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), fullfile(root, 'verify'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_probe_sim2.log');
fid = fopen(out, 'w');
try
   [P_init, ps] = sim_init_params(); %#ok<ASGLU>
   P = evalin('base', 'P');
   D = sim_load_design(P);
   sim_push_design_vars(P, D);
   tr = trim_cruise(P, P.op.cruise);
   V_min = P.limits.V_min_mps; V_max = P.limits.V_max_mps;
   thr = (P.op.cruise.V_mps - V_min) / (V_max - V_min);
   t = (0:0.02:10)';
   stick = [t, zeros(numel(t), 3), thr * ones(numel(t), 1)];
   S = sim_push_scenario(struct('t_end', 10, 'x0', tr.x_star, 'stick', stick, ...
      'wind', [0 0 0 0; 10 0 0 0], 'flight_mode', 0));
   fprintf(fid, 'scenario fields: %s\n', strjoin(fieldnames(S).', ', '));
   mdl = 'uacs_manual_pid';
   if bdIsLoaded(mdl), close_system(mdl, 0); end
   load_system(fullfile(root, 'model', [mdl '.slx']));
   set_param(mdl, 'StopTime', 'sim_t_end', 'SimulationMode', 'normal', ...
      'SaveTime', 'on', 'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on');
   set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'sim_ts_ctrl');
   t0 = tic;
   set_param(mdl, 'SimulationCommand', 'update');
   fprintf(fid, 'update ok in %.2f s\n', toc(t0));
   try
      o = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
      fprintf(fid, 'sim ok, yout class %s\n', class(o.get('yout')));
   catch ME
      fprintf(fid, 'SIM FAILED %s | %s\n', ME.identifier, ME.message);
      for k = 1:numel(ME.cause)
         fprintf(fid, '  cause %d: %s | %s\n', k, ME.cause{k}.identifier, ME.cause{k}.message);
      end
   end
   close_system(mdl, 0);
catch ME
   fprintf(fid, 'PROBE FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
disp(fileread(out));
