% diag_qstep Open-loop pitch rate response to a constant elevator step, on the
% nonlinear plant model. This decides whether the zero that de->q shows at the
% origin is real or a linearisation artefact.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'sim'), ...
   fullfile(root, 'control'), fullfile(root, 'run'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_qstep.log');
fid = fopen(out, 'w');
try
   [P_init, params_src] = sim_init_params(); %#ok<ASGLU>
   P = evalin('base', 'P');
   trim = trim_cruise(P, P.op.cruise);
   t = (0:0.01:4.0)';
   u = repmat(trim.u_star(:).', numel(t), 1);
   u(:, 2) = u(:, 2) + deg2rad(1.0);
   wind = zeros(numel(t), 4);
   [T, meta] = sim_run_plant_cmd('uacs_plant', t, u, wind);
   q = T.by_name.x_state(:, 5);
   th = T.by_name.x_state(:, 8);
   al = T.by_name.x_state(:, 2);
   fprintf(fid, 'model %s, %d samples, solver %s\n', meta.model, meta.n_samples, ...
      meta.solver);
   for tv = [0, 0.5, 1, 2, 4]
      [~, i] = min(abs(T.t - tv));
      fprintf(fid, 't %5.2f  q %+12.6e  theta %+12.6e  alpha %+12.6e\n', ...
         T.t(i), q(i), th(i), al(i));
   end
catch ME
   fprintf(fid, 'FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);
