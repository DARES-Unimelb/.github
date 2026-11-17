% diag_int_probe Measure what the BUILT model actually does on a saturating pitch command.
%
% Answers by measurement, not by reading source:
%   1. which integral-state bound each loop's integrator really rides
%   2. how often the rate-command clamp q_c_lim acts
%   3. what the elevator reaches against its travel limit
%
% The stick goes to full pitch, not the 0.3 that run_manual_pid's pitch_step uses.
% 0.3 of the pitch authority stays inside the linear region, so it saturates neither
% the elevator nor the rate-command clamp and cannot answer any of the three.
%
% Output goes to results/int_limit_probe.txt, which is the evidence file this probe
% exists to produce, and it is also echoed to the console.

root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), fullfile(root, 'verify'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', 'int_limit_probe.txt');
fid = fopen(out, 'w');
mdl = 'uacs_manual_pid';
try
   evalc('[~, ~] = sim_init_params();');
   P = evalin('base', 'P');
   D = sim_load_design(P);
   sim_push_design_vars(P, D);
   Isat_th = evalin('base', 'sim_pid_theta_Isat');
   qc_lim = evalin('base', 'sim_rate_cmd_lim_q');
   % The comparison column reads the fields the linear design simulation actually
   % applies. It used to hold the retired formulas int_limit/bw and 10*bw, which is
   % how this probe first reported a 2.5x disagreement that was half definition and
   % half real. Comparing a pushed value against a formula is not a consistency check.
   lin_th = P.ctrl.int_state_limit.theta;
   lin_V = P.ctrl.int_state_limit.V;
   fprintf(fid, 'P %s rev %g\n', P.meta.version, P.meta.revision);
   fprintf(fid, '%-22s %14s %14s %8s\n', 'quantity', 'model', 'linear sim', 'ratio');
   fprintf(fid, '%-22s %14.9f %14.9f %8.4f\n', 'theta integral bound', Isat_th, lin_th, Isat_th / lin_th);
   fprintf(fid, '%-22s %14.9f %14.9f %8.4f\n', 'V integral bound', ...
      evalin('base', 'sim_pid_V_Isat'), lin_V, evalin('base', 'sim_pid_V_Isat') / lin_V);
   fprintf(fid, '%-22s %14.9f %14.9f %8.4f\n', 'q_c command clamp', qc_lim, ...
      P.ctrl.rate_cmd_limit.q_rad_s, qc_lim / P.ctrl.rate_cmd_limit.q_rad_s);
   fprintf(fid, 'a ratio of 1.0 on every line is the point of this probe\n');

   t = (0:0.02:12)';
   pitch = zeros(size(t));
   pitch(t >= 2) = 1.0;
   V_min = P.limits.V_min_mps; V_max = P.limits.V_max_mps;
   thr = (P.op.cruise.V_mps - V_min) / (V_max - V_min);
   stick = [t, zeros(numel(t), 1), pitch, zeros(numel(t), 1), thr * ones(numel(t), 1)];
   sim_push_scenario(struct('t_end', 12, 'x0', D.trim.x_star, 'stick', stick, ...
      'wind', [0 0 0 0; 12 0 0 0], 'flight_mode', 0));

   if bdIsLoaded(mdl)
      close_system(mdl, 0);
   end
   load_system(fullfile(root, 'model', [mdl '.slx']));

   [T, meta] = sim_run_model(mdl, struct('solver', 'ode4', 't_end', 12, 'states', true));
   fprintf(fid, '\nyout names: %s\n', strjoin(T.names, ', '));
   fprintf(fid, 'state matrix %s\n', mat2str(size(meta.states)));
   for k = 1:numel(meta.state_blocks)
      idx = meta.state_col0(k) + (1:(meta.state_col0(k + 1) - meta.state_col0(k)));
      fprintf(fid, '  sig %2d  cols %2d..%2d  max %14.8f  %s\n', ...
         k, idx(1), idx(end), max(abs(meta.states(:, idx)), [], 'all'), ...
         meta.state_blocks(k));
   end

   rc = local_out(T, 'rate_cmd');
   st = local_out(T, 'x_state');
   cm = local_out(T, 'cmd');
   ua = local_out(T, 'u_act');
   fprintf(fid, '\n--- measured behaviour on a full pitch stick at t = 2 s ---\n');
   fprintf(fid, 'rate_cmd |q_c| peak %.6f; at the %.6f clamp on %d of %d samples\n', ...
      max(abs(rc(:, 2))), qc_lim, sum(abs(abs(rc(:, 2)) - qc_lim) < 1e-6), size(rc, 1));
   fprintf(fid, 'rate_cmd |p_c| peak %.6f\n', max(abs(rc(:, 1))));
   fprintf(fid, 'theta peak %.6f deg against a %.6f deg command\n', ...
      max(st(:, 8)) * 180 / pi, cm(end, 2) * 180 / pi);
   fprintf(fid, 'theta_c command peak %.6f deg\n', max(cm(:, 2)) * 180 / pi);
   fprintf(fid, 'elevator reach %.6f deg of a %.6f deg limit, pinned on %d samples\n', ...
      max(abs(ua(:, 2))) * 180 / pi, P.act.elevator.max_deg, ...
      sum(abs(abs(ua(:, 2)) - deg2rad(P.act.elevator.max_deg)) < 1e-6));

   % Which state is the theta integral: the one that moves when Isat is moved.
   assignin('base', 'sim_pid_theta_Isat', Isat_th * 0.25);
   assignin('base', 'sim_pid_theta_Isat_neg', -Isat_th * 0.25);
   [~, meta2] = sim_run_model(mdl, struct('solver', 'ode4', 't_end', 12, 'states', true));
   d = max(abs(meta2.states - meta.states), [], 1);
   [dmx, kth] = max(d);
   fprintf(fid, '\nIsat moved to %.9f: largest state change %.3e at column %d\n', ...
      Isat_th * 0.25, dmx, kth);
   for k = 1:numel(meta.state_blocks)
      idx = meta.state_col0(k) + (1:(meta.state_col0(k + 1) - meta.state_col0(k)));
      if any(kth == idx)
         fprintf(fid, '  that column is %s\n', meta.state_blocks(k));
         fprintf(fid, '  its max was %.9f with Isat %.9f\n', ...
            max(abs(meta.states(:, kth))), Isat_th);
         fprintf(fid, '  its max is  %.9f with Isat %.9f\n', ...
            max(abs(meta2.states(:, kth))), Isat_th * 0.25);
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

function y = local_out(T, want)
% Look an output up by name, listing what exists when the name is not there.
key = matlab.lang.makeValidName(want);
if isfield(T.by_name, key)
   y = T.by_name.(key);
else
   error('diag_int_probe:NoOutput', 'no output %s; available: %s', ...
      want, strjoin(T.names, ', '));
end
end
