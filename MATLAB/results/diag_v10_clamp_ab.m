% diag_v10_clamp_ab Attribute the V10 step-size difference to a specific clamp.
%
% V10 compares ode45 against ode4 on one command sequence and requires the two to
% agree to 1 percent of the trajectory peak-to-peak. At rev 10 it reported 1.888e-08;
% at rev 11 it reports 1.347e-02, so something introduced by the rev 11 clamp change
% made the system step-size sensitive. This runs the SAME scenario four times with
% the clamp set varied one family at a time, so the attribution is a measurement.
%
%   A  rev 11 as shipped
%   B  the rate-command clamps widened back to the old 10*bandwidth guards
%   C  the integral-state clamps narrowed back to the old int_limit values
%   D  both, which is the rev 10 controller
%
% Output goes to results/v10_clamp_ab.txt.

root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), fullfile(root, 'verify'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', 'v10_clamp_ab.txt');
fid = fopen(out, 'w');
try
   evalc('[~, ~] = sim_init_params();');
   P = evalin('base', 'P');
   D = sim_load_design(P);
   sim_push_design_vars(P, D);
   fprintf(fid, 'P %s rev %g, design branch %s\n', P.meta.version, P.meta.revision, D.src.branch);

   mdl = 'uacs_manual_pid';
   if ~bdIsLoaded(mdl)
      load_system(fullfile(root, 'model', [mdl '.slx']));
   end
   t_end = 8.0;
   V_min = P.limits.V_min_mps; V_max = P.limits.V_max_mps;
   thr = (P.op.cruise.V_mps - V_min) / (V_max - V_min);
   t = (0:0.02:t_end)';
   n = numel(t);
   pitch = zeros(n, 1);
   pitch(t >= 1.0 & t < 2.0) = 0.4;
   pitch(t >= 2.0 & t < 3.0) = -0.4;
   thr_cmd = thr * ones(n, 1);
   thr_cmd(t >= 3.5 & t < 4.5) = thr + 0.05;
   stick = [t, zeros(n, 1), pitch, zeros(n, 1), thr_cmd];
   sim_push_scenario(struct('t_end', t_end, 'x0', D.trim.x_star, 'stick', stick, ...
      'wind', [0, 0, 0, 0; t_end, 0, 0, 0], 'flight_mode', 0));

   scale = sim_state_scale(P);
   names = {'u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'pn', 'pe', 'pd'};

   cfg = struct('name', {}, 'set', {});
   cfg(1).name = 'A rev 11 as shipped';
   cfg(1).set = struct();
   cfg(2).name = 'B rate clamps widened to 10*bw';
   cfg(2).set = struct('sim_rate_cmd_lim_p', 10 * P.ctrl.bw.rate_p_rad_s, ...
      'sim_rate_cmd_lim_q', 10 * P.ctrl.bw.rate_q_rad_s, ...
      'sim_rate_cmd_lim_r', 10 * P.ctrl.bw.rate_r_rad_s);
   cfg(3).name = 'C integral clamps narrowed to int_limit';
   cfg(3).set = struct('sim_pid_theta_Isat', P.ctrl.int_limit.theta, ...
      'sim_pid_theta_Isat_neg', -P.ctrl.int_limit.theta, ...
      'sim_pid_V_Isat', P.ctrl.int_limit.V, 'sim_pid_V_Isat_neg', -P.ctrl.int_limit.V, ...
      'sim_pid_phi_Isat', P.ctrl.int_limit.phi, ...
      'sim_pid_phi_Isat_neg', -P.ctrl.int_limit.phi, ...
      'sim_pid_h_Isat', P.ctrl.int_limit.h, 'sim_pid_h_Isat_neg', -P.ctrl.int_limit.h);
   cfg(4).name = 'D both, the rev 10 controller';
   cfg(4).set = cfg(2).set;
   s4 = cfg(3).set;
   f4 = fieldnames(s4);
   for k = 1:numel(f4)
      cfg(4).set.(f4{k}) = s4.(f4{k});
   end

   fprintf(fid, '%-30s %12s %12s %12s  %s\n', 'configuration', 'max_rel_ptp', ...
      'rel_scale', 'worst_state', 'clamp activity');
   for k = 1:numel(cfg)
      sim_push_design_vars(P, D);
      f = fieldnames(cfg(k).set);
      for j = 1:numel(f)
         assignin('base', f{j}, cfg(k).set.(f{j}));
      end
      [T4, ~] = sim_run_model(mdl, struct('solver', 'ode4', 't_end', t_end));
      [T45, ~] = sim_run_model(mdl, struct('solver', 'ode45', 't_end', t_end));
      X4 = T4.by_name.x_state;
      X45 = pchip_interp(T45.t, T45.by_name.x_state, T4.t);
      d = max(abs(X4 - X45), [], 1)';
      ptp = (max(X4, [], 1) - min(X4, [], 1))';
      den = ptp;
      small = den < 1e-4 * scale;
      den(small) = scale(small);
      rel = d ./ den;
      [mx, iw] = max(rel);
      rc = T4.by_name.rate_cmd;
      qlim = evalin('base', 'sim_rate_cmd_lim_q');
      plim = evalin('base', 'sim_rate_cmd_lim_p');
      act = sum(abs(abs(rc(:, 2)) - qlim) < 1e-9) + sum(abs(abs(rc(:, 1)) - plim) < 1e-9);
      fprintf(fid, '%-30s %12.4e %12.4e %12s  %d samples at a rate clamp\n', ...
         cfg(k).name, mx, max(d ./ scale), names{iw}, act);
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

function Y = pchip_interp(tx, X, tq)
Y = zeros(numel(tq), size(X, 2));
for j = 1:size(X, 2)
   Y(:, j) = interp1(tx, X(:, j), tq, 'pchip');
end
end
