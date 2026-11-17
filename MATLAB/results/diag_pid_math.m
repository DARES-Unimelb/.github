% diag_pid_math Validate the design_pid search logic without the params self-check.
% Calls uacs_twin_params directly, so it works while params/ is mid-edit. The
% official pipeline still goes through init_uacs_twin.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_math.log');
fid = fopen(out, 'w');
loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
P = uacs_twin_params();
assignin('base', 'P', P);
fprintf(fid, 'rev %g mass %.3f V_min %.4f V_max %.1f (params direct, no self check)\n', ...
   P.meta.revision, P.mass.m, P.limits.V_min_mps, P.limits.V_max_mps);
try
   trim = trim_cruise(P, P.op.cruise);
   lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
   lc = lin_contract(lin);
   lag = @(t) tf(1, [t, 1]);
   Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag(P.act.aileron.tau_s);
   Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag(P.act.elevator.tau_s);
   Gr = ss(lc.A_lat, lc.B_lat(:, 2), [0 0 1 0], 0) * lag(P.act.rudder.tau_s);
   GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * lag(P.prop.esc.tau_s);
   fprintf(fid, 'trim V %.2f alpha %.4f deg theta %.4f deg de %.4f deg throttle %.4f\n', ...
      trim.V_mps, trim.alpha_deg, trim.theta_deg, trim.de_deg, trim.throttle);
   fprintf(fid, 'lat poles %s\n', mat2str(sort(eig(lc.A_lat), 'ComparisonMethod', 'real').', 5));
   fprintf(fid, 'long poles %s\n', mat2str(sort(eig(lc.A_long), 'ComparisonMethod', 'real').', 5));
   for nm = {'Gp', 'Gq', 'Gr', 'GV'}
      fprintf(fid, '%s zeros %s\n', nm{1}, mat2str(sort(zero(eval(nm{1})), 'ComparisonMethod', 'real').', 6));
   end
   fclose(fid); fid = fopen(out, 'a');
catch ME
   fprintf(fid, 'SETUP FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
   fclose(fid); type(out); return;
end
t0 = tic;
try
   gp = design_pid(P, lin, struct('verbose', false));   fprintf(fid, 'design_pid OK in %.1f s\n', toc(t0));
   for k = 1:numel(loops)
      nm = loops{k};
      g = gp.(nm); v = gp.verify.(nm);
      fprintf(fid, ['%-6s Kp %+15.8g Ki %+15.8g Kd %+15.8g Tf %8.5f pol %+d stable %d\n' ...
         '       bw %11.4f target %8.4f ratio %8.4f pm %8.3f gm %8.3f rise %9.4f os %6.2f settle %9.4f\n' ...
         '       worst %+12.4e art_share %10.3e tried %6d stable %6d score %8.4f\n'], ...
         nm, g.Kp, g.Ki, g.Kd, g.Tf, g.sign, v.stable, v.bandwidth_rad_s, v.wc_target_rad_s, ...
         v.bandwidth_ratio, v.pm_deg, v.gm, v.rise_time_s, v.overshoot_pct, v.settling_time_s, ...
         v.worst_real, v.artifact_share, g.search_tried, g.search_stable, g.score);
      if isfield(g, 'ki_reason') && ~isempty(g.ki_reason)
         fprintf(fid, '       %s\n', g.ki_reason);
      end
      if isfield(v, 'mode_shares')
         [~, ord] = sort(real(v.mode_shares.poles), 'descend');
         for j = ord(1:min(4, numel(ord)))'
            fprintf(fid, '       pole %+11.5e %+11.5e i  share %.3e\n', ...
               real(v.mode_shares.poles(j)), imag(v.mode_shares.poles(j)), v.mode_shares.share(j));
         end
      end
   end
   s = gp.verify.separation;
   fprintf(fid, 'separation phi/p %.3f theta/q %.3f V/q %.3f h/theta %.3f h/V %.3f limit %.2f ok %d\n', ...
      s.phi_over_p, s.theta_over_q, s.V_over_q, s.h_over_theta, s.h_over_V, s.limit, s.ok);
   fprintf(fid, 'polarity %s\n', mat2str([gp.polarity.p gp.polarity.q gp.polarity.r ...
      gp.polarity.phi gp.polarity.theta gp.polarity.V gp.polarity.h]));
catch ME
   fprintf(fid, 'DESIGN FAILED %s | %s after %.1f s\n', ME.identifier, ME.message, toc(t0));
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);
