% diag_design_v3 Design all seven loops in ONE call so the outer loops can use
% the inner ones, and save the gains for the job.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_design_v3.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
fprintf(fid, 'rev %g mass %.3f\n', P.meta.revision, P.mass.m);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
fprintf(fid, 'trim V %.2f alpha %.4f de %.4f throttle %.4f\n', trim.V_mps, ...
   trim.alpha_deg, trim.de_deg, trim.throttle);
fclose(fid); fid = fopen(out, 'a');
t0 = tic;
try
   gp = design_pid(P, lin, struct('verbose', false));
   fprintf(fid, 'design_pid OK in %.1f s\n', toc(t0));
   loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
   for k = 1:numel(loops)
      nm = loops{k};
      g = gp.(nm); v = gp.verify.(nm);
      fprintf(fid, ['%-6s Kp %+14.8g Ki %+14.8g Kd %+14.8g Tf %8.5f pol %+d stable %d designed %d\n' ...
         '       crossover %10.4f (target %7.4f)  bw %10.4f  pm %8.2f gm %8.3f\n' ...
         '       rise %9.4f os %7.2f settle %9.4f dc %8.5f\n' ...
         '       worst %+12.4e art %10.3e tried %5d stable %5d score %8.4f\n'], ...
         nm, g.Kp, g.Ki, g.Kd, g.Tf, g.sign, v.stable, g.designed, ...
         v.wcp_rad_s, v.wc_target_rad_s, v.bandwidth_rad_s, v.pm_deg, v.gm, ...
         v.rise_time_s, v.overshoot_pct, v.settling_time_s, ...
         v.dc_gain, v.worst_real, v.artifact_share, g.search_tried, g.search_stable, g.score);
      if isfield(g, 'ki_reason') && ~isempty(g.ki_reason)
         fprintf(fid, '       %s\n', g.ki_reason);
      end
   end
   s = gp.verify.separation;
   fprintf(fid, 'separation phi/p %.3f theta/q %.3f V/q %.3f h/theta %.3f h/V %.3f limit %.2f ok %d\n', ...
      s.phi_over_p, s.theta_over_q, s.V_over_q, s.h_over_theta, s.h_over_V, s.limit, s.ok);
   fprintf(fid, 'polarity %s\n', mat2str([gp.polarity.p gp.polarity.q gp.polarity.r ...
      gp.polarity.phi gp.polarity.theta gp.polarity.V gp.polarity.h]));
   save(fullfile(root, 'results', 'diag_gains_v3.mat'), 'gp');
   fprintf(fid, 'saved diag_gains_v3.mat\n');
catch ME
   fprintf(fid, 'DESIGN FAILED %s | %s after %.1f s\n', ME.identifier, ME.message, toc(t0));
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
disp(fileread(out));
