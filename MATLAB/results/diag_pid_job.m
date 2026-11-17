% diag_pid_job Full design_pid run, resilient to a channel timeout.
% Reads params directly (no self check) so it works while params/ is mid-edit.
% Progress lands in results/_diag_job.log after every loop.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_job.log');
fid = fopen(out, 'w');
loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
fprintf(fid, 'START %s\n', datestr(now, 'HH:MM:SS'));
fclose(fid);
fid = fopen(out, 'a');
P = uacs_twin_params();
assignin('base', 'P', P);
fprintf(fid, 'rev %g mass %.3f\n', P.meta.revision, P.mass.m);
try
   trim = trim_cruise(P, P.op.cruise);
   lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
catch ME
   fprintf(fid, 'SETUP FAILED %s | %s\n', ME.identifier, ME.message);
   fclose(fid); type(out); return;
end
fclose(fid); fid = fopen(out, 'a');
t0 = tic;
for k = 1:numel(loops)
   nm = loops{k};
   tk = tic;
   try
      gp = design_pid(P, lin, struct('which', {{nm}}, 'verbose', true));
      g = gp.(nm); v = gp.verify.(nm);
      fprintf(fid, ['%-6s Kp %+15.8g Ki %+15.8g Kd %+15.8g Tf %8.5f pol %+d stable %d score %.4f\n' ...
         '       bw %10.4f target %7.4f pm %8.2f gm %8.3f rise %9.4f os %7.2f settle %9.4f dc %8.5f\n' ...
         '       worst %+12.4e art %10.3e tried %6d stable %6d [%.1f s]\n'], ...
         nm, g.Kp, g.Ki, g.Kd, g.Tf, g.sign, v.stable, g.score, v.bandwidth_rad_s, ...
         v.wc_target_rad_s, v.pm_deg, v.gm, v.rise_time_s, v.overshoot_pct, v.settling_time_s, ...
         v.dc_gain, v.worst_real, v.artifact_share, g.search_tried, g.search_stable, toc(tk));
      if isfield(g, 'ki_reason') && ~isempty(g.ki_reason)
         fprintf(fid, '       %s\n', g.ki_reason);
      end
      fclose(fid); fid = fopen(out, 'a');
   catch ME
      fprintf(fid, '%-6s FAILED %s | %s [%.1f s]\n', nm, ME.identifier, ME.message, toc(tk));
      for kk = 1:numel(ME.stack)
         fprintf(fid, '        at %s line %d\n', ME.stack(kk).name, ME.stack(kk).line);
      end
      fclose(fid); fid = fopen(out, 'a');
   end
end
fprintf(fid, 'TOTAL %.1f s\n', toc(t0));
fclose(fid);
type(out);
