% diag_pid_new Run the redesigned design_pid on rev 8 and print the seven loops.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_new.log');
fid = fopen(out, 'w');
loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
fprintf(fid, 'rev %g mass %.2f, trim de %.4f deg alpha %.4f deg throttle %.4f\n', ...
   P.meta.revision, P.mass.m, trim.de_deg, trim.alpha_deg, trim.throttle);
t0 = tic;
for k = 1:numel(loops)
   nm = loops{k};
   if ~any(strcmp({'p', 'q', 'r', 'phi', 'theta', 'V', 'h'}, nm))
      continue;
   end
   tk = tic;
   try
      gp = design_pid(P, lin, struct('which', {nm}, 'verbose', true));
      g = gp.(nm); v = gp.verify.(nm);
      fprintf(fid, ['%-6s Kp %+14.8g Ki %+14.8g Kd %+14.8g Tf %8.5f pol %+d stable %d\n' ...
         '       bw %9.4f target %8.4f ratio %7.4f | pm %7.3f gm %7.3f gm_dB %7.2f | rise %8.4f os %6.2f%%\n' ...
         '       worst_real %+12.4e artifact_share %10.3e n_tried %6d n_stable %6d score %8.4f [%.1f s]\n'], ...
         nm, g.Kp, g.Ki, g.Kd, g.Tf, g.sign, v.stable, v.bandwidth_rad_s, v.wc_target_rad_s, ...
         v.bandwidth_ratio, v.pm_deg, v.gm, v.gm_db, v.rise_time_s, v.overshoot_pct, ...
         v.worst_real, v.artifact_share, g.search_tried, g.search_stable, g.score, toc(tk));
      if isfield(g, 'ki_reason') && ~isempty(g.ki_reason)
         fprintf(fid, '       ki_reason: %s\n', g.ki_reason);
      end
      if isfield(v, 'stability_note')
         fprintf(fid, '       stability_note: %s\n', v.stability_note);
      end
   catch ME
      fprintf(fid, '%-6s FAILED %s | %s [%.1f s]\n', nm, ME.identifier, ME.message, toc(tk));
   end
   fclose(fid); fid = fopen(out, 'a');
end
fprintf(fid, 'total %.1f s\n', toc(t0));
fclose(fid);
type(out);
