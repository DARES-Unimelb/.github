% diag_pid_run Call the delivered design_pid and report the failure verbatim.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
try
   gp = design_pid(P, lin);
   fn = fieldnames(gp);
   fprintf('design_pid returned. fields: %s\n', strjoin(fn', ','));
   for nm = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'}
      g = gp.(nm{1});
      fprintf('%5s Kp %14.6g Ki %14.6g Kd %14.6g sign %+d wc %g stable %d bw %.4g rise %.4g os %.3g\n', ...
         nm{1}, g.Kp, g.Ki, g.Kd, g.sign, g.wc_target_rad_s, gp.verify.(nm{1}).stable, ...
         gp.verify.(nm{1}).bandwidth_rad_s, gp.verify.(nm{1}).rise_time_s, ...
         gp.verify.(nm{1}).overshoot_pct);
   end
catch ME
   fprintf('design_pid FAILED\n  id: %s\n  msg: %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf('  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
