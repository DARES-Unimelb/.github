% diag_pid_vmax Push the V loop until it breaks, then report the ceiling.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_vmax.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * tf(1, [P.prop.esc.tau_s, 1]);
fprintf(fid, 'GV zeros %s, dcgain %.6g\n', mat2str(sort(zero(GV)).', 6), dcgain(GV));
Kp_nom = 1 / abs(freqresp(GV, 1i * P.ctrl.bw.speed_rad_s));
fprintf(fid, 'target wc %.3f, Kp_nom %.6g\n', P.ctrl.bw.speed_rad_s, Kp_nom);
fprintf(fid, '%-8s %-6s %12s %10s %10s %10s %10s\n', 'scale', 'f', 'worst', 'bw', 'rise', 'os', 'dc');
for scale = [1, 3, 10, 30, 100, 300, 1000, 3000]
   for f = [0, 0.3, 1, 3]
      C = scale * Kp_nom * tf([1, f * P.ctrl.bw.speed_rad_s], [1, 0]);
      T = feedback(C * GV, 1);
      p = pole(T);
      w = max(real(p));
      if w < -1e-7
         b = NaN;
         try, b = bandwidth(T); catch, end %#ok<CTCH>
         si = stepinfo(T);
         fprintf(fid, '%-8g %-6g %12.4e %10.5f %10.4f %10.2f %10.5f\n', scale, f, w, b, ...
            si.RiseTime, si.Overshoot, dcgain(T));
      else
         fprintf(fid, '%-8g %-6g %12.4e %10s\n', scale, f, w, 'unstable');
      end
   end
end
fclose(fid);
type(out);
