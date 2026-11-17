% diag_vh Scan the V and h loops directly for a usable design.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_vh.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
Tf = P.ctrl.rate_filter_tau_s;
lag = @(t) tf(1, [t, 1]);
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag(P.act.aileron.tau_s);
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag(P.act.elevator.tau_s);
GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * lag(P.prop.esc.tau_s);
fprintf(fid, 'GV poles %s zeros %s\n', mat2str(sort(pole(GV)).', 5), mat2str(sort(zero(GV)).', 5));
fprintf(fid, 'GV dcgain %.6g\n', dcgain(GV));
% h plant from a fitted theta loop (use the theta design the run finds later; here
% use a representative one) then times V0/s
Kp_q = 1 / abs(freqresp(Gq, 1i * P.ctrl.bw.rate_q_rad_s));
Tq = feedback(ss(tf([Kp_q * Tf, Kp_q], [Tf, 1])) * Gq, 1);
Gth = ss([Tq.A, zeros(size(Tq.A, 1), 1); Tq.C, 0], [Tq.B; Tq.D], ...
   [zeros(1, size(Tq.A, 1)), 1], 0);
Gh = Gth * (lc.x_op_long(1) * tf(1, [1 0]));
fprintf(fid, 'Gh poles %s\n', mat2str(sort(pole(Gh)).', 5));
fprintf(fid, 'Gh zeros %s\n', mat2str(sort(zero(Gh)).', 5));

for nm = {'V', 'h'}
   if strcmp(nm{1}, 'V')
      G = GV; wc = P.ctrl.bw.speed_rad_s;
   else
      G = Gh; wc = P.ctrl.bw.height_rad_s;
   end
   Kp_nom = 1 / abs(freqresp(G, 1i * wc));
   fprintf(fid, '\n=== %s: wc %.3f, Kp_nom %.5g, dcgain %.5g ===\n', nm{1}, wc, Kp_nom, dcgain(G));
   for scale = [0.01, 0.03, 0.1, 0.3, 1, 3, 10, 30]
      for f = [0, 0.1, 0.3, 1.0]
         C = scale * Kp_nom * tf([1, f * wc], [1, 0]);
         T = feedback(C * G, 1);
         p = pole(T);
         line = sprintf('  Kp_scale %6.2f f %4.2f worst %11.3e', scale, f, max(real(p)));
         if max(real(p)) < -1e-7
            si = stepinfo(T);
            line = sprintf('%s | bw %8.4f rise %8.4f os %7.2f dc %6.4f', line, ...
               bandwidth_safe(T), si.RiseTime, si.Overshoot, dcgain(T));
         end
         fprintf(fid, '%s\n', line);
      end
   end
end
fclose(fid);
type(out);

function b = bandwidth_safe(T)
try
   b = bandwidth(T);
catch
   b = NaN;
end
end
