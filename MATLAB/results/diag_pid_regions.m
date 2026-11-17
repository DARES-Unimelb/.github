% diag_pid_regions Map feasible gain regions: stability and phase margin.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
wc = P.ctrl.bw; Tf = P.ctrl.rate_filter_tau_s;
lag.da = tf(1, [P.act.aileron.tau_s, 1]);
lag.de = tf(1, [P.act.elevator.tau_s, 1]);
lag.dr = tf(1, [P.act.rudder.tau_s, 1]);
lag.T = tf(1, [P.prop.esc.tau_s, 1]);
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag.da;
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag.de;
Gr = ss(lc.A_lat, lc.B_lat(:, 2), [0 0 1 0], 0) * lag.dr;
GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * lag.T;
tol = 1e-7;

fprintf('\n=== inner rate loops: Kp grid, Kd grid (stable? pm deg) ===\n');
Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
Kp_q = 1 / abs(freqresp(Gq, 1i * wc.rate_q_rad_s));
Kp_r = 1 / abs(freqresp(Gr, 1i * wc.rate_r_rad_s));
inner = {'p', Gp, Kp_p * [31.25 62.5 125 250]; 'q', Gq, Kp_q * [0.5 1 2 4 8]; ...
         'r', Gr, Kp_r * [0.1 0.2 0.4 0.8 1.6]};
for k = 1:3
   nm = inner{k, 1}; G = inner{k, 2}; kg = inner{k, 3};
   fprintf('-- %s --\n', nm);
   for kp = kg
      line = '';
      for kd = [0, 0.005, 0.01, 0.02, 0.05, 0.1]
         C = ss(tf([kp * Tf + kd, kp], [Tf, 1]));
         CL = feedback(C * G, 1);
         w = max(real(pole(CL)));
         [pm, gm] = local_margins(C * G);
         if w < -tol
            line = [line sprintf(' Kd%6.3f:kp%8.3f:pm%6.1f:gm%6.1f', kd, kp, pm, gm)]; %#ok<AGROW>
         else
            line = [line sprintf(' Kd%6.3f:UNSTABLE ', kd)]; %#ok<AGROW>
         end
      end
      fprintf('%s\n', line);
   end
end

Tq = feedback(ss(tf([Kp_q * Tf, Kp_q], [Tf, 1])) * Gq, 1);
Tp = feedback(ss(tf([Kp_p * Tf, Kp_p], [Tf, 1])) * Gp, 1);
np = size(Tp.A, 1); nq = size(Tq.A, 1);
Gphi = ss([Tp.A, zeros(np, 1); Tp.C, 0], [Tp.B; Tp.D], [zeros(1, np), 1], 0);
Gtheta = ss([Tq.A, zeros(nq, 1); Tq.C, 0], [Tq.B; Tq.D], [zeros(1, nq), 1], 0);

fprintf('\n=== phi: sgn, Kp, f -> worst / pm ===\n');
for sgn = [1, -1]
   for f = [0, 0.5]
      for kp = [0.5, 1, 2, 4, 8, 16, 32, 64, 128]
         C = ss(tf([sgn * kp, sgn * kp * f * wc.att_phi_rad_s], [1, 0]));
         CL = feedback(C * Gphi, 1);
         w = max(real(pole(CL)));
         [pm, gm] = local_margins(C * Gphi);
         fprintf('  sgn %+d f %4.2f Kp %8.3f worst %12.4e pm %7.2f gm %7.2f\n', ...
            sgn, f, kp, w, pm, gm);
      end
   end
end

fprintf('\n=== theta: sgn, Kp, f -> worst / pm ===\n');
for sgn = [1, -1]
   for f = [0, 0.5]
      for kp = [0.05, 0.1, 0.2, 0.4, 0.8, 1.6, 3.2, 6.4]
         C = ss(tf([sgn * kp, sgn * kp * f * wc.att_theta_rad_s], [1, 0]));
         CL = feedback(C * Gtheta, 1);
         w = max(real(pole(CL)));
         [pm, gm] = local_margins(C * Gtheta);
         fprintf('  sgn %+d f %4.2f Kp %8.3f worst %12.4e pm %7.2f gm %7.2f\n', ...
            sgn, f, kp, w, pm, gm);
      end
   end
end

fprintf('\n=== V: sgn, Kp, f -> worst / pm ===\n');
for sgn = [1, -1]
   for f = [0, 0.2]
      for kp = [0.02, 0.05, 0.1, 0.2, 0.5, 1, 2]
         C = ss(tf([sgn * kp, sgn * kp * f * wc.speed_rad_s], [1, 0]));
         CL = feedback(C * GV, 1);
         w = max(real(pole(CL)));
         [pm, gm] = local_margins(C * GV);
         fprintf('  sgn %+d f %4.2f Kp %8.3f worst %12.4e pm %7.2f gm %7.2f\n', ...
            sgn, f, kp, w, pm, gm);
      end
   end
end

function [pm_deg, gm_db] = local_margins(L)
w = logspace(-4, 4, 9000);
H = squeeze(freqresp(L, 1i * w));
mag = abs(H(:)); ph = angle(H(:)) * 180 / pi;
i = find(mag(1:end - 1) >= 1 & mag(2:end) < 1, 1);
if isempty(i)
   pm_deg = NaN;
else
   pm_deg = 180 + ph(i);
end
j = find(ph(1:end - 1) > -180 & ph(2:end) <= -180, 1);
if isempty(j)
   gm_db = Inf;
else
   gm_db = -20 * log10(mag(j));
end
end
