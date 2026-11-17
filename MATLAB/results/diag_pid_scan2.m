% diag_pid_scan2 State-space closed-loop scan, artifact-free.
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
tol = 1e-6;

Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
Kp_q = 1 / abs(freqresp(Gq, 1i * wc.rate_q_rad_s));
Tp = feedback(ss(tf([Kp_p * Tf, Kp_p], [Tf, 1])) * Gp, 1);
Tq = feedback(ss(tf([Kp_q * Tf, Kp_q], [Tf, 1])) * Gq, 1);
np = size(Tp.A, 1); nq = size(Tq.A, 1);
Gphi = ss([Tp.A, zeros(np, 1); Tp.C, 0], [Tp.B; Tp.D], [zeros(1, np), 1], 0);
Gtheta = ss([Tq.A, zeros(nq, 1); Tq.C, 0], [Tq.B; Tq.D], [zeros(1, nq), 1], 0);
fprintf('Tp states %d, Tq states %d\n', np, nq);

fprintf('\n== phi, state space ==\n');
scan_pi(Gphi, logspace(log10(0.3), log10(120), 33), [0, 0.2, 0.5, 1, 2], wc.att_phi_rad_s, tol);
fprintf('\n== theta, state space ==\n');
scan_pi(Gtheta, logspace(log10(0.05), log10(8), 33), [0, 0.2, 0.5, 1, 2], wc.att_theta_rad_s, tol);
fprintf('\n== h over the theta loop, f 0.5 ==\n');
Kth = 0.8; fth = 0.5;
Th = feedback(ss(tf([Kth, Kth * fth * wc.att_theta_rad_s], [1, 0])) * Gtheta, 1);
nh = size(Th.A, 1);
Gh = ss([Th.A, zeros(nh, 1); Th.C, 0], [Th.B; Th.D], [zeros(1, nh), 1], 0) * lc.x_op_long(1);
scan_pi(Gh, logspace(log10(0.002), log10(0.5), 33), [0, 0.2, 0.5, 1], wc.height_rad_s, tol);

function scan_pi(G, kgrid, fgrid, wtgt, tol)
for f = fgrid
   s = ''; best = [];
   for kp = kgrid
      C = ss(tf([kp, kp * f * wtgt], [1, 0]));
      [ok, worst, pm, gm, wcx, bw] = local_eval(C, G, tol);
      if ~ok, continue; end
      s = [s sprintf('(%.4g,%.4g,%.0f) ', kp, wcx, pm)]; %#ok<AGROW>
      if isempty(best) || local_cost(pm, wcx, wtgt) < best.cost
         best = struct('cost', local_cost(pm, wcx, wtgt), 'kp', kp, 'worst', worst, ...
            'pm', pm, 'gm', gm, 'wc', wcx, 'bw', bw);
      end
   end
   if isempty(best)
      fprintf('  f %4.2f none stable\n', f);
   else
      fprintf('  f %4.2f best Kp %9.4f pm %6.2f gm %6.2f wc %8.4f bw %8.4f worst %10.3e\n', ...
         f, best.kp, best.pm, best.gm, best.wc, best.bw, best.worst);
      fprintf('        stable (Kp,wc,pm): %s\n', s);
   end
end
end

function c = local_cost(pm, wc, wtgt)
c = max(0, 45 - pm) / 45 + abs(log(wc / wtgt)) * 0.5;
end

function [ok, worst, pm, gm, wc, bw] = local_eval(C, G, tol)
L = C * G;
CL = feedback(L, 1);
p = pole(CL);
worst = max(real(p));
ok = worst < -tol;
w = logspace(-5, 4, 6000);
H = squeeze(freqresp(L, 1i * w));
mag = abs(H(:)); ph = angle(H(:)) * 180 / pi;
i = find(mag(1:end - 1) >= 1 & mag(2:end) < 1, 1);
if isempty(i)
   pm = NaN; wc = NaN;
else
   wc = w(i); pm = 180 + ph(i);
end
j = find(ph(1:end - 1) > -180 & ph(2:end) <= -180, 1);
if isempty(j)
   gm = Inf;
else
   gm = -20 * log10(mag(j));
end
Ht = squeeze(freqresp(CL, 1i * w));
mt = abs(Ht(:));
i2 = find(mt(1:end - 1) >= 1 / sqrt(2) & mt(2:end) < 1 / sqrt(2), 1);
if isempty(i2)
   bw = NaN;
else
   bw = w(i2);
end
end
