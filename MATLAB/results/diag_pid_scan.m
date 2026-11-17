% diag_pid_scan Scan the real factorisation: Kp, Kd, integral zero fraction.
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

fprintf('\n== p rate loop ==\n');
scan_pd(Gp, logspace(log10(0.3), log10(60), 27), [0, 0.002, 0.005, 0.01, 0.02, 0.05], Tf, wc.rate_p_rad_s, tol);
fprintf('\n== q rate loop ==\n');
scan_pd(Gq, logspace(log10(0.5), log10(1.6), 23), [0, 0.001, 0.002, 0.005, 0.01, 0.02], Tf, wc.rate_q_rad_s, tol);
fprintf('\n== r rate loop ==\n');
scan_pd(Gr, logspace(log10(0.05), log10(0.5), 21), [0, 0.002, 0.005, 0.01, 0.02, 0.05], Tf, wc.rate_r_rad_s, tol);

Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
Kp_q = 1 / abs(freqresp(Gq, 1i * wc.rate_q_rad_s));
Tp = feedback(ss(tf([Kp_p * Tf, Kp_p], [Tf, 1])) * Gp, 1);
Tq = feedback(ss(tf([Kp_q * Tf, Kp_q], [Tf, 1])) * Gq, 1);
np = size(Tp.A, 1); nq = size(Tq.A, 1);
Gphi = ss([Tp.A, zeros(np, 1); Tp.C, 0], [Tp.B; Tp.D], [zeros(1, np), 1], 0);
Gtheta = ss([Tq.A, zeros(nq, 1); Tq.C, 0], [Tq.B; Tq.D], [zeros(1, nq), 1], 0);

fprintf('\n== phi ==\n');
scan_pi(Gphi, logspace(log10(0.3), log10(60), 27), [0, 0.2, 0.5, 1, 2], wc.att_phi_rad_s, tol);
fprintf('\n== theta ==\n');
scan_pi(Gtheta, logspace(log10(0.05), log10(6), 27), [0, 0.2, 0.5, 1, 2], wc.att_theta_rad_s, tol);
fprintf('\n== V ==\n');
scan_pi(GV, logspace(log10(0.02), log10(2), 27), [0, 0.1, 0.2, 0.5, 1], wc.speed_rad_s, tol);

% h loop uses the closed theta loop; use the theta design found by hand later,
% here with a unit-gain theta loop for shape only
Th = feedback(ss(tf([0.8, 0.8 * 0.5 * wc.att_theta_rad_s], [1, 0])) * Gtheta, 1);
Gh = Th * (lc.x_op_long(1) * tf(1, [1 0]));
fprintf('\n== h ==\n');
scan_pi(Gh, logspace(log10(0.005), log10(1), 27), [0, 0.1, 0.2, 0.5, 1], wc.height_rad_s, tol);

function scan_pd(G, kgrid, kdgrid, Tf, wtgt, tol)
best = [];
for kp = kgrid
   for kd = kdgrid
      C = ss(tf([kp * Tf + kd, kp], [Tf, 1]));
      [ok, worst, pm, gm, wc, bw] = local_eval(C, G, tol);
      if ~ok, continue; end
      if isempty(best) || local_cost(pm, wc, wtgt) < best.cost
         best = struct('cost', local_cost(pm, wc, wtgt), 'kp', kp, 'kd', kd, 'worst', worst, ...
            'pm', pm, 'gm', gm, 'wc', wc, 'bw', bw);
      end
   end
end
if isempty(best)
   fprintf('  none stable\n');
else
   fprintf('  best Kp %8.4f Kd %7.4f pm %6.2f gm %6.2f wc %8.4f bw %8.4f worst %10.3e\n', ...
      best.kp, best.kd, best.pm, best.gm, best.wc, best.bw, best.worst);
end
% also report the whole stable set compactly
for kd = kdgrid
   s = '';
   for kp = kgrid
      C = ss(tf([kp * Tf + kd, kp], [Tf, 1]));
      [ok, ~, pm, ~, wcx, ~] = local_eval(C, G, tol);
      if ok
         s = [s sprintf('(%.3g,%.4g,%.0f) ', kp, wcx, pm)]; %#ok<AGROW>
      end
   end
   fprintf('  Kd %6.3f stable (Kp,wc,pm): %s\n', kd, s);
end
end

function scan_pi(G, kgrid, fgrid, wtgt, tol)
for f = fgrid
   s = '';
   best = [];
   for kp = kgrid
      C = ss(tf([kp, kp * f * wtgt], [1, 0]));
      [ok, worst, pm, gm, wc, bw] = local_eval(C, G, tol);
      if ~ok, continue; end
      s = [s sprintf('(%.3g,%.4g,%.0f) ', kp, wc, pm)]; %#ok<AGROW>
      if isempty(best) || local_cost(pm, wc, wtgt) < best.cost
         best = struct('cost', local_cost(pm, wc, wtgt), 'kp', kp, 'worst', worst, ...
            'pm', pm, 'gm', gm, 'wc', wc, 'bw', bw);
      end
   end
   if isempty(best)
      fprintf('  f %4.2f none stable\n', f);
   else
      fprintf('  f %4.2f best Kp %8.4f pm %6.2f gm %6.2f wc %8.4f bw %8.4f worst %10.3e\n', ...
         f, best.kp, best.pm, best.gm, best.wc, best.bw, best.worst);
      fprintf('        stable (Kp,wc,pm): %s\n', s);
   end
end
end

function c = local_cost(pm, wc, wtgt)
% want phase margin at least 45 deg and crossover near the target
c = max(0, 45 - pm) / 45 + abs(log(wc / wtgt)) * 0.5;
end

function [ok, worst, pm, gm, wc, bw] = local_eval(C, G, tol)
L = C * G;
CL = feedback(L, 1);
p = pole(CL);
worst = max(real(p));
ok = worst < -tol;
w = logspace(-4, 4, 6000);
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
