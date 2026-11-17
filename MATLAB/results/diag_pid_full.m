% diag_pid_full Evaluate the full gain search per loop in state space.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
wc = P.ctrl.bw;
Tf = P.ctrl.rate_filter_tau_s;
lag.da = tf(1, [P.act.aileron.tau_s, 1]);
lag.de = tf(1, [P.act.elevator.tau_s, 1]);
lag.dr = tf(1, [P.act.rudder.tau_s, 1]);
lag.T = tf(1, [P.prop.esc.tau_s, 1]);
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag.da;
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag.de;
Gr = ss(lc.A_lat, lc.B_lat(:, 2), [0 0 1 0], 0) * lag.dr;
GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * lag.T;
tol = 1e-7;

fprintf('%-6s %9s %9s %9s %11s %9s %8s %8s %8s\n', 'loop', 'Kp_ref', 'Kp_max', 'Kp_use', ...
   'worst', 'bw', 'bw_tgt', 'pm_deg', 'gm_db');

% ---- inner rate loops: Kp + Kd*s/(Tf*s+1) ----
inner = {'p', Gp, wc.rate_p_rad_s; 'q', Gq, wc.rate_q_rad_s; 'r', Gr, wc.rate_r_rad_s};
for k = 1:3
   nm = inner{k, 1}; G = inner{k, 2}; w = inner{k, 3};
   Kp_ref = 1 / abs(freqresp(G, 1i * w));
   best = [];
   for kd = [0, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5]
      C = @(kp) ss(tf([kp * Tf + kd, kp], [Tf, 1]));
      [kmax, kmaxp] = local_max_gain(G, C, Kp_ref, tol);
      if ~isempty(kmax) && (isempty(best) || kmax > best.Kp)
         best = struct('Kp', kmax, 'Kp_max', kmaxp, 'Kd', kd);
      end
   end
   if isempty(best)
      fprintf('%-6s no stable gain\n', nm); continue;
   end
   Kp = 0.7 * best.Kp;
   C = ss(tf([Kp * Tf + best.Kd, Kp], [Tf, 1]));
   CL = feedback(C * G, 1);
   [pm, gm] = local_margins(C * G);
   fprintf('%-6s %9.4f %9.4f %9.4f %11.4e %9.4f %8.3f %8.2f %8.2f  Kd %.4f\n', ...
      nm, Kp_ref, best.Kp_max, Kp, max(real(pole(CL))), local_bw(CL), w, pm, gm, best.Kd);
end

% ---- outer loops: PI closed on the closed inner loop ----
Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
Kp_q = 1 / abs(freqresp(Gq, 1i * wc.rate_q_rad_s));
Tp = feedback(ss(tf([Kp_p * Tf, Kp_p], [Tf, 1])) * Gp, 1);
Tq = feedback(ss(tf([Kp_q * Tf, Kp_q], [Tf, 1])) * Gq, 1);
Gphi = ss([Tp.A, zeros(5, 1); Tp.C, 0], [Tp.B; Tp.D], [zeros(1, 5), 1], 0);
Gtheta = ss([Tq.A, zeros(5, 1); Tq.C, 0], [Tq.B; Tq.D], [zeros(1, 5), 1], 0);

outer = {'phi', Gphi, wc.att_phi_rad_s; 'theta', Gtheta, wc.att_theta_rad_s; ...
         'V', GV, wc.speed_rad_s};
for k = 1:3
   nm = outer{k, 1}; G = outer{k, 2}; w = outer{k, 3};
   Kp_ref = 1 / abs(freqresp(G, 1i * w));
   best = [];
   for sgn = [1, -1]
      for f = [0, 0.2, 0.5, 1.0]
         C = @(kp) ss(tf([sgn * kp, sgn * kp * f * w], [1, 0]));
         [kmax, kmaxp] = local_max_gain(G, C, Kp_ref, tol);
         if ~isempty(kmax) && (isempty(best) || kmax > best.Kp)
            best = struct('Kp', kmax, 'Kp_max', kmaxp, 'sgn', sgn, 'f', f);
         end
      end
   end
   if isempty(best)
      fprintf('%-6s no stable gain\n', nm); continue;
   end
   Kp = 0.7 * best.Kp;
   C = ss(tf([Kp, Kp * best.f * w], [1, 0]));
   CL = feedback(C * G, 1);
   [pm, gm] = local_margins(C * G);
   fprintf('%-6s %9.4f %9.4f %9.4f %11.4e %9.4f %8.3f %8.2f %8.2f  sgn %+d f %.2f Ki %.4f\n', ...
      nm, Kp_ref, best.Kp_max, Kp, max(real(pole(CL))), local_bw(CL), w, pm, gm, ...
      best.sgn, best.f, Kp * best.f * w);
end

% ---- h loop: closed theta loop, then V0/s ----
Kp_th = 0.7 * 1;  % placeholder, replaced below
Th = feedback(ss(tf([1, 0.5 * wc.att_theta_rad_s], [1, 0])) * Gtheta, 1);
V0 = lc.x_op_long(1);
Gh = Th * (V0 * tf(1, [1 0]));
best = [];
for sgn = [1, -1]
   for f = [0, 0.2, 0.5, 1.0]
      C = @(kp) ss(tf([sgn * kp, sgn * kp * f * wc.height_rad_s], [1, 0]));
      [kmax, kmaxp] = local_max_gain(Gh, C, 1, tol);
      if ~isempty(kmax) && (isempty(best) || kmax > best.Kp)
         best = struct('Kp', kmax, 'Kp_max', kmaxp, 'sgn', sgn, 'f', f);
      end
   end
end
if ~isempty(best)
   Kp = 0.7 * best.Kp;
   C = ss(tf([Kp, Kp * best.f * wc.height_rad_s], [1, 0]));
   CL = feedback(C * Gh, 1);
   [pm, gm] = local_margins(C * Gh);
   fprintf('%-6s %9s %9.4f %9.4f %11.4e %9.4f %8.3f %8.2f %8.2f  sgn %+d f %.2f Ki %.4f\n', ...
      'h', '-', best.Kp_max, Kp, max(real(pole(CL))), local_bw(CL), wc.height_rad_s, ...
      pm, gm, best.sgn, best.f, Kp * best.f * wc.height_rad_s);
end

function [kmax, kmaxp] = local_max_gain(G, Cfun, kref, tol)
% Largest gain that leaves every closed-loop pole left of -tol, by bisection on
% the log gain. kref is the gain of the unit loop; it is NOT assumed stable.
lo = 0; hi = 0;
k = kref;
if ~local_stable(G, Cfun, k, tol)
   for j = 1:200
      k = k * 0.7;
      if local_stable(G, Cfun, k, tol)
         lo = k; hi = k / 0.7;
         break;
      end
      if k < 1e-14 * kref
         kmax = []; kmaxp = [];
         return;
      end
   end
else
   lo = k;
   for j = 1:200
      k = k * 1.4;
      if ~local_stable(G, Cfun, k, tol)
         hi = k;
         break;
      end
      lo = k;
   end
   if hi == 0
      kmax = []; kmaxp = [];
      return;
   end
end
kmaxp = hi;
for j = 1:200
   if (hi - lo) <= 1e-9 * max(lo, hi)
      break;
   end
   mid = sqrt(lo * hi);
   if local_stable(G, Cfun, mid, tol)
      lo = mid;
   else
      hi = mid;
   end
end
kmax = lo;
end

function ok = local_stable(G, Cfun, kp, tol)
try
   ok = max(real(pole(feedback(Cfun(kp) * G, 1)))) < -tol;
catch
   ok = false;
end
end

function [pm_deg, gm_db] = local_margins(L)
w = logspace(-4, 3, 6000);
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

function bw = local_bw(T)
w = logspace(-4, 3, 6000);
mag = abs(squeeze(freqresp(T, 1i * w)));
i = find(mag(1:end - 1) >= 1 / sqrt(2) & mag(2:end) < 1 / sqrt(2), 1);
if isempty(i)
   bw = NaN;
else
   bw = w(i);
end
end
