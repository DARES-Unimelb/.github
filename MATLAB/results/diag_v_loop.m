% diag_v_loop Phase budget of the airspeed loop, and whether the frozen 0.8 rad/s
% crossover target is reachable at an ordinary phase margin.
% Questions, in order:
%   1. how the 49.6 deg phase margin decomposes between plant and controller
%   2. what crossover the plant supports at 60 deg, and what P.ctrl.bw.speed_rad_s asks for
%   3. PI against PID, measured, not assumed
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_v_loop.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
Tf = P.ctrl.rate_filter_tau_s;

Graw = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0);
GV = Graw * tf(1, [P.prop.esc.tau_s, 1]);
fprintf(fid, '=== 0. the loop as designed ===\n');
fprintf(fid, 'GV = dV/dT, ESC lag tau = %.4f s, rate filter Tf = %.4f s\n', P.prop.esc.tau_s, Tf);
fprintf(fid, 'frozen target P.ctrl.bw.speed_rad_s = %.4f rad/s\n', P.ctrl.bw.speed_rad_s);
fprintf(fid, 'longitudinal poles: %s\n', mat2str(sort(eig(lc.A_long)).', 5));
fprintf(fid, 'GV zeros          : %s\n', mat2str(sort(zero(GV)).', 5));
fprintf(fid, 'dcgain(GV) = %.6f\n', dcgain(GV));

S = load(fullfile(root, 'results', 'gains_baseline.mat'));
gV = S.gains_pid.V;
fprintf(fid, 'delivered: Kp %+.6f Ki %+.6f Kd %+.6f Tf %.4f\n', gV.Kp, gV.Ki, gV.Kd, gV.Tf);

fprintf(fid, '\n=== 1. phase budget at the delivered crossover ===\n');
Ctf = gV.ctrl;
w = logspace(-4, 3, 4000);
Hc = squeeze(freqresp(Ctf, 1i * w));
Hg = squeeze(freqresp(GV, 1i * w));
Hraw = squeeze(freqresp(Graw, 1i * w));
Hesc = squeeze(freqresp(tf(1, [P.prop.esc.tau_s, 1]), 1i * w));
Hc = Hc(:); Hg = Hg(:); Hraw = Hraw(:); Hesc = Hesc(:);
mag = abs(Hc .* Hg);
j = find(mag(1:end - 1) > 1 & mag(2:end) <= 1, 1);
wc = w(j);
loc = @(y) interp1(log10(w), unwrap(angle(y)) * 180 / pi, log10(wc));
fprintf(fid, 'crossover wc = %.4f rad/s (frozen target %.4f)\n', wc, P.ctrl.bw.speed_rad_s);
fprintf(fid, '  phase of controller C    %+9.3f deg\n', loc(Hc));
fprintf(fid, '  phase of plant     GV    %+9.3f deg\n', loc(Hg));
fprintf(fid, '  phase of loop      C*GV  %+9.3f deg  -> pm %+7.3f deg\n', ...
   loc(Hc .* Hg), 180 + loc(Hc .* Hg));
fprintf(fid, '  of which the ESC lag contributes %+9.3f deg\n', loc(Hesc));
fprintf(fid, '  of which the rigid-body GV contributes %+9.3f deg\n', loc(Hraw));
fprintf(fid, 'phase of GV alone at that frequency: %+.3f deg (a plant that alone eats %+.1f deg\n', ...
   loc(Hg), loc(Hg));
fprintf(fid, '  of the 180 available before the loop is on the stability boundary)\n');

fprintf(fid, '\n=== 2. what crossover does the plant support, at a target phase margin? ===\n');
fprintf(fid, 'for each candidate crossover wc_t, the proportional gain that puts |C*G| = 1 there,\n');
fprintf(fid, 'and the phase margin that results (with the integral zero placed at 0.2*wc_t as the rule says)\n');
fprintf(fid, '%-10s %-12s %-12s %-12s %s\n', 'wc_t', 'Kp', 'pm_deg', 'gm', 'note');
for wct = logspace(log10(0.05), log10(3.0), 22)
   Kp = 1 / abs(interp1(w, abs(Hg), wct));
   Ki = Kp * 0.2 * wct;
   C = tf([Kp, Ki], [1, 0]);
   L = C * GV;
   T = minreal(feedback(L, 1), 1e-9);
   if max(real(pole(T))) >= -1e-6
      fprintf(fid, '%-10.4f %-12.5f %-12s %-12s unstable\n', wct, Kp, '-', '-');
      continue;
   end
   HL = squeeze(freqresp(L, 1i * w));
   HL = HL(:);
   mL = abs(HL);
   phL = unwrap(angle(HL)) * 180 / pi;
   kk = find(mL(1:end - 1) > 1 & mL(2:end) <= 1, 1);
   pm = NaN; gm = Inf;
   if ~isempty(kk)
      pm = 180 + interp1(log10(w), phL, log10(w(kk)));
   end
   gc = find(phL(1:end - 1) > -180 & phL(2:end) <= -180, 1);
   if ~isempty(gc)
      gm = 1 / mL(gc);
   end
   note = '';
   if abs(wct - P.ctrl.bw.speed_rad_s) < 0.06
      note = '<- frozen target';
   end
   fprintf(fid, '%-10.4f %-12.5f %-12.3f %-12.4f %s\n', wct, Kp, pm, gm, note);
end

fprintf(fid, '\n=== 3. PI against PID on the airspeed loop, measured ===\n');
fprintf(fid, 'Kd swept as a multiple of Kp/wc; filter tau = 1 or 3 times Tf\n');
fprintf(fid, '%-8s %-8s %-9s %-9s %-8s %-9s %-9s %-9s %s\n', ...
   'wc_t', 'Kp', 'Ki', 'Kd', 'tau_d', 'pm_deg', 'rise_s', 'os_pct', 'bw');
for wct = [0.4 0.8 1.2]
   Kp0 = 1 / abs(interp1(w, abs(Hg), wct));
   for kdf = [0, 0.2, 0.5, 1.0]
      for kf = [1, 3]
         Ki = Kp0 * 0.2 * wct;
         Kd = Kp0 * kdf / wct;
         td = kf * Tf;
         C = tf([Kp0 * td + Kd, Kp0 + Ki * td, Ki], [td, 1, 0]);
         L = C * GV;
         T = minreal(feedback(L, 1), 1e-9);
         if max(real(pole(T))) >= -1e-6
            fprintf(fid, '%-8.2f %-8.4f %-9.4f %-9.4f %-8.4f unstable\n', wct, Kp0, Ki, Kd, td);
            continue;
         end
         HL = squeeze(freqresp(L, 1i * w)); HL = HL(:);
         mL = abs(HL); phL = unwrap(angle(HL)) * 180 / pi;
         kk = find(mL(1:end - 1) > 1 & mL(2:end) <= 1, 1);
         pm = NaN;
         if ~isempty(kk)
            pm = 180 + interp1(log10(w), phL, log10(w(kk)));
         end
         si = stepinfo(T);
         try
            bw = bandwidth(T);
         catch
            bw = NaN;
         end
         if ~isfinite(bw), bw = NaN; end
         fprintf(fid, '%-8.2f %-8.4f %-9.4f %-9.4f %-8.4f %-9.3f %-9.4f %-9.3f %.4f\n', ...
            wct, Kp0, Ki, Kd, td, pm, si.RiseTime, si.Overshoot, bw);
      end
   end
end
fclose(fid);
type(out);
