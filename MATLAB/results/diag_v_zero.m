% diag_v_zero The airspeed loop faces a trade of integral-zero placement against
% phase margin, at a crossover pinned to the frozen target. This scan holds the
% crossover and sweeps the zero fraction, with and without the D term.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_v_zero.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
Tf = P.ctrl.rate_filter_tau_s;
GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * tf(1, [P.prop.esc.tau_s, 1]);
w = logspace(-5, 3, 6000);

fprintf(fid, 'crossover pinned to %.3f rad/s by solving Kp so that |C(jwc)G(jwc)| = 1.\n', ...
   P.ctrl.bw.speed_rad_s);
fprintf(fid, 'zero fraction zf = (Ki/Kp)/wc, so zf = 0.2 is the rule in the header.\n');
fprintf(fid, 'D fraction df = Kd/(Kp/wc), filter tau = Tf = %.4f s.\n\n', Tf);
fprintf(fid, '%-7s %-7s %-10s %-10s %-10s %-9s %-9s %-10s %-9s %s\n', ...
   'zf', 'df', 'Kp', 'Ki', 'Kd', 'pm_deg', 'rise_s', 'os_pct', 'rip_pct', 'bw');
wc = P.ctrl.bw.speed_rad_s;
for df = [0, 0.5, 2.0]
   for zf = [0.05, 0.1, 0.2, 0.35, 0.6, 1.0, 1.5, 2.5]
      % solve Kp so the loop crosses exactly at wc
      f = @(Kp) local_loop_mag(Kp, zf, df, wc, Tf, GV) - 1;
      try
         Kp = fzero(f, [1e-3, 1e3]);
      catch
         fprintf(fid, '%-7.2f %-7.2f %s\n', zf, df, 'no Kp puts the crossover there');
         continue;
      end
      Ki = Kp * zf * wc;
      Kd = Kp * df / wc;
      C = tf([Kp * Tf + Kd, Kp + Ki * Tf, Ki], [Tf, 1, 0]);
      L = C * GV;
      T = minreal(feedback(L, 1), 1e-9);
      pl = pole(T);
      if isempty(pl) || max(real(pl)) >= -1e-6
         fprintf(fid, '%-7.2f %-7.2f %-10.4f %-10.4f %-10.4f unstable\n', zf, df, Kp, Ki, Kd);
         continue;
      end
      HL = squeeze(freqresp(minreal(L, 1e-9), 1i * w)); HL = HL(:);
      mL = abs(HL); phL = unwrap(angle(HL)) * 180 / pi;
      kk = find(mL(1:end - 1) > 1 & mL(2:end) <= 1, 1);
      pm = NaN;
      if ~isempty(kk)
         pm = 180 + interp1(log10(w), phL, log10(w(kk)));
      end
      tv = linspace(0, 400, 4000);
      yv = step(T, tv);
      yss = yv(end);
      k10 = find(yv >= 0.1 * yss, 1); k90 = find(yv >= 0.9 * yss, 1);
      rise = NaN;
      if ~isempty(k10) && ~isempty(k90) && k90 > k10, rise = tv(k90) - tv(k10); end
      os = NaN; rip = NaN;
      if isfinite(yss) && abs(yss) > 1e-12
         os = 100 * max(0, (max(yv) - yss) / yss);
         tail = yv(round(0.75 * numel(yv)):end);
         rip = 100 * (max(tail) - min(tail)) / abs(yss);
      end
      mT = abs(squeeze(freqresp(T, 1i * w)));
      bw = NaN;
      if numel(mT) == numel(w)
         i3 = find(mT(1:end - 1) >= abs(mT(1)) / sqrt(2) & mT(2:end) < abs(mT(1)) / sqrt(2), 1);
         if ~isempty(i3), bw = w(i3); end
      end
      fprintf(fid, '%-7.2f %-7.2f %-10.4f %-10.4f %-10.4f %-9.3f %-9.4f %-10.3f %-9.4f %.4f\n', ...
         zf, df, Kp, Ki, Kd, pm, rise, os, rip, bw);
   end
   fprintf(fid, '\n');
end
fclose(fid);
type(out);

function m = local_loop_mag(Kp, zf, df, wc, Tf, GV)
Ki = Kp * zf * wc;
Kd = Kp * df / wc;
C = tf([Kp * Tf + Kd, Kp + Ki * Tf, Ki], [Tf, 1, 0]);
m = abs(freqresp(C * GV, 1i * wc));
m = m(1);
end
