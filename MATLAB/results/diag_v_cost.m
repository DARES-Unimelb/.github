% diag_v_cost Evaluate the search cost of the V loop on a fine grid.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_vcost.log');
fid = fopen(out, 'w');
try
   P = uacs_twin_params();
   assignin('base', 'P', P);
   trim = trim_cruise(P, P.op.cruise);
   lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
   lc = lin_contract(lin);
   GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * tf(1, [P.prop.esc.tau_s, 1]);
   wc = P.ctrl.bw.speed_rad_s;
   Kp_nom = 1 / abs(freqresp(GV, 1i * wc));
   fprintf(fid, 'wc %.3f Kp_nom %.6g dcgain %.6g\n', wc, Kp_nom, dcgain(GV));
   best = struct('cost', Inf);
   for sgn = [1, -1]
      for kp = logspace(log10(1e-3 * Kp_nom), log10(1e3 * Kp_nom), 121)
         for ki = wc * [0.05 0.2 0.5 1.0 2.0 3.5 6.0]
            [c, d] = local_v_cost(sgn * kp, sgn * ki, GV, wc);
            if c < best.cost
               best = struct('cost', c, 'kp', kp, 'ki', ki, 'sgn', sgn, 'd', d);
            end
         end
      end
   end
   fprintf(fid, 'BEST cost %.4f sgn %+d kp %.6g ki %.6g | %s\n', best.cost, best.sgn, ...
      best.kp, best.ki, best.d);
catch ME
   fprintf(fid, 'FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);

function [c, d] = local_v_cost(kp, ki, GV, wc)
c = 1e6;
d = 'unstable';
if kp == 0
   return;
end
T = minreal(feedback(tf([kp, ki], [1, 0]) * GV, 1), 1e-7);
p = pole(T);
if max(real(p)) >= -1e-7
   return;
end
tv = linspace(0, max(30 / wc, 4), 100);
yv = step(T, tv);
yss = yv(end);
if ~isfinite(yss) || abs(yss) < 1e-9
   return;
end
k10 = find(yv >= 0.1 * yss, 1);
k90 = find(yv >= 0.9 * yss, 1);
if isempty(k10) || isempty(k90) || k90 <= k10
   return;
end
rise = tv(k90) - tv(k10);
os = 100 * max(0, (max(yv) - yss) / yss);
tail = yv(round(0.75 * numel(yv)):end);
rip = 100 * (max(tail) - min(tail)) / abs(yss);
bw = NaN;
try
   bw = bandwidth(T);
catch
end
dc = dcgain(T);
if ~isfinite(dc) || isnan(bw)
   return;
end
r = log(bw / wc);
e_slow = max(0, -r);
e_fast = max(0, r);
e_rise = log(rise / (2.2 / wc));
if os <= 8
   e_os = 0;
elseif os <= 20
   e_os = (os - 8) / 100;
else
   e_os = 0.12 + ((os - 20) / 100)^2 * 10;
end
e_rip = max(0, rip - 2) / 100;
c = 40 * abs(1 - dc)^2 + 0.3 * e_slow^2 + 0.02 * e_fast^2 + 1.0 * e_rise^2 ...
   + 2 * e_os + 2 * e_rip;
d = sprintf('bw %.5f rise %.4f os %.2f rip %.3f dc %.5f', bw, rise, os, rip, dc);
end
