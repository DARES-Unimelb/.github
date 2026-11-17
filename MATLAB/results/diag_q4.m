% diag_q4 Fine scan of the q rate loop: PI, PID with a small integral gain.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_q4.log');
fid = fopen(out, 'w');
try
   P = uacs_twin_params();
   assignin('base', 'P', P);
   trim = trim_cruise(P, P.op.cruise);
   lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
   lc = lin_contract(lin);
   Tf = P.ctrl.rate_filter_tau_s;
   wc = P.ctrl.bw.rate_q_rad_s;
   Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * tf(1, [P.act.elevator.tau_s, 1]);
   Kp_nom = 1 / abs(freqresp(Gq, 1i * wc));
   fprintf(fid, 'Kp_nom %.6g wc %.3f\n', Kp_nom, wc);
   best = struct('cost', Inf, 'kp', NaN, 'ki', NaN, 'kd', NaN, 'bw', NaN, 'rise', NaN, ...
      'os', NaN, 'dc', NaN);
   n_ok = 0;
   for kp = logspace(log10(1e-3 * Kp_nom), log10(1e3 * Kp_nom), 120)
      for ki = wc * [0.001, 0.005, 0.02, 0.05, 0.2, 0.6, 2.0]
         for kd = [0, 1e-5, 1e-4, 1e-3, 1e-2]
            for kf = [0.5, 1, 2]
               cn = [kp * (kf * Tf) + kd, kp + ki * (kf * Tf), ki];
               cd = [kf * Tf, 1, 0];
               T = feedback(tf(cn, cd) * Gq, 1);
               p = pole(T);
               if max(real(p)) >= -1e-7
                  continue;
               end
               dc = dcgain(T);
               if abs(dc) < 1e-9
                  continue;
               end
               n_ok = n_ok + 1;
               b = NaN; try, b = bandwidth(T); catch, end %#ok<CTCH>
               tv = linspace(0, 4, 200); yv = step(T, tv);
               yss = yv(end);
               rise = NaN; os = NaN;
               if isfinite(yss) && abs(yss) > 1e-12
                  k10 = find(yv >= 0.1 * yss, 1); k90 = find(yv >= 0.9 * yss, 1);
                  if ~isempty(k10) && ~isempty(k90) && k90 > k10
                     rise = tv(k90) - tv(k10);
                  end
                  os = 100 * max(0, (max(yv) - yss) / yss);
               end
               if isnan(rise)
                  continue;
               end
               cost = abs(1 - dc) * 10 + abs(log(max(b, realmin) / wc)) + (os / 100)^2 * 5;
               if cost < best.cost
                  best = struct('cost', cost, 'kp', kp, 'ki', ki, 'kd', kd, 'kf', kf, ...
                     'bw', b, 'rise', rise, 'os', os, 'dc', dc);
               end
            end
         end
      end
   end
   fprintf(fid, 'stable sets with nonzero dc: %d\n', n_ok);
   fprintf(fid, 'BEST cost %.4f kp %.6g ki %.6g kd %.6g kf %.2f | bw %.5f rise %.4f os %.2f dc %.5f\n', ...
      best.cost, best.kp, best.ki, best.kd, best.kf, best.bw, best.rise, best.os, best.dc);
catch ME
   fprintf(fid, 'FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);
