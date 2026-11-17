% diag_q2 Scan the q rate loop for stable PD gain sets (no integral).
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_q2.log');
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
   best = struct('cost', Inf);
   n_ok = 0;
   list = '';
   for sgn = [1, -1]
      for kp = logspace(log10(1e-3 * Kp_nom), log10(1e3 * Kp_nom), 400)
         for kd = [0, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2]
            for kf = [0.5, 2]
               cnum = sgn * [kp * (kf * Tf) + kd, kp];
               cden = [kf * Tf, 1];
               T = feedback(tf(cnum, cden) * Gq, 1);
               p = pole(T);
               if max(real(p)) >= -1e-7
                  continue;
               end
               n_ok = n_ok + 1;
               si = stepinfo(T);
               b = NaN; try, b = bandwidth(T); catch, end %#ok<CTCH>
               dc = dcgain(T);
               cost = abs(log(max(b, realmin) / wc)) + (si.Overshoot / 100)^2 * 5 ...
                  + abs(1 - dc) * 3;
               if n_ok <= 20
                  list = [list sprintf('OK sgn %+d kp %11.5g kd %9.3g kf %4.1f | bw %9.4f rise %8.4f os %7.2f dc %7.5f\n', ...
                     sgn, kp, kd, kf, b, si.RiseTime, si.Overshoot, dc)]; %#ok<AGROW>
               end
               if cost < best.cost
                  best = struct('cost', cost, 'sgn', sgn, 'kp', kp, 'kd', kd, 'kf', kf, ...
                     'bw', b, 'rise', si.RiseTime, 'os', si.Overshoot, 'dc', dc);
               end
            end
         end
      end
   end
   fprintf(fid, 'stable gain sets found: %d\n%s', n_ok, list);
   fprintf(fid, 'BEST cost %.4f sgn %+d kp %.6g kd %.6g kf %.2f bw %.5f rise %.4f os %.2f dc %.5f\n', ...
      best.cost, best.sgn, best.kp, best.kd, best.kf, best.bw, best.rise, best.os, best.dc);
catch ME
   fprintf(fid, 'FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);
