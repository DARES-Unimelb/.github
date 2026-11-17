% diag_q3 Scan the q rate loop over PI, PID and PD structures, positive polarity.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_q3.log');
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
   kinds = {'PI', 'PID', 'PD'};
   for ki_ = 1:numel(kinds)
      kind = kinds{ki_};
      best = struct('cost', Inf);
      n_ok = 0;
      for kp = logspace(log10(1e-2 * Kp_nom), log10(1e2 * Kp_nom), 60)
         for f = [0, 0.05, 0.2, 0.6, 2.0]
            for kd = [0, 1e-4, 1e-3]
               switch kind
                  case 'PI',  cn = [kp, kp * f * wc]; cd = [1, 0];
                  case 'PID', cn = [kp * Tf + kd, kp + kp * f * wc * Tf, kp * f * wc]; cd = [Tf, 1, 0];
                  case 'PD',  cn = [kp * Tf + kd, kp]; cd = [Tf, 1];
               end
               T = feedback(tf(cn, cd) * Gq, 1);
               p = pole(T);
               if max(real(p)) >= -1e-7
                  continue;
               end
               dc = dcgain(T);
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
               n_ok = n_ok + 1;
               cost = abs(1 - dc) * 10 + abs(log(max(b, realmin) / wc)) ...
                  + (isnan(rise) * 2) + (os / 100)^2 * 5;
               if cost < best.cost
                  best = struct('cost', cost, 'kp', kp, 'f', f, 'kd', kd, 'bw', b, ...
                     'rise', rise, 'os', os, 'dc', dc);
               end
            end
         end
      end
      fprintf(fid, '%s: %d stable sets. BEST kp %.6g f %.3g kd %.3g | bw %10.4f rise %8.4f os %8.2f dc %8.5f cost %.4f\n', ...
         kind, n_ok, best.kp, best.f, best.kd, best.bw, best.rise, best.os, best.dc, best.cost);
   end
catch ME
   fprintf(fid, 'FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);
