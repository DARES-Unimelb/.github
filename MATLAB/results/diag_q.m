% diag_q Scan the q rate loop for any stable PI or PID gain set.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_q.log');
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
   fprintf(fid, 'Gq poles %s\nGq zeros %s\nGq dcgain %.6g, B_long(:,1) %s\n', ...
      mat2str(sort(pole(Gq)).', 5), mat2str(sort(zero(Gq)).', 6), dcgain(Gq), ...
      mat2str(lc.B_long(:, 1).', 5));
   Kp_nom = 1 / abs(freqresp(Gq, 1i * wc));
   fprintf(fid, 'wc %.3f Kp_nom %.6g\n', wc, Kp_nom);
   n_ok = 0;
   for sgn = [1, -1]
      for kp = logspace(log10(1e-3 * Kp_nom), log10(1e3 * Kp_nom), 200)
         for frac = [0, 0.01, 0.05, 0.2, 1.0]
            for kd = [0, 1e-4, 1e-3]
               ki = kp * frac * wc;
               cnum = sgn * [kp * Tf + kd, kp + ki * Tf, ki];
               cden = [Tf, 1, 0];
               T = feedback(tf(cnum, cden) * Gq, 1);
               p = pole(T);
               if max(real(p)) < -1e-7
                  n_ok = n_ok + 1;
                  if n_ok <= 25
                     b = NaN; try, b = bandwidth(T); catch, end %#ok<CTCH>
                     si = stepinfo(T);
                     fprintf(fid, 'OK sgn %+d kp %10.6g ki %10.6g kd %9.3g | worst %+9.3e bw %8.4f rise %8.4f os %7.2f\n', ...
                        sgn, kp, ki, kd, max(real(p)), b, si.RiseTime, si.Overshoot);
                  end
               end
            end
         end
      end
   end
   fprintf(fid, 'stable gain sets found: %d\n', n_ok);
catch ME
   fprintf(fid, 'FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);
