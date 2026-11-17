% diag_rate_sweep Rate-loop gain versus the time-domain response V6 measures.
% The closed-loop poles of the rate loop alone are not the right arbiter: the
% acceptance case drives the whole cascade. This sweeps the q gain against the
% theta step response, which is what V6's pitch_step scores.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_rate_sweep.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * tf(1, [P.act.elevator.tau_s, 1]);
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * tf(1, [P.act.aileron.tau_s, 1]);
fprintf(fid, '=== q gain vs the inner loop and vs the theta step ===\n');
fprintf(fid, '%-8s %-14s %-10s %-10s %-10s\n', 'Kq', 'rate maxRe', 'th maxRe', 'th rise', 'th os');
for Kq = [0.05 0.10 0.15 0.20 0.25 0.30 0.40 0.50 0.60]
   Tq = minreal(feedback(Kq * Gq, 1), 1e-9);
   pq = max(real(pole(Tq)));
   Gth = local_series_integrator(Tq);
   best = [];
   for Kth = logspace(-3, 0.5, 60)
      Tth = minreal(feedback(Kth * Gth, 1), 1e-9);
      p = pole(Tth);
      if max(real(p)) >= -1e-6
         continue;
      end
      si = stepinfo(Tth);
      if isnan(si.RiseTime), continue; end
      if isempty(best) || si.Overshoot < best.os
         best = struct('Kth', Kth, 'worst', max(real(p)), 'rise', si.RiseTime, 'os', si.Overshoot);
      end
   end
   if isempty(best)
      fprintf(fid, '%-8.3f %-14.4e %s\n', Kq, pq, 'no stable theta gain');
   else
      fprintf(fid, '%-8.3f %-14.4e %-10.4e %-10.4f %-10.2f (Kth %.4f)\n', Kq, pq, ...
         best.worst, best.rise, best.os, best.Kth);
   end
end
fprintf(fid, '\n=== p gain vs the roll attitude step ===\n');
fprintf(fid, '%-8s %-14s %-10s %-10s %-10s\n', 'Kp', 'rate maxRe', 'phi maxRe', 'phi rise', 'phi os');
for Kp = [0.05 0.1 0.2 0.4 0.6 0.77 1.0 1.5]
   Tp = minreal(feedback(Kp * Gp, 1), 1e-9);
   pp = max(real(pole(Tp)));
   Gphi = local_series_integrator(Tp);
   best = [];
   for Kph = logspace(-2, 2, 80)
      Tph = minreal(feedback(Kph * Gphi, 1), 1e-9);
      p = pole(Tph);
      if max(real(p)) >= -1e-6, continue; end
      si = stepinfo(Tph);
      if isnan(si.RiseTime), continue; end
      if isempty(best) || si.Overshoot < best.os
         best = struct('Kph', Kph, 'worst', max(real(p)), 'rise', si.RiseTime, 'os', si.Overshoot);
      end
   end
   if isempty(best)
      fprintf(fid, '%-8.3f %-14.4e %s\n', Kp, pp, 'no stable phi gain');
   else
      fprintf(fid, '%-8.3f %-14.4e %-10.4e %-10.4f %-10.2f (Kph %.4f)\n', Kp, pp, ...
         best.worst, best.rise, best.os, best.Kph);
   end
end
fclose(fid);
type(out);

function G = local_series_integrator(T)
[Ad, Bd, Cd, Dd] = ssdata(T);
n = size(Ad, 1);
G = ss([Ad, zeros(n, 1); Cd, 0], [Bd; Dd], [zeros(1, n), 1], 0);
end
