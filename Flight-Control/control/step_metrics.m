function m = step_metrics(t, y, t_step, target)
% STEP_METRICS Rise time, overshoot, settling time and steady-state error.
%
%   m = step_metrics(t, y, t_step, target)
%   t       time vector, s
%   y       response vector, same length as t
%   t_step  instant of the step, s
%   target  commanded value, for the steady-state error
%
% Reports 10-90 percent rise time, overshoot against the settled value, the
% last time the response is outside 2 percent of the settled value, and the
% steady-state error against the command.

t = t(:);
y = y(:);
i0 = find(t <= t_step, 1, 'last');
if isempty(i0)
   i0 = 1;
end
y0 = y(i0);
n = numel(y);
i_ss = max(n - round(0.05 * n), i0 + 1);
ss = mean(y(i_ss:n));
dy = ss - y0;

m = struct('y0', y0, 'y_ss', ss, 'target', target, 'delta', dy, ...
   'rise_time_s', NaN, 'overshoot_pct', NaN, 'settling_time_s', NaN, ...
   'peak', NaN, 'error_ss', target - ss);
if abs(dy) < 1e-12
   return;
end

seg = y(i0:n);
tsg = t(i0:n);
lo = y0 + 0.1 * dy;
hi = y0 + 0.9 * dy;
i_lo = find((seg - lo) * sign(dy) >= 0, 1);
i_hi = find((seg - hi) * sign(dy) >= 0, 1);
if ~isempty(i_lo) && ~isempty(i_hi);
   m.rise_time_s = tsg(i_hi) - tsg(i_lo);
end
if dy > 0
   m.peak = max(seg);
   m.overshoot_pct = max(0, (m.peak - ss) / abs(dy) * 100);
else
   m.peak = min(seg);
   m.overshoot_pct = max(0, (ss - m.peak) / abs(dy) * 100);
end
out = find(abs(seg - ss) > 0.02 * abs(dy), 1, 'last');
if isempty(out)
   m.settling_time_s = 0;
else
   m.settling_time_s = tsg(out) - t_step;
end
end
