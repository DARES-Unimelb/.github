function dth = sim_dth_for_thrust(P, V, T_per_motor)
% SIM_DTH_FOR_THRUST Throttle that produces a wanted per-motor thrust
%
%   dth = sim_dth_for_thrust(P, V, T_per_motor)
%
% Bisection on sim_thrust, which increases monotonically with throttle

lo = P.prop.esc.min_throttle;
hi = P.prop.esc.max_throttle;
T_hi = sim_thrust(P, hi, V);
if T_per_motor <= sim_thrust(P, lo, V)
   dth = lo;
   return;
end
if T_per_motor >= T_hi
   dth = hi;
   return;
end
for iter = 1:60
   mid = 0.5 * (lo + hi);
   if sim_thrust(P, mid, V) < T_per_motor
      lo = mid;
   else
      hi = mid;
   end
end
dth = 0.5 * (lo + hi);
end
