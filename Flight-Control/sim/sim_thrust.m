function T = sim_thrust(P, dth, V)
% SIM_THRUST Per-motor thrust in N from throttle and airspeed
%
%   P     parameter struct
%   dth   throttle per motor, scalar or vector, 0 to 1
%   V     airspeed in m/s
%   T     thrust per motor in N

if sim_have('prop_thrust_from_throttle')
   T = prop_thrust_from_throttle(P, dth, V);
   return;
end
if sim_have('throttle_to_thrust')
   T = throttle_to_thrust(P, dth, V);
   return;
end
error('sim_thrust:NoModel', ...
   'neither prop_thrust_from_throttle nor throttle_to_thrust is on the path');
end
