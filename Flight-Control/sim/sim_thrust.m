function T = sim_thrust(P, dth, V)
% SIM_THRUST Per-motor thrust, from the authoritative momentum-theory map.
%
% architecture.md 5.0 makes params/prop_thrust_from_throttle.m the single thrust
% entry point and retires P.prop.k_ram. plant/throttle_to_thrust.m is only a
% fallback for the window where it has not been re-pointed yet, and it is used
% with a warning so a stale duplicate model cannot pass unnoticed.

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
