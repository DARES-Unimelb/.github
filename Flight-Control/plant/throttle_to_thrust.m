function T = throttle_to_thrust(P, dth, V)
% THROTTLE_TO_THRUST Per-motor thrust from throttle setting and speed.
%
%   dth  0..1, V in m/s, T in N per motor

T = prop_thrust_from_throttle(P, dth, V);
end
