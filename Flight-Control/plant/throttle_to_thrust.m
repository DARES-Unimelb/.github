function T = throttle_to_thrust(P, dth, V)
% THROTTLE_TO_THRUST Per-motor thrust from throttle setting and speed.
%
% Thin wrapper over the single authoritative propulsion map in
% params/prop_thrust_from_throttle.m (architecture.md 5.0). Throttle is the
% fraction of peak shaft power, so thrust grows about as throttle^(4/9); the
% plant, the trim solve and the control allocation all come through here so
% they cannot disagree about how throttle becomes thrust.
%
%   dth  0..1, V in m/s, T in N per motor

T = prop_thrust_from_throttle(P, dth, V);
end
