function s = sim_state_scale(P)
% SIM_STATE_SCALE Reference scale of each of the 12 states.
%
% Used to turn a trajectory difference into a dimensionless number when the
% trajectory's own excursion is not a usable denominator, for instance for a
% state that never moves. The scales are the design point quantities the states
% are compared against elsewhere in the project:
%   u, v, w   cruise airspeed
%   p, q, r   1 rad/s, the unit of the rate command
%   phi, theta, psi  1 rad
%   pn, pe, pd  cruise altitude

V = P.op.cruise.V_mps;
h = P.op.cruise.h_m;
s = [V; V; V; 1; 1; 1; 1; 1; 1; h; h; h];
end
