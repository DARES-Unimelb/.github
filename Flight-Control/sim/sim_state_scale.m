function s = sim_state_scale(P)
% SIM_STATE_SCALE 12x1 reference magnitude per state, for scaling residuals
%
%   P   parameter struct, reads P.op.cruise.V_mps and P.op.cruise.h_m
%   s   per state: u, v, w  cruise airspeed; p, q, r  1 rad/s;
%       phi, theta, psi  1 rad; pn, pe, pd  cruise altitude

V = P.op.cruise.V_mps;
h = P.op.cruise.h_m;
s = [V; V; V; 1; 1; 1; 1; 1; 1; h; h; h];
end
