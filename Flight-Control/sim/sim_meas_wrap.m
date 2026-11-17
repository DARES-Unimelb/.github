function [V, alpha, beta, qbar] = sim_meas_wrap(P, x)
% SIM_MEAS_WRAP Air data of the state, from plant/flow_state.m when present
%
%   P                        parameter struct, default P from the base workspace
%   x                        12x1 state
%   V, alpha, beta, qbar     airspeed m/s, angle of attack rad, sideslip rad,
%                            dynamic pressure Pa
%
% The local branch uses the same definitions as flow_state: V the norm of the body
% velocity, alpha = atan2(w, u), beta = asin(v/V), qbar = 0.5*rho*V^2 with the
% constant density P.env.rho_sl

if nargin < 1 || isempty(P)
   P = evalin('base', 'P');
end
x = x(:);
if sim_have('flow_state')
   [V, alpha, beta, qbar] = flow_state(P, x);
   return;
end
u = x(1);
v = x(2);
w = x(3);
V = sqrt(u^2 + v^2 + w^2);
V_safe = max(V, 1e-6);
alpha = atan2(w, u);
beta = asin(max(-1, min(1, v / V_safe)));
qbar = 0.5 * P.env.rho_sl * V^2;
end
