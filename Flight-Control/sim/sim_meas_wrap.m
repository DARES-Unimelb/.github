function [V, alpha, beta, qbar] = sim_meas_wrap(P, x)
% SIM_MEAS_WRAP Air data of the state, from plant/flow_state.m when present.
%
% The controller models and the placeholder plant both need air data, so the
% dependency is resolved in one place. The local branch is the same definition
% as flow_state: V the norm of the body velocity, alpha = atan2(w, u),
% beta = asin(v/V), qbar = 0.5*rho*V^2 with the constant density of P.env.

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
