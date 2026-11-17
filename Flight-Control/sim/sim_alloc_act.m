function [da, de, dr, dth_l, dth_r] = sim_alloc_act(P, M_cmd, T_cmd)
% SIM_ALLOC_ACT Five actuator commands from a moment and a thrust request
%
%   [da, de, dr, dth_l, dth_r] = sim_alloc_act(P, M_cmd, T_cmd)
%
% Calls allocate when it is on the path, otherwise a pseudo-inverse fallback
% P defaults to the base workspace variable P

if nargin < 1 || isempty(P)
   P = evalin('base', 'P');
end
if sim_have('allocate')
   [da, de, dr, dth_l, dth_r] = allocate(P, M_cmd, T_cmd);
   return;
end

V_a = P.op.cruise.V_mps;
qbar_a = 0.5 * P.env.rho_sl * V_a^2;
Sb = qbar_a * P.geom.S * P.geom.b;
Sc = qbar_a * P.geom.S * P.geom.cbar;
% effectiveness matrix for the three aerodynamic surfaces
G = [Sb * P.aero.Cl_da, 0, Sb * P.aero.Cl_dr; ...
     0, Sc * P.aero.Cm_de, 0; ...
     Sb * P.aero.Cn_da, 0, Sb * P.aero.Cn_dr];
v = pinv(G) * M_cmd(:);

da_lim = deg2rad(P.act.aileron.max_deg);
de_lim = deg2rad(P.act.elevator.max_deg);
dr_lim = deg2rad(P.act.rudder.max_deg);
da = min(max(v(1), -da_lim), da_lim);
de = min(max(v(2), -de_lim), de_lim);
dr = min(max(v(3), -dr_lim), dr_lim);

dth = sim_dth_for_thrust(P, V_a, 0.5 * T_cmd);
dth_l = dth;
dth_r = dth;
end
