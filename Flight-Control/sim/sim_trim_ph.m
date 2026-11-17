function trim = sim_trim_ph(P)
% SIM_TRIM_PH Closed-form cruise trim on the placeholder aerodynamics
%
%   P      parameter struct
%   trim   x_star, u_star, residual, and trim.source = 'placeholder'; the
%          lateral channel stays at zero bank, zero sideslip and zero rudder

op = P.op.cruise;
V = op.V_mps;
rho = P.env.rho_sl;
S = P.geom.S;
cbar = P.geom.cbar;
qbar = 0.5 * rho * V^2;

CL_req = P.mass.m * P.env.g / (qbar * S);
alpha = (CL_req - P.aero.CL0) / P.aero.CL_alpha;
CD = P.aero.CD0 + P.aero.k_induced * CL_req^2;
T_total = qbar * S * CD;
z_arm = P.prop.z_thrust_offset;
M_aero_req = -z_arm * T_total;
de = (M_aero_req / (qbar * S * cbar) - P.aero.Cm0 - P.aero.Cm_alpha * alpha) / P.aero.Cm_de;

dth = sim_dth_for_thrust(P, V, 0.5 * T_total);
theta = alpha;
u = V * cos(alpha);
w = V * sin(alpha);
x_star = [u; 0; w; 0; 0; 0; 0; theta; 0; 0; 0; -op.h_m];
u_star = [0; de; 0; dth; dth; 0];

res = eom_6dof_ph_residual(P, x_star, u_star);
trim = struct();
trim.x_star = x_star;
trim.u_star = u_star;
trim.residual = res;
trim.exitflag = 1;
trim.alpha_deg = rad2deg(alpha);
trim.theta_deg = rad2deg(theta);
trim.de_deg = rad2deg(de);
trim.throttle = dth;
trim.T_total = T_total;
trim.source = 'placeholder';
trim.note = 'closed-form trim on the placeholder aerodynamics, no optimiser';
end

function res = eom_6dof_ph_residual(P, x, u)
% six force and moment equations of the placeholder plant at the trim point
dx = sim_plant_rhs_ph(0, x, u, P);
res = [dx(1:3); dx(4:6)];
end
