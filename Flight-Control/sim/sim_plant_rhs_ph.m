function dx = sim_plant_rhs_ph(t, x, u, P)
% SIM_PLANT_RHS_PH Placeholder 12-state plant, used only when plant/ is absent
%
%   t, x, u, P   as sim_plant_rhs, with the wind shift already applied
%   dx           12x1 state derivative
%
% Same physics as plant/eom_6dof.m except for the missing alpha-dot (downwash
% lag) term, the CL_q and Cm_q rate terms beyond q, and the gust shaping

[V, alpha, beta, qbar] = sim_meas_wrap(P, x);
aero = P.aero;
S = P.geom.S;
b = P.geom.b;
cbar = P.geom.cbar;

de = u(2);
da = u(1);
dr = u(3);
V_safe = max(V, 1e-6);
q_hat = x(5) * cbar / (2 * V_safe);
p_hat = x(4) * b / (2 * V_safe);
r_hat = x(6) * b / (2 * V_safe);

CL = aero.CL0 + aero.CL_alpha * alpha + aero.CL_de * de + aero.CL_q * q_hat;
CD = aero.CD0 + aero.k_induced * CL^2 + aero.CD_de * de;
CY = aero.CY_beta * beta + aero.CY_dr * dr;
Cl = aero.Cl_beta * beta + aero.Cl_p * p_hat + aero.Cl_r * r_hat ...
   + aero.Cl_da * da + aero.Cl_dr * dr;
Cm = aero.Cm0 + aero.Cm_alpha * alpha + aero.Cm_de * de + aero.Cm_q * q_hat;
Cn = aero.Cn_beta * beta + aero.Cn_p * p_hat + aero.Cn_r * r_hat ...
   + aero.Cn_da * da + aero.Cn_dr * dr;

F_aero = qbar * S * [-CD * cos(alpha) + CL * sin(alpha); ...
                     CY; ...
                    -CD * sin(alpha) - CL * cos(alpha)];
M_aero = qbar * S * [b * Cl; cbar * Cm; b * Cn];

% two throttles plus the additive symmetric thrust command
T_l = sim_thrust(P, u(4), V) + 0.5 * u(6);
T_r = sim_thrust(P, u(5), V) + 0.5 * u(6);
T_l = max(T_l, 0);
T_r = max(T_r, 0);
if sim_have('thrust_moment')
   [Tm, Tf] = thrust_moment(P, T_l, T_r);
else
   x_arm = P.prop.x_motor - P.mass.x_cg;
   y_arm = P.prop.y_motor - P.mass.y_cg;
   z_arm = P.prop.z_thrust_offset;
   Tm = cross([x_arm; y_arm; z_arm], [T_l; 0; 0]) ...
      + cross([x_arm; -y_arm; z_arm], [T_r; 0; 0]);
   Tf = [T_l + T_r; 0; 0];
end

phi = x(7);
theta = x(8);
om = x(4:6);
Vb = x(1:3);
g_body = P.mass.m * P.env.g * [-sin(theta); sin(phi) * cos(theta); cos(phi) * cos(theta)];
F_body = F_aero + Tf + g_body;

I_mat = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; ...
         0, P.mass.Iyy_cg, 0; ...
         -P.mass.Ixz_cg, 0, P.mass.Izz_cg];
d_uvw = F_body / P.mass.m - cross(om, Vb);
d_pqr = I_mat \ (M_aero + Tm - cross(om, I_mat * om));
d_eul = [x(4) + tan(theta) * (x(5) * sin(phi) + x(6) * cos(phi)); ...
         x(5) * cos(phi) - x(6) * sin(phi); ...
         (x(5) * sin(phi) + x(6) * cos(phi)) / cos(theta)];
d_pos = (sim_dcm(x(7:9)))' * Vb;

dx = [d_uvw; d_pqr; d_eul; d_pos];
end
