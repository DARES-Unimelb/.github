function [dx, F_body, M_body] = eom_6dof(~, x, u, P)
% EOM_6DOF Twelve-state rigid-body equations of motion.
%
%   x = [u v w | p q r | phi theta psi | pn pe pd]'
%   u = [da de dr dth_l dth_r dT]'
%   dx = eom_6dof(t, x, u, P)          for ode45 and Simulink blocks
%   [dx, F_body, M_body] = eom_6dof(...) also returns the total body force
%   and moment used, which is what the trim residual is built from.
%
% Gravity is applied here, thrust is applied here, aerodynamics comes from
% aero_forces. The inertia matrix keeps the Ixz cross term.
% u(6) = dT is an additive symmetric thrust deviation in N, half to each motor,
% used by the LQR and energy loops; u(4) and u(5) are the throttles, the actuator
% path used by the PID and allocation path. The two paths agree to first order:
% dT maps to a throttle pair through the local slope of the authoritative
% propulsion map, and the modes report checks that slope against B(:,4), B(:,5).

[V, alpha, beta, qbar] = flow_state(P, x);
aero = aero_forces(P, x, u, V, alpha, beta, qbar);

% Propulsion: two throttles plus half the additive thrust command each.
T_l = throttle_to_thrust(P, u(4), V) + 0.5 * u(6);
T_r = throttle_to_thrust(P, u(5), V) + 0.5 * u(6);
T_l = max(T_l, 0);
T_r = max(T_r, 0);
[Tm, Tf] = thrust_moment(P, T_l, T_r);

p = x(4); q = x(5); r = x(6);
phi = x(7); theta = x(8); psi = x(9);
Vb = x(1:3);
om = [p; q; r];

% Gravity resolved into body axes.
g_body = P.mass.m * P.env.g * [-sin(theta); sin(phi) * cos(theta); cos(phi) * cos(theta)];

F_body = aero.F_body + Tf + g_body;
M_body = aero.M_body + Tm;

% Newton-Euler in body axes.
I_mat = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; ...
         0, P.mass.Iyy_cg, 0; ...
         -P.mass.Ixz_cg, 0, P.mass.Izz_cg];
d_uvw = F_body / P.mass.m - cross(om, Vb);
d_pqr = I_mat \ (M_body - cross(om, I_mat * om));

% Euler angle kinematics.
d_eul = [p + tan(theta) * (q * sin(phi) + r * cos(phi)); ...
         q * cos(phi) - r * sin(phi); ...
         (q * sin(phi) + r * cos(phi)) / cos(theta)];

% Body to NED direction cosine matrix, for the position states.
ct = cos(theta); st = sin(theta);
cp = cos(phi);   sp = sin(phi);
cs = cos(psi);   ss = sin(psi);
R_bn = [ct * cs, sp * st * cs - cp * ss, cp * st * cs + sp * ss; ...
        ct * ss, sp * st * ss + cp * cs, cp * st * ss - sp * cs; ...
        -st,     sp * ct,                cp * ct];
d_pos = R_bn * Vb;

dx = [d_uvw; d_pqr; d_eul; d_pos];
end
