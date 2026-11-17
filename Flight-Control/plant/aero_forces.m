function [aero, dbg] = aero_forces(P, x, u, V, alpha, beta, qbar)
% AERO_FORCES Body-axis aerodynamic force and moment of the UACS twin.
%
% Inputs
%   P      parameter struct
%   x      12x1 state, u 6x1 control
%   V      true airspeed m/s, alpha/beta rad, qbar Pa, all from flow_state
% Outputs
%   aero.F_body  3x1 body force N, without gravity and without thrust
%   aero.M_body  3x1 body moment N*m, without the thrust moment
%   dbg          coefficients and intermediate values for reports and checks
%
%   Cm_alphadot is evaluated with alpha_dot = q, the level-flight identity

A = P.aero;
p = x(4);
q = x(5);
r = x(6);
da = u(1);
de = u(2);
dr = u(3);

V_safe = max(V, 1e-6);
q_hat = q * P.geom.cbar / (2 * V_safe);
p_hat = p * P.geom.b / (2 * V_safe);
r_hat = r * P.geom.b / (2 * V_safe);
alpha_zl = -A.CL0 / A.CL_alpha;   % zero-lift angle implied by the lift curve
alpha_dot = q;                    % v1 approximation

CL = A.CL0 + A.CL_alpha * alpha + A.CL_q * q_hat + A.CL_de * de;
CD = A.CD0 + A.k_induced * CL^2 + A.CD_alpha * (alpha - alpha_zl)^2 ...
   + A.CD_de * abs(de);
CY = A.CY_beta * beta + A.CY_dr * dr;
Cl = A.Cl_beta * beta + A.Cl_p * p_hat + A.Cl_r * r_hat + A.Cl_da * da ...
   + A.Cl_dr * dr;
Cm = A.Cm0 + A.Cm_alpha * alpha + A.Cm_q * q_hat + A.Cm_de * de ...
   + A.Cm_alphadot * (alpha_dot * P.geom.cbar / (2 * V_safe));
Cn = A.Cn_beta * beta + A.Cn_p * p_hat + A.Cn_r * r_hat + A.Cn_da * da ...
   + A.Cn_dr * dr;

D = qbar * P.geom.S * CD;
Y = qbar * P.geom.S * CY;
L = qbar * P.geom.S * CL;
M_roll = qbar * P.geom.S * P.geom.b * Cl;
M_pitch = qbar * P.geom.S * P.geom.cbar * Cm;
M_yaw = qbar * P.geom.S * P.geom.b * Cn;

% Wind axes to body axes, small-angle form
sa = sin(alpha); ca = cos(alpha);
sb = sin(beta);  cb = cos(beta);
Fx = L * sa - D * ca * cb - Y * ca * sb;
Fy = Y * cb - D * sb;
Fz = -L * ca - D * sa * cb - Y * sa * sb;

aero = struct('F_body', [Fx; Fy; Fz], 'M_body', [M_roll; M_pitch; M_yaw], ...
   'CL', CL, 'CD', CD, 'CY', CY, 'L', L, 'D', D, 'Y', Y);

if nargout > 1
   dbg = struct('CL', CL, 'CD', CD, 'CY', CY, 'Cl', Cl, 'Cm', Cm, 'Cn', Cn, ...
      'L', L, 'D', D, 'Y', Y, ...
      'M_roll', M_roll, 'M_pitch', M_pitch, 'M_yaw', M_yaw, ...
      'q_hat', q_hat, 'p_hat', p_hat, 'r_hat', r_hat, ...
      'alpha_zl', alpha_zl, 'alpha_dot', alpha_dot, ...
      'alphadot_law', 'alpha_dot = q (level-flight kinematic identity)', ...
      'Cm_q_eff', A.Cm_q + A.Cm_alphadot, ...
      'da', da, 'de', de, 'dr', dr);
end
end
