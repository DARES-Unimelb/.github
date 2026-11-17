function [Tm, Tf] = thrust_moment(P, T_l, T_r)
% THRUST_MOMENT Thrust force and moment about the centre of gravity.
%
%   Tm  3x1 moment, N*m, about the CG
%   Tf  3x1 body-axis thrust force, N, optional second output
%
%   The vertical arm is P.prop.z_thrust_offset, not P.prop.z_motor - P.mass.z_cg

x_arm = P.prop.x_motor - P.mass.x_cg;
y_arm = P.prop.y_motor - P.mass.y_cg;
z_arm = P.prop.z_thrust_offset;

r_l = [x_arm;  y_arm; z_arm];
r_r = [x_arm; -y_arm; z_arm];
F_l = [T_l; 0; 0];
F_r = [T_r; 0; 0];

Tm = cross(r_l, F_l) + cross(r_r, F_r);
if nargout > 1
   Tf = F_l + F_r;
end
end
