function [Tm, Tf] = thrust_moment(P, T_l, T_r)
% THRUST_MOMENT Thrust force and moment about the centre of gravity.
%
% Each motor pushes along +x_body from its station
% (P.prop.x_motor, +-P.prop.y_motor) with the moment arm measured from the
% CG. P.prop.x_motor and P.prop.y_motor are stated about the reference
% origin, so the CG is subtracted; the vertical arm is the tabulated
% P.prop.z_thrust_offset, which is already referenced to the CG.
%   Tm  3x1 moment, N*m, about the CG
%   Tf  3x1 body-axis thrust force, N, optional second output
% Counter-rotating propellers cancel the reaction torque, so no torque term
% is modelled in v1.
%
% Note: P.prop.z_thrust_offset (-0.28 m) and P.prop.z_motor - P.mass.z_cg
% (-0.1185 m) disagree; the tabulated offset is used here. See
% docs/design_notes_plant.md, section on thrust line.

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
