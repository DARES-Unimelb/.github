function [dx, src] = sim_plant_rhs(t, x, u, wind_ned, P)
% SIM_PLANT_RHS Plant right-hand side with a steady wind, 12 states.
%
%   dx = sim_plant_rhs(t, x, u, wind_ned, P)
%
% Wind enters as an air-relative velocity shift. Let W be constant in NED, R the
% body-from-NED matrix and omega = x(4:6). The integrated state is the GROUND
% velocity, so the plant is handed the air-relative velocity
%   v_air = v_g - R*W.
% The two velocities do not have the same body-frame derivative: with W constant
% in NED, d(R*W)/dt in body axes is -omega x (R*W), so
%   d(v_g)/dt = d(v_air)/dt - omega x (R*W).
% The plant returns d(v_air)/dt, therefore omega x (R*W) is subtracted below.
% That term is exactly zero when the wind is zero, so a no-wind run is unchanged
% bit for bit, and it is what makes a constant wind a pure translation of the
% still-air trajectory instead of a spurious rotation rate.
%
% The position rate keeps the GROUND velocity, dx(10:12) = C_bn*v_g: the wind
% carries the aircraft over the ground without changing its airspeed, so the wind
% effect on the trajectory is already inside the aerodynamics and must not be
% counted a second time here.
%
% plant/eom_6dof.m carries no wind argument and reads no wind field from P, so
% the shift happens here, once, at the model boundary.
%
% src is 'eom_6dof' or 'placeholder', so every result file records which plant
% produced it.

if nargin < 5 || isempty(P)
   P = evalin('base', 'P');
end
if nargin < 4 || isempty(wind_ned)
   wind_ned = zeros(3, 1);
end
x = x(:);
u = u(:);
wind_ned = wind_ned(:);

R = sim_dcm(x(7:9));
Wb = R * wind_ned;      % wind in body axes, m/s
x_air = x;
x_air(1:3) = x(1:3) - Wb;

mode = sim_plant_source();
use_real = strcmp(mode, 'eom_6dof') || (strcmp(mode, 'auto') && sim_have('eom_6dof'));
if use_real
   dx = eom_6dof(t, x_air, u, P);
   src = 'eom_6dof';
else
   dx = sim_plant_rhs_ph(t, x_air, u, P);
   src = 'placeholder';
end

dx = dx(:);
% Remove the frame term between the air-relative and the ground velocity, then
% keep the ground velocity in the position kinematics.
dx(1:3) = dx(1:3) - cross(x(4:6), Wb);
dx(10:12) = R' * x(1:3);
end
