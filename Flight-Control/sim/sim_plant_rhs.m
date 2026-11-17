function [dx, src] = sim_plant_rhs(t, x, u, wind_ned, P)
% SIM_PLANT_RHS Plant right-hand side with a steady wind, 12 states
%
%   t         time, s
%   x         12x1 state, x(1:3) is GROUND velocity and x(10:12) is position
%   u         6x1 [da de dr dth_l dth_r dT]
%   wind_ned  3x1 steady wind in NED, m/s, default zero
%   P         parameter struct, default from the base workspace
%   dx        12x1 state derivative
%   src       'eom_6dof' or 'placeholder', which plant produced dx
%
% Wind shifts the aerodynamics to the air-relative velocity v_g - R*W, so
% dx(1:3) is d(v_air)/dt while dx(10:12) uses the ground velocity

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
% convert d(v_air)/dt to d(v_g)/dt, then keep the ground velocity in the position
% kinematics
dx(1:3) = dx(1:3) - cross(x(4:6), Wb);
dx(10:12) = R' * x(1:3);
end
