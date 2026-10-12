function sim_plant_ref_aero(block)
% SIM_PLANT_REF_AERO Level-2 MATLAB S-Function: total body force and moment of the twin.
%
% The aerodynamics, the propulsion map, the thrust moment and gravity are the
% same functions plant/eom_6dof.m calls, so the only difference between this and
% the hand written plant is that the air density arrives on an input port instead
% of being read as the constant P.env.rho_sl. That lets the Aerospace Blockset
% Atmosphere (ISA) block drive it, and it keeps the aerodynamic model identical
% between the two 6DOF models under test. At rho = P.env.rho_sl the outputs equal
% eom_6dof's F_body and M_body to the last bit; sim_ref_calibrate measures it.
%
% Ports
%   in  1  Vb       3x1  body velocity over ground, m/s
%   in  2  om       3x1  body rates [p q r], rad/s
%   in  3  eul      3x1  [phi theta psi], rad
%   in  4  wind_ned 3x1  steady wind, NED, m/s
%   in  5  u_act    6x1  [da de dr dth_l dth_r dT]
%   in  6  rho      1x1  air density, kg/m^3
%   out 1  F_body   3x1  total body force: aero + thrust + gravity, N
%   out 2  M_body   3x1  total body moment: aero + thrust moment, N*m
%   out 3  aux      11x1 [rho; V; alpha; beta; qbar; F_body; M_body]
%
% Wind enters as the air-relative velocity shift sim_plant_rhs applies, so the
% two models treat wind the same way.

setup(block);
end

%% ---------------- block interface ----------------
function setup(block)
block.NumInputPorts = 6;
dims = [3, 3, 3, 3, 6, 1];
for k = 1:6
   block.InputPort(k).Dimensions = dims(k);
   block.InputPort(k).DatatypeID = 0;
   block.InputPort(k).Complexity = 'Real';
   block.InputPort(k).SamplingMode = 'Sample';
   % The force depends algebraically on the states the 6DOF block publishes, and
   % the 6DOF block's own outputs are integrator states, so this feedthrough does
   % not close a loop. Declaring it false would let Simulink silently hold the
   % previous sample instead, which is a wrong answer that still runs.
   block.InputPort(k).DirectFeedthrough = true;
end

out_dims = [3, 3, 11];
block.NumOutputPorts = 3;
for k = 1:3
   block.OutputPort(k).Dimensions = out_dims(k);
   block.OutputPort(k).DatatypeID = 0;
   block.OutputPort(k).Complexity = 'Real';
   block.OutputPort(k).SamplingMode = 'Sample';
end

block.NumContStates = 0;
block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';
block.NumDialogPrms = 0;

block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Terminate', @terminate);
end

%% ---------------- methods ----------------
function outputs(block)
Vb = block.InputPort(1).Data(:);
om = block.InputPort(2).Data(:);
eul = block.InputPort(3).Data(:);
wind_ned = block.InputPort(4).Data(:);
u = block.InputPort(5).Data(:);
rho = block.InputPort(6).Data(1);

P = evalin('base', 'P');

% Air-relative body velocity, exactly as sim_plant_rhs derives it.
R = sim_dcm(eul);
x = [Vb - R * wind_ned; om; eul; zeros(3, 1)];
[V, alpha, beta, ~] = flow_state(P, x);
qbar = 0.5 * rho * V^2;

aero = aero_forces(P, x, u, V, alpha, beta, qbar);
phi = eul(1);
theta = eul(2);
T_l = max(throttle_to_thrust(P, u(4), V) + 0.5 * u(6), 0);
T_r = max(throttle_to_thrust(P, u(5), V) + 0.5 * u(6), 0);
[Tm, Tf] = thrust_moment(P, T_l, T_r);

% Gravity resolved into body axes, as in eom_6dof.
g_body = P.mass.m * P.env.g * [-sin(theta); sin(phi) * cos(theta); cos(phi) * cos(theta)];

F_body = aero.F_body + Tf + g_body;
M_body = aero.M_body + Tm;

block.OutputPort(1).Data = F_body;
block.OutputPort(2).Data = M_body;
block.OutputPort(3).Data = [rho; V; alpha; beta; qbar; F_body; M_body];
end

function terminate(~)
end
