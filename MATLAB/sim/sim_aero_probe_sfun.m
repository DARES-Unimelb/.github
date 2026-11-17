function sim_aero_probe_sfun(block)
% SIM_AERO_PROBE_SFUN Level-2 MATLAB S-Function: steady aerodynamic coefficients.
%
% Ports
%   in  1  7x1   [V alpha beta h de da dr]
%              V m/s, alpha and beta rad, h m (used for density), surface angles rad
%   out 1  13x1  [CL CD CY Cl Cm Cn L D Y M_roll M_pitch M_yaw rho]
%
% A thin wrapper around plant/flow_state.m and plant/aero_forces.m, the same two
% functions the flight models use, so the test page cannot report coefficients the
% aircraft does not fly. The state is built from the inputs with the rates and the
% attitude angles at zero, which is the steady case: p_hat, q_hat and r_hat all vanish
% and Cm_alphadot is evaluated with alpha_dot = q = 0, exactly as aero_forces
% documents. It runs at the continuous sample time and has no states, so it is direct
% feedthrough.
%
% P is read from the base workspace once per run and held in a persistent: calling
% evalin on every major step would dominate the run time of a page that is meant to
% respond while a slider is dragged.

setup(block);
end

function setup(block)
block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 7;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 13;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';
block.NumDialogPrms = 0;

block.RegBlockMethod('Start', @start);
block.RegBlockMethod('Outputs', @outputs);
end

function start(block) %#ok<INUSD>
clear sim_aero_probe_sfun_p
end

function outputs(block)
persistent P
if isempty(P)
   P = evalin('base', 'P');
end
s = block.InputPort(1).Data;
V = max(s(1), 1e-6);
alpha = s(2);
beta = s(3);
h = s(4);
de = s(5);
da = s(6);
dr = s(7);

ca = cos(alpha); cb = cos(beta);
x = [V * ca * cb; V * sin(beta); V * sin(alpha) * cb; ...
   0; 0; 0; 0; alpha; 0; 0; 0; -h];
u = [da; de; dr; 0; 0; 0];

% flow_state gives the density and the dynamic pressure for this altitude, so the test
% page follows the same atmosphere switch (P.atmo.model) the plant does.
[~, ~, ~, qbar, rho] = flow_state(P, x);
[~, d] = aero_forces(P, x, u, V, alpha, beta, qbar);

block.OutputPort(1).Data = [d.CL; d.CD; d.CY; d.Cl; d.Cm; d.Cn; ...
   d.L; d.D; d.Y; d.M_roll; d.M_pitch; d.M_yaw; rho];
end
