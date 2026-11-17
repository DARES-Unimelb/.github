function sim_aero_probe_sfun(block)
% SIM_AERO_PROBE_SFUN Level-2 MATLAB S-Function: steady aerodynamic coefficients
%
%   in  1  7x1   [V alpha beta h de da dr]
%              V m/s, alpha and beta rad, h m for density, surface angles rad
%   out 1  13x1  [CL CD CY Cl Cm Cn L D Y M_roll M_pitch M_yaw rho]
%
% Steady case: the rates and the attitude angles are zero, so p_hat, q_hat and
% r_hat vanish and Cm_alphadot uses alpha_dot = q = 0
% P is read from base once per run and then held in a persistent

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

% flow_state gives rho and qbar for this altitude, so the same atmosphere switch applies
[~, ~, ~, qbar, rho] = flow_state(P, x);
[~, d] = aero_forces(P, x, u, V, alpha, beta, qbar);

block.OutputPort(1).Data = [d.CL; d.CD; d.CY; d.Cl; d.Cm; d.Cn; ...
   d.L; d.D; d.Y; d.M_roll; d.M_pitch; d.M_yaw; rho];
end
