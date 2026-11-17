function sim_plant_sfun(block)
% SIM_PLANT_SFUN Level-2 MATLAB S-Function: the 12-state nonlinear plant
%
%   in  1  u_act    6x1  [da de dr dth_l dth_r dT]
%   in  2  wind_ned 3x1  steady wind, NED, m/s
%   out 1  x_state  12x1 plant state
%
% Continuous states, so the block serves both the fixed-step ode4 main line and
% the variable-step ode45 cross-check

setup(block);
end

%% Block interface
function setup(block)
block.NumInputPorts = 2;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 6;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).DirectFeedthrough = false;

block.InputPort(2).Dimensions = 3;
block.InputPort(2).DatatypeID = 0;
block.InputPort(2).Complexity = 'Real';
block.InputPort(2).SamplingMode = 'Sample';
block.InputPort(2).DirectFeedthrough = false;

block.OutputPort(1).Dimensions = 12;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.NumContStates = 12;
block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';
block.NumDialogPrms = 0;

block.RegBlockMethod('InitializeConditions', @initialize_conditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Derivatives', @derivatives);
block.RegBlockMethod('Terminate', @terminate);
end

function initialize_conditions(block)
block.ContStates.Data = sim_plant_x0();
end

function outputs(block)
block.OutputPort(1).Data = block.ContStates.Data;
end

function derivatives(block)
x = block.ContStates.Data;
u = block.InputPort(1).Data;
wind = block.InputPort(2).Data;
dx = sim_plant_rhs(block.CurrentTime, x, u, wind);
block.Derivatives.Data = dx;
end

function terminate(~)
end
