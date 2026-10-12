function sim_meas_sfun(block)
% SIM_MEAS_SFUN Level-2 MATLAB S-Function: air data of the state.
%
% Ports
%   in  1  x_state 12x1
%   out 1  air     4x1  [V alpha beta qbar]
%
% Continuous and direct-feedthrough: the block is algebraic, and the discrete
% controller downstream does the sampling. Calling plant/flow_state.m keeps the
% Controller's idea of V, alpha and beta identical to the plant's.

setup(block);
end

function setup(block)
block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 12;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 4;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';
block.NumDialogPrms = 0;

block.RegBlockMethod('Outputs', @outputs);
end

function outputs(block)
x = block.InputPort(1).Data;
[V, alpha, beta, qbar] = sim_meas_wrap([], x);
block.OutputPort(1).Data = [V; alpha; beta; qbar];
end
