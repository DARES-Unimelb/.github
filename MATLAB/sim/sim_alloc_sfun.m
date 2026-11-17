function sim_alloc_sfun(block)
% SIM_ALLOC_SFUN Level-2 MATLAB S-Function: control allocation.
%
% Ports
%   in  1  M_cmd 3x1  desired body aerodynamic moment, N*m
%   in  2  T_cmd 1x1  desired total thrust, N
%   out 1  act   5x1  [da de dr dth_l dth_r]
%
% Wraps control/allocate.m, so the weighted pseudo-inverse, the elevator
% priority and every saturation come from the delivered allocator. The plant's
% sixth input dT stays zero because allocate realises the thrust through the two
% throttles, which is what its own header states.
%
% Sample time is P.sim.Ts_alloc.

setup(block);
end

function setup(block)
block.NumInputPorts = 2;
block.NumOutputPorts = 1;

for k = 1:2
   block.InputPort(k).DatatypeID = 0;
   block.InputPort(k).Complexity = 'Real';
   block.InputPort(k).SamplingMode = 'Sample';
   block.InputPort(k).DirectFeedthrough = false;
end
block.InputPort(1).Dimensions = 3;
block.InputPort(2).Dimensions = 1;

block.OutputPort(1).Dimensions = 5;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.SampleTimes = [evalin('base', 'sim_ts_alloc'), 0];
block.SimStateCompliance = 'DefaultSimState';
block.NumDialogPrms = 0;

block.RegBlockMethod('Outputs', @outputs);
end

function outputs(block)
M_cmd = block.InputPort(1).Data;
T_cmd = block.InputPort(2).Data;
[da, de, dr, dth_l, dth_r] = sim_alloc_act([], M_cmd, T_cmd);
block.OutputPort(1).Data = [da; de; dr; dth_l; dth_r];
end
