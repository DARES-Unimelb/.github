function sim_pilot_sfun(block)
% SIM_PILOT_SFUN Level-2 MATLAB S-Function: RC stick to reference commands
%
%   in  1  stick 4x1  [roll pitch yaw throttle]
%   out 1  cmd   5x1  [phi_c theta_c r_c V_c h_c]
%
% Wraps sim_pilot_wrap, so the stick shaping and the ramp limits live in that
% function and not in the model; the sample time is sim_ts_outer from the base
% workspace
%
% cmd is packed in the frozen order above, which the scenario scripts rely on

setup(block);
end

function setup(block)
block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 4;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).DirectFeedthrough = false;

block.OutputPort(1).Dimensions = 5;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.SampleTimes = [evalin('base', 'sim_ts_outer'), 0];
block.SimStateCompliance = 'DefaultSimState';
block.NumDialogPrms = 0;

block.RegBlockMethod('Outputs', @outputs);
end

function outputs(block)
s = block.InputPort(1).Data;
stick = struct('roll', s(1), 'pitch', s(2), 'yaw', s(3), 'throttle', s(4));
cmd = sim_pilot_wrap([], stick, block.CurrentTime);
block.OutputPort(1).Data = [cmd.phi_c; cmd.theta_c; cmd.r_c; cmd.V_c; cmd.h_c];
end
