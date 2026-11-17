function info = sim_add_pid(mdl, name, pos, loop)
% SIM_ADD_PID Assemble one parallel PID as a subsystem
%
%   info = sim_add_pid(mdl, name, pos, loop)
%
% Every block parameter is read from base as sim_pid_<loop>_*
% The error is cmd - meas, the sample time is sim_pid_<loop>_Ts, and the
% integral path clamps at +-sim_pid_<loop>_Isat
% sim_pid_<loop>_pol carries the loop polarity and is applied once, by the gain
% block named pol; the Kp, Ki and Kd blocks carry magnitudes only

sub = [mdl '/' name];
sim_add_block(mdl, name, 'simulink/Ports & Subsystems/Subsystem', pos);
sim_clear_model(sub);
seq = @(f) sprintf('sim_pid_%s_%s', loop, f);

sim_add_block(sub, 'e', 'simulink/Sources/In1', sim_pos(0, 1, 30, 14), 'Port', '1');
sim_add_block(sub, 'Kp', 'simulink/Math Operations/Gain', sim_pos(1, 0), ...
   'Gain', seq('Kp_abs'));
sim_add_block(sub, 'Ki', 'simulink/Discrete/Discrete-Time Integrator', sim_pos(1, 1), ...
   'gainval', seq('Ki_abs'), 'SampleTime', seq('Ts'), ...
   'LimitOutput', 'on', ...
   'UpperSaturationLimit', seq('Isat'), ...
   'LowerSaturationLimit', seq('Isat_neg'));
sim_add_block(sub, 'Kd', 'simulink/Discrete/Discrete Transfer Fcn', sim_pos(1, 2), ...
   'Numerator', seq('Dnum_abs'), 'Denominator', seq('Dden'), 'SampleTime', seq('Ts'));
sim_add_block(sub, 'sum', 'simulink/Math Operations/Sum', sim_pos(2, 1), ...
   'Inputs', '+++', 'IconShape', 'rectangular');
sim_add_block(sub, 'pol', 'simulink/Math Operations/Gain', sim_pos(2, 5, 40, 30), ...
   'Gain', seq('pol'), 'Multiplication', 'Element-wise(K.*u)');
sim_add_block(sub, 'u', 'simulink/Sinks/Out1', sim_pos(3, 1, 30, 14), 'Port', '1');

sim_connect(sub, 'e/1', 'Kp/1', 'e/1', 'Ki/1', 'e/1', 'Kd/1');
sim_connect(sub, 'Kp/1', 'sum/1', 'Ki/1', 'sum/2', 'Kd/1', 'sum/3');
sim_connect(sub, 'sum/1', 'pol/1', 'pol/1', 'u/1');

info = struct('name', name, 'loop', loop, 'path', sub);
end
