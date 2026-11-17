function info = sim_add_pid(mdl, name, pos, loop)
% SIM_ADD_PID Assemble one parallel PID as a subsystem.
%
%   info = sim_add_pid(mdl, 'pid_q', pos, 'q')
%
% Structure, all parameters read from the base workspace
%   e -> pol*Kp        -> sum
%   e -> pol*Ki/s      -> sum      Discrete-Time Integrator, sample time, clamped
%   e -> pol*Kd*s/(Tf*s+1) -> sum  one Discrete Transfer Fcn with Tustin coefficients
%   sum -> pol gain -> u
%
% The sample time comes from sim_pid_<loop>_Ts, which the design pusher sets to
% P.sim.Ts_ctrl for the three rate loops and to P.sim.Ts_outer for the attitude,
% airspeed and altitude loops. The integral path is clamped at
% +-sim_pid_<loop>_Isat, which is the loop's authority limit: the design's
% P.ctrl.int_limit entry where it has one, the surface limit for the rate loops.
%
% pol = sim_pid_<loop>_pol = sign(gains.<loop>.Kp) = gains.<loop>.sign is the
% loop polarity the design chose. The error is always err = cmd - meas, so a loop
% whose plant carries a right half plane zero in the wrong direction needs a
% negative polarity through the whole controller, and the pid_theta pitch
% attitude loop is one of them: its plant has a negative DC gain (positive
% elevator gives a nose down moment), so a positive pitch error must command a
% negative pitch rate. The polarity is applied once, by the Gain block named pol
% between the sum and the output, while the three gain blocks carry magnitudes.
% Applying it anywhere else would need a different sign per path, because the
% Discrete-Time Integrator accepts a positive gainval only.
%
% gains.<loop>.Kb is the design's back-calculation anti-windup gain; the block
% path uses output clamping on the integrator instead, which is the form the
% Discrete-Time Integrator supports. The difference is recorded in
% docs/design_notes_sim.md.

sub = [mdl '/' name];
sim_add_block(mdl, name, 'simulink/Ports & Subsystems/Subsystem', pos);
sim_clear_model(sub);
seq = @(f) sprintf('sim_pid_%s_%s', loop, f);

sim_add_block(sub, 'e', 'simulink/Sources/In1', sim_pos(0, 1, 30, 14), 'Port', '1');
% All three gains carry magnitudes; sim_push_design_vars publishes the absolute
% values. Negating gainval instead would apply the polarity to the proportional
% and integral paths but not to the derivative tap, whose numerator is built from
% Kd alone by sim_pid_coeffs.
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
