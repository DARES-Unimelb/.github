function info = sim_bench_pid(mdl)
% SIM_BENCH_PID PID controller subsystem: the whole cascade inside one block
%
%   in ref 5x1 / fb 9x1    out du 4x1 / rate_cmd 3x1
%
%   du       = [dda dde ddr dT]
%   rate_cmd = [p_c q_c r_c], the limited rate commands, for scopes and gauges
%
% Six loops, all built by sim_add_pid, gains read from base
%   pid_phi    attitude, error ref(1)-fb(1), outputs the roll rate command, limited by p_c_lim
%   pid_theta  attitude, error ref(2)-fb(2), outputs the pitch rate command, limited by q_c_lim
%   pid_p      rate, error p_c-fb(3), outputs the aileron increment
%   pid_q      rate, error q_c-fb(4), outputs the elevator increment
%   pid_r      rate, error r_c-fb(5), outputs the rudder increment
%   pid_V      airspeed, error ref(4)-fb(6), outputs the thrust increment
%
% The rate limiters keep an attitude error from demanding a rate the airframe
% cannot hold
% The yaw limiter sits after the r error rather than after the command, so
% rate_cmd(3) is the unlimited r_c

sub = [mdl '/PID控制器'];
sim_add_block(mdl, 'PID控制器', 'simulink/Ports & Subsystems/Subsystem', [1280 40 1450 210]);
sim_clear_model(sub);

sim_add_block(sub, 'ref', 'simulink/Sources/In1', sim_pos(0, 0, 30, 14), 'Port', '1');
sim_add_block(sub, 'fb', 'simulink/Sources/In1', sim_pos(0, 3, 30, 14), 'Port', '2');
sim_add_block(sub, 'ref_split', 'simulink/Signal Routing/Demux', sim_pos(1, 0, 40, 110), ...
   'Outputs', '5');
sim_add_block(sub, 'fb_split', 'simulink/Signal Routing/Demux', sim_pos(1, 3, 40, 200), ...
   'Outputs', '9');

sim_add_block(sub, 'phi_err', 'simulink/Math Operations/Sum', sim_pos(2, 1, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(sub, 'theta_err', 'simulink/Math Operations/Sum', sim_pos(2, 2, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(sub, 'r_err', 'simulink/Math Operations/Sum', sim_pos(2, 3, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(sub, 'V_err', 'simulink/Math Operations/Sum', sim_pos(2, 4, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');

sim_add_pid(sub, 'pid_phi', sim_pos(3, 1, 120, 60), 'phi');
sim_add_pid(sub, 'pid_theta', sim_pos(3, 2, 120, 60), 'theta');
sim_add_pid(sub, 'pid_V', sim_pos(3, 4, 120, 60), 'V');

sim_add_block(sub, 'p_c_lim', 'simulink/Discontinuities/Saturation', sim_pos(5, 0, 60, 30), ...
   'UpperLimit', 'sim_rate_cmd_lim_p', 'LowerLimit', '-sim_rate_cmd_lim_p');
sim_add_block(sub, 'q_c_lim', 'simulink/Discontinuities/Saturation', sim_pos(5, 1, 60, 30), ...
   'UpperLimit', 'sim_rate_cmd_lim_q', 'LowerLimit', '-sim_rate_cmd_lim_q');
sim_add_block(sub, 'r_c_lim', 'simulink/Discontinuities/Saturation', sim_pos(5, 4, 60, 30), ...
   'UpperLimit', 'sim_rate_cmd_lim_r', 'LowerLimit', '-sim_rate_cmd_lim_r');

sim_add_block(sub, 'p_err', 'simulink/Math Operations/Sum', sim_pos(6, 0, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(sub, 'q_err', 'simulink/Math Operations/Sum', sim_pos(6, 1, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_pid(sub, 'pid_p', sim_pos(7, 0, 120, 60), 'p');
sim_add_pid(sub, 'pid_q', sim_pos(7, 1, 120, 60), 'q');
sim_add_pid(sub, 'pid_r', sim_pos(7, 4, 120, 60), 'r');

sim_add_block(sub, 'rate_mux', 'simulink/Signal Routing/Mux', sim_pos(8, 1, 20, 90), ...
   'Inputs', '3');
sim_add_block(sub, 'rate_cmd', 'simulink/Sinks/Out1', sim_pos(9, 1, 40, 14), 'Port', '2');
sim_add_block(sub, 'du_mux', 'simulink/Signal Routing/Mux', sim_pos(9, 3, 20, 90), ...
   'Inputs', '4');
sim_add_block(sub, 'du', 'simulink/Sinks/Out1', sim_pos(10, 3, 30, 14), 'Port', '1');

%% Attitude to rate loop
sim_connect(sub, 'ref/1', 'ref_split/1', 'fb/1', 'fb_split/1');
sim_connect(sub, 'ref_split/1', 'phi_err/1', 'fb_split/1', 'phi_err/2', ...
   'phi_err/1', 'pid_phi/1', 'pid_phi/1', 'p_c_lim/1');
sim_connect(sub, 'ref_split/2', 'theta_err/1', 'fb_split/2', 'theta_err/2', ...
   'theta_err/1', 'pid_theta/1', 'pid_theta/1', 'q_c_lim/1');
sim_connect(sub, 'ref_split/3', 'r_err/1', 'fb_split/5', 'r_err/2', ...
   'r_err/1', 'r_c_lim/1', 'r_c_lim/1', 'pid_r/1');
sim_connect(sub, 'ref_split/4', 'V_err/1', 'fb_split/6', 'V_err/2', 'V_err/1', 'pid_V/1');

%% Rate loops
sim_connect(sub, 'p_c_lim/1', 'p_err/1', 'fb_split/3', 'p_err/2', ...
   'p_err/1', 'pid_p/1');
sim_connect(sub, 'q_c_lim/1', 'q_err/1', 'fb_split/4', 'q_err/2', ...
   'q_err/1', 'pid_q/1');
sim_connect(sub, 'p_c_lim/1', 'rate_mux/1', 'q_c_lim/1', 'rate_mux/2', ...
   'ref_split/3', 'rate_mux/3', 'rate_mux/1', 'rate_cmd/1');
sim_connect(sub, 'pid_p/1', 'du_mux/1', 'pid_q/1', 'du_mux/2', ...
   'pid_r/1', 'du_mux/3', 'pid_V/1', 'du_mux/4', 'du_mux/1', 'du/1');

sim_name_line(sub, 'du_mux/1', 'du 4x1 [dda dde ddr dT]');
sim_name_line(sub, 'rate_mux/1', 'rate_cmd 3x1 [p_c q_c r_c]');

info = struct('name', 'PID控制器', 'path', sub);
end
