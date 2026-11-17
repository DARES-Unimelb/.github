function info = sim_bench_guidance(mdl)
% SIM_BENCH_GUIDANCE Guidance and mode subsystem, turning pilot commands into an effective reference
%
%   in cmd 5x1 / fb 9x1    out ref 5x1 / state 1x1
%
% cmd and ref share the component order [phi_c theta_c r_c V_c h_c]. In attitude
% mode ref is cmd; in altitude mode component 2 becomes the pid_h output, so the
% pitch stick commands a climb rate while the altitude loop drives theta
%
% fb is the feedback vector, fixed order
%   1 phi   2 theta   3 p   4 q   5 r   6 V   7 h   8 alpha   9 beta
%
% sim_flight_mode selects the mode

sub = [mdl '/制导与模式'];
sim_add_block(mdl, '制导与模式', 'simulink/Ports & Subsystems/Subsystem', [1060 60 1220 210]);
sim_clear_model(sub);

sim_add_block(sub, 'cmd', 'simulink/Sources/In1', sim_pos(0, 0, 30, 14), 'Port', '1');
sim_add_block(sub, 'fb', 'simulink/Sources/In1', sim_pos(0, 3, 30, 14), 'Port', '2');
sim_add_block(sub, 'cmd_split', 'simulink/Signal Routing/Demux', sim_pos(1, 0, 40, 110), ...
   'Outputs', '5');
sim_add_block(sub, 'fb_split', 'simulink/Signal Routing/Demux', sim_pos(1, 3, 40, 200), ...
   'Outputs', '9');

sim_add_block(sub, 'mode_const', 'simulink/Sources/Constant', sim_pos(0, 6, 90, 30), ...
   'Value', 'sim_flight_mode');
sim_add_block(sub, 'h_err', 'simulink/Math Operations/Sum', sim_pos(2, 5, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_pid(sub, 'pid_h', sim_pos(3, 5, 120, 60), 'h');
sim_add_block(sub, 'theta_lim', 'simulink/Discontinuities/Saturation', sim_pos(5, 5, 60, 30), ...
   'UpperLimit', 'sim_pid_h_lim', 'LowerLimit', 'sim_pid_h_lim_neg');
sim_add_block(sub, 'theta_switch', 'simulink/Signal Routing/Switch', sim_pos(5, 2, 60, 60), ...
   'Criteria', 'u2 ~= 0', 'Threshold', 'sim_c_zero');
sim_add_block(sub, 'ref_mux', 'simulink/Signal Routing/Mux', sim_pos(6, 2, 20, 110), ...
   'Inputs', '5');
sim_add_block(sub, 'ref', 'simulink/Sinks/Out1', sim_pos(7, 2, 30, 14), 'Port', '1');
sim_add_block(sub, 'state', 'simulink/Sinks/Out1', sim_pos(7, 6, 30, 14), 'Port', '2');

% altitude error: cmd component 5 minus fb component 7
sim_connect(sub, 'cmd/1', 'cmd_split/1', 'fb/1', 'fb_split/1');
sim_connect(sub, 'cmd_split/5', 'h_err/1', 'fb_split/7', 'h_err/2');
sim_connect(sub, 'h_err/1', 'pid_h/1', 'pid_h/1', 'theta_lim/1', ...
   'theta_lim/1', 'theta_switch/1', 'mode_const/1', 'theta_switch/2', ...
   'cmd_split/2', 'theta_switch/3');
sim_connect(sub, 'cmd_split/1', 'ref_mux/1', 'theta_switch/1', 'ref_mux/2', ...
   'cmd_split/3', 'ref_mux/3', 'cmd_split/4', 'ref_mux/4', 'cmd_split/5', 'ref_mux/5', ...
   'ref_mux/1', 'ref/1');
sim_connect(sub, 'mode_const/1', 'state/1');

sim_name_line(sub, 'cmd_split/1', 'phi_c');
sim_name_line(sub, 'theta_switch/1', 'theta_c 有效：模式 0 取杆位，模式 1 取 pid_h 输出');
sim_name_line(sub, 'ref_mux/1', 'ref 5x1 [phi_c theta_c r_c V_c h_c]');

info = struct('name', '制导与模式', 'path', sub);
end
