function info = sim_bench_select(mdl)
% SIM_BENCH_SELECT Controller selector subsystem, switching the output between PID and LQR
%
%   in du_pid 4x1 / du_lqr 4x1    out du 4x1 / state 1x1
%
% sim_ctrl_sel selects: 0 is PID, 1 is LQR. The Switch criterion is u2 ~= 0, so
% port 1 takes LQR and port 3 takes PID

sub = [mdl '/控制器选择'];
sim_add_block(mdl, '控制器选择', 'simulink/Ports & Subsystems/Subsystem', [1520 60 1670 180]);
sim_clear_model(sub);

sim_add_block(sub, 'du_pid', 'simulink/Sources/In1', sim_pos(0, 0, 30, 14), 'Port', '1');
sim_add_block(sub, 'du_lqr', 'simulink/Sources/In1', sim_pos(0, 3, 30, 14), 'Port', '2');
sim_add_block(sub, 'k_sel', 'simulink/Sources/Constant', sim_pos(0, 5, 90, 30), ...
   'Value', 'sim_ctrl_sel');
sim_add_block(sub, 'sw', 'simulink/Signal Routing/Switch', sim_pos(2, 1, 60, 70), ...
   'Criteria', 'u2 ~= 0', 'Threshold', 'sim_c_zero');
sim_add_block(sub, 'du', 'simulink/Sinks/Out1', sim_pos(3, 1, 30, 14), 'Port', '1');
sim_add_block(sub, 'state', 'simulink/Sinks/Out1', sim_pos(3, 6, 30, 14), 'Port', '2');

sim_connect(sub, 'du_lqr/1', 'sw/1', 'k_sel/1', 'sw/2', 'du_pid/1', 'sw/3', 'sw/1', 'du/1');
sim_connect(sub, 'k_sel/1', 'state/1');
sim_name_line(sub, 'du_lqr/1', '1 LQR');
sim_name_line(sub, 'du_pid/1', '0 PID');

info = struct('name', '控制器选择', 'path', sub);
end
