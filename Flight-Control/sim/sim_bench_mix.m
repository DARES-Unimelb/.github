function info = sim_bench_mix(mdl)
% SIM_BENCH_MIX Mixing and allocation subsystem: surface increments to actuator commands
%
%   in du 4x1    out u6 6x1 / act5 5x1 / dr_deg 1x1 / da_deg 1x1 / de_deg 1x1
%
%   du    = [dda dde ddr dT], the interface both controllers use
%   act5  = [da de dr dth_l dth_r], radians and throttle fraction
%   u6    = [act5; dT_direct], the six input plant vector. Component 6 is the
%           plant's direct thrust increment and stays zero here: thrust goes
%           through the two throttles, which are already in act5
%
% Three steps: add the sim_u_trim_surf trim deflections to the three surfaces,
% form the moment command M_cmd = G_surf*surf with the effectiveness matrix
% sim_G_surf, then allocate with sim_alloc_sfun
%
% dr_deg is negated so the rudder gauge follows the stick direction
% The three degree outputs are separate because an HMI block binds a whole signal,
% not one element of a vector

sub = [mdl '/混合与分配'];
sim_add_block(mdl, '混合与分配', 'simulink/Ports & Subsystems/Subsystem', [1720 40 1890 220]);
sim_clear_model(sub);

sim_add_block(sub, 'du', 'simulink/Sources/In1', sim_pos(0, 1, 30, 14), 'Port', '1');
sim_add_block(sub, 'du_split', 'simulink/Signal Routing/Demux', sim_pos(1, 1, 40, 90), ...
   'Outputs', '4');
sim_add_block(sub, 'const_utrim', 'simulink/Sources/Constant', sim_pos(0, 5, 90, 30), ...
   'Value', 'sim_u_trim_surf');
sim_add_block(sub, 'utrim_split', 'simulink/Signal Routing/Demux', sim_pos(1, 5, 40, 90), ...
   'Outputs', '3');
sim_add_block(sub, 'const_Ttrim', 'simulink/Sources/Constant', sim_pos(0, 8, 80, 30), ...
   'Value', 'sim_T_trim');
sim_add_block(sub, 'const_zero', 'simulink/Sources/Constant', sim_pos(0, 10, 60, 30), ...
   'Value', 'sim_c_zero');

sim_add_block(sub, 'da_sum', 'simulink/Math Operations/Sum', sim_pos(2, 4, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(sub, 'de_sum', 'simulink/Math Operations/Sum', sim_pos(2, 5, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(sub, 'dr_sum', 'simulink/Math Operations/Sum', sim_pos(2, 6, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(sub, 'surf_mux', 'simulink/Signal Routing/Mux', sim_pos(3, 4, 20, 90), ...
   'Inputs', '3');
sim_add_block(sub, 'G_gain', 'simulink/Math Operations/Gain', sim_pos(4, 4, 90, 40), ...
   'Gain', 'sim_G_surf', 'Multiplication', 'Matrix(K*u)');
sim_add_block(sub, 'T_sum', 'simulink/Math Operations/Sum', sim_pos(4, 8, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(sub, 'alloc', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(5, 4, 120, 70), 'FunctionName', 'sim_alloc_sfun');
sim_add_block(sub, 'act_split', 'simulink/Signal Routing/Demux', sim_pos(6, 4, 40, 140), ...
   'Outputs', '5');
sim_add_block(sub, 'u6_mux', 'simulink/Signal Routing/Mux', sim_pos(7, 4, 20, 150), ...
   'Inputs', '6');
sim_add_block(sub, 'act5', 'simulink/Sinks/Out1', sim_pos(6, 8, 30, 14), 'Port', '2');
sim_add_block(sub, 'u6', 'simulink/Sinks/Out1', sim_pos(8, 4, 30, 14), 'Port', '1');
sim_add_block(sub, 'da_deg', 'simulink/Math Operations/Gain', sim_pos(6, 5, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'da_deg_out', 'simulink/Sinks/Out1', sim_pos(7, 5, 40, 14), 'Port', '4');
sim_add_block(sub, 'de_deg', 'simulink/Math Operations/Gain', sim_pos(6, 7, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'de_deg_out', 'simulink/Sinks/Out1', sim_pos(7, 7, 40, 14), 'Port', '5');
sim_add_block(sub, 'dr_deg', 'simulink/Math Operations/Gain', sim_pos(6, 6, 50, 26), ...
   'Gain', '-180/pi');
sim_add_block(sub, 'dr_deg_out', 'simulink/Sinks/Out1', sim_pos(7, 6, 40, 14), 'Port', '3');
sim_connect(sub, 'act_split/1', 'da_deg/1', 'da_deg/1', 'da_deg_out/1', ...
   'act_split/2', 'de_deg/1', 'de_deg/1', 'de_deg_out/1', ...
   'act_split/3', 'dr_deg/1', 'dr_deg/1', 'dr_deg_out/1');

sim_connect(sub, 'du/1', 'du_split/1', 'const_utrim/1', 'utrim_split/1');
sim_connect(sub, 'du_split/1', 'da_sum/1', 'utrim_split/1', 'da_sum/2', ...
   'du_split/2', 'de_sum/1', 'utrim_split/2', 'de_sum/2', ...
   'du_split/3', 'dr_sum/1', 'utrim_split/3', 'dr_sum/2');
sim_connect(sub, 'da_sum/1', 'surf_mux/1', 'de_sum/1', 'surf_mux/2', ...
   'dr_sum/1', 'surf_mux/3', 'surf_mux/1', 'G_gain/1', 'G_gain/1', 'alloc/1');
sim_connect(sub, 'du_split/4', 'T_sum/1', 'const_Ttrim/1', 'T_sum/2', 'T_sum/1', 'alloc/2');
sim_connect(sub, 'alloc/1', 'act_split/1', 'alloc/1', 'act5/1');
sim_connect(sub, 'act_split/1', 'u6_mux/1', 'act_split/2', 'u6_mux/2', ...
   'act_split/3', 'u6_mux/3', 'act_split/4', 'u6_mux/4', ...
   'act_split/5', 'u6_mux/5', 'const_zero/1', 'u6_mux/6', 'u6_mux/1', 'u6/1');

sim_name_line(sub, 'alloc/1', 'act5 5x1 [da de dr dth_l dth_r]');
sim_name_line(sub, 'da_deg/1', 'da 副翼偏度 [deg]');
sim_name_line(sub, 'de_deg/1', 'de 升降舵偏度 [deg]');
sim_name_line(sub, 'dr_deg/1', 'dr 方向舵偏度 右舵正 [deg]');
sim_name_line(sub, 'G_gain/1', 'M_cmd 3x1 力矩指令 [roll pitch yaw]');

info = struct('name', '混合与分配', 'path', sub);
end
