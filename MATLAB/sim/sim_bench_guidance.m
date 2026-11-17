function info = sim_bench_guidance(mdl)
% SIM_BENCH_GUIDANCE 台架的制导与模式子系统：把驾驶员指令加工成有效参考。
%
%   制导与模式/   入 cmd 5x1 / fb 9x1      出 ref 5x1
%
%   cmd 与 ref 的分量次序相同，都是 [phi_c theta_c r_c V_c h_c]。
%   姿态模式下 ref 就是 cmd；高度模式下第二个分量换成 pid_h 的输出，
%   也就是"俯仰杆变成爬升率指令"，由高度环把 theta 当中间指令用。
%
% fb 是导航与测量送来的反馈向量，次序固定
%   1 phi   2 theta   3 p   4 q   5 r   6 V   7 h   8 alpha   9 beta
%
% 飞行模式的选择量 sim_flight_mode 是本子系统里的一个 Constant，画布上那个"模式"
% 拨动开关绑的就是它。放在这里而不是顶层，是为了顶层不再多一根选择线。
%
% 高度环 pid_h 放在这里而不是放在 PID 控制器里，是因为两个控制器都要用它：
% 姿态模式下它就是纯透传，高度模式下它给出俯仰指令。放在任一控制器里都会让另一个
% 控制器反过来依赖那个块。
%
% theta_lim 把 pid_h 的输出卡在俯仰包线上。没有它，高度环在大误差下会给出超过结构
% 限制的俯仰指令。

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
% 模式档位单独送出去一路，顶层那块状态读数接的就是它。
sim_add_block(sub, 'state', 'simulink/Sinks/Out1', sim_pos(7, 6, 30, 14), 'Port', '2');

% 高度误差用的是 cmd 的第 5 路与 fb 的第 7 路。
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
