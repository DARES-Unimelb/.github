function info = sim_bench_nav(mdl)
% SIM_BENCH_NAV 台架的导航与测量子系统：状态分解、大气数据、仪表分接。
%
%   导航与测量/   入 x 12x1      出 fb 9x1 / x 12x1 / inst 6x1
%
% fb 是反馈向量，次序在这里定死，下游三个子系统都按这个次序 Demux
%   1 phi   2 theta   3 p   4 q   5 r   6 V   7 h   8 alpha   9 beta
%
% 高度取负号：机体系 z 向下，状态向量的第 12 路是 pd，向上为正要用 -pd。
%
% inst 是仪表分接向量，供画布上的仪表读数用
%   1 V [m/s]   2 h [m]   3 alpha [deg]   4 phi [deg]   5 theta [deg]   6 stall 0/1
%   7 r [deg/s]
% 角度在这里就换成度，因为仪表刻度是按度给的，读数也按度才看得懂。方向舵偏度不在这里，
% 它出自混合与分配，由那个子系统自己送出来。
%
% 这几个分接块的输出必须真的被用到，否则块化简会把它们连同信号一起删掉，绑在
% 它们上面的仪表就收不到数据。所以 inst 在顶层要接到 Outport 上，Outport 是真正的
% 汇，顺带把这几路记进仿真输出。

sub = [mdl '/导航与测量'];
sim_add_block(mdl, '导航与测量', 'simulink/Ports & Subsystems/Subsystem', [1960 300 2120 500]);
sim_clear_model(sub);

sim_add_block(sub, 'x', 'simulink/Sources/In1', sim_pos(0, 0, 30, 14), 'Port', '1');
sim_add_block(sub, 'x_split', 'simulink/Signal Routing/Demux', sim_pos(1, 0, 40, 260), ...
   'Outputs', '12');
sim_add_block(sub, 'meas', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(1, 3, 100, 40), 'FunctionName', 'sim_meas_sfun');
sim_add_block(sub, 'meas_split', 'simulink/Signal Routing/Demux', sim_pos(2, 3, 40, 90), ...
   'Outputs', '4');
sim_add_block(sub, 'h_neg', 'simulink/Math Operations/Gain', sim_pos(2, 6, 40, 30), ...
   'Gain', 'sim_c_neg1');
sim_add_block(sub, 'fb_mux', 'simulink/Signal Routing/Mux', sim_pos(3, 1, 20, 230), ...
   'Inputs', '9');
sim_add_block(sub, 'fb', 'simulink/Sinks/Out1', sim_pos(4, 1, 30, 14), 'Port', '1');
sim_add_block(sub, 'x_out', 'simulink/Sinks/Out1', sim_pos(4, 3, 30, 14), 'Port', '2');

sim_connect(sub, 'x/1', 'x_split/1', 'x/1', 'meas/1', 'x/1', 'x_out/1');
sim_connect(sub, 'meas/1', 'meas_split/1');
sim_connect(sub, 'x_split/12', 'h_neg/1');

% ---------------- 仪表分接 ----------------
sim_add_block(sub, 'alpha_deg', 'simulink/Math Operations/Gain', sim_pos(3, 8, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'phi_deg', 'simulink/Math Operations/Gain', sim_pos(3, 9, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'theta_deg', 'simulink/Math Operations/Gain', sim_pos(3, 10, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'r_deg', 'simulink/Math Operations/Gain', sim_pos(3, 11, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'alpha_abs', 'simulink/Math Operations/Abs', sim_pos(4, 8, 40, 26));
% 失速比较用两输入形式，不用块自带的常数阈值：那个阈值的参数名在不同版本里不一样，
% 两输入形式没有这个不确定。
sim_add_block(sub, 'const_stall', 'simulink/Sources/Constant', sim_pos(4, 10, 90, 30), ...
   'Value', 'sim_alpha_stall_abs');
sim_add_block(sub, 'stall_rel', 'simulink/Logic and Bit Operations/Relational Operator', ...
   sim_pos(5, 8, 60, 26), 'Operator', '>');
% 关系运算输出的是 boolean，直接进 Mux 会报数据类型不匹配
% （Simulink:DataType:InputPortDataTypeMismatch），所以过一级转换成 double。
sim_add_block(sub, 'stall_dbl', 'simulink/Signal Attributes/Data Type Conversion', ...
   sim_pos(6, 8, 60, 26), 'OutDataTypeStr', 'double');
sim_connect(sub, 'meas_split/2', 'alpha_deg/1', 'meas_split/2', 'alpha_abs/1');
sim_connect(sub, 'alpha_abs/1', 'stall_rel/1', 'const_stall/1', 'stall_rel/2');
sim_connect(sub, 'stall_rel/1', 'stall_dbl/1');
sim_connect(sub, 'x_split/7', 'phi_deg/1', 'x_split/8', 'theta_deg/1', ...
   'x_split/6', 'r_deg/1');

sim_add_block(sub, 'inst_mux', 'simulink/Signal Routing/Mux', sim_pos(7, 7, 20, 160), ...
   'Inputs', '7');
sim_add_block(sub, 'inst', 'simulink/Sinks/Out1', sim_pos(8, 7, 30, 14), 'Port', '3');
sim_connect(sub, 'meas_split/1', 'inst_mux/1', 'h_neg/1', 'inst_mux/2', ...
   'alpha_deg/1', 'inst_mux/3', 'phi_deg/1', 'inst_mux/4', ...
   'theta_deg/1', 'inst_mux/5', 'stall_dbl/1', 'inst_mux/6', ...
   'r_deg/1', 'inst_mux/7', 'inst_mux/1', 'inst/1');

% ---------------- 反馈向量 ----------------
% 次序 phi theta p q r V h alpha beta
sim_connect(sub, 'x_split/7', 'fb_mux/1', 'x_split/8', 'fb_mux/2', ...
   'x_split/4', 'fb_mux/3', 'x_split/5', 'fb_mux/4', 'x_split/6', 'fb_mux/5', ...
   'meas_split/1', 'fb_mux/6', 'h_neg/1', 'fb_mux/7', ...
   'meas_split/2', 'fb_mux/8', 'meas_split/3', 'fb_mux/9', 'fb_mux/1', 'fb/1');

sim_name_line(sub, 'fb_mux/1', 'fb 9x1 [phi theta p q r V h alpha beta]');
sim_name_line(sub, 'inst_mux/1', 'inst 7x1 [V h alpha_deg phi_deg theta_deg stall r_deg]');
sim_name_line(sub, 'meas_split/1', 'V 空速 [m/s]');
sim_name_line(sub, 'h_neg/1', 'h 高度 [m]');

info = struct('name', '导航与测量', 'path', sub);
end
