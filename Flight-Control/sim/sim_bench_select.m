function info = sim_bench_select(mdl)
% SIM_BENCH_SELECT 台架的控制器选择子系统：在 PID 与 LQR 之间切输出。
%
%   控制器选择/   入 du_pid 4x1 / du_lqr 4x1      出 du 4x1
%
% 两个控制器是并联同时跑的，不是互斥使能。这样切换是瞬时的，不触发重编译，切过去
% 的那一个积分器已经跟上了误差，不会在切换瞬间给出一个需要长时间恢复的大指令。
% 代价是两个控制器的积分状态都在走，所以切换前的饱和历史会被带过去。
%
% 选择量 sim_ctrl_sel 是本子系统里的一个 Constant：0 取 PID，1 取 LQR，画布上那个
% "控制器"旋转开关绑的就是它。Switch 的判据是 u2 ~= 0，所以端口 1 接 LQR、端口 3
% 接 PID。

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
% 控制器档位单独送出去一路，顶层那块状态读数接的就是它。
sim_add_block(sub, 'state', 'simulink/Sinks/Out1', sim_pos(3, 6, 30, 14), 'Port', '2');

sim_connect(sub, 'du_lqr/1', 'sw/1', 'k_sel/1', 'sw/2', 'du_pid/1', 'sw/3', 'sw/1', 'du/1');
sim_connect(sub, 'k_sel/1', 'state/1');
sim_name_line(sub, 'du_lqr/1', '1 LQR');
sim_name_line(sub, 'du_pid/1', '0 PID');

info = struct('name', '控制器选择', 'path', sub);
end
