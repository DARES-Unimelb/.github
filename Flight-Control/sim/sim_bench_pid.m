function info = sim_bench_pid(mdl)
% SIM_BENCH_PID 台架的 PID 控制器子系统：整套串级 PID 装进一个块。
%
%   PID控制器/   入 ref 5x1 / fb 9x1      出 du 4x1 / rate_cmd 3x1
%
%   du       = [dda dde ddr dT]，三个舵面增量与推力增量，单位与混合与分配一致
%   rate_cmd = [p_c q_c r_c]，限幅后的速率指令，供示波器与仪表看
%
% 七个回路，全部用 sim_add_pid 生成，增益从基础工作区读
%   pid_phi   姿态环，误差 ref(1)-fb(1)，输出滚转速率指令，经 p_c_lim 限幅
%   pid_theta 姿态环，误差 ref(2)-fb(2)，输出俯仰速率指令，经 q_c_lim 限幅
%   pid_p     p 速率环，误差 p_c-fb(3)，输出副翼增量
%   pid_q     q 速率环，误差 q_c-fb(4)，输出升降舵增量
%   pid_r     r 速率环，误差 r_c-fb(5)，输出方向舵增量
%   pid_V     空速环，误差 ref(4)-fb(6)，输出推力增量
%
% 速率指令限幅器把姿态环的输出卡在机体真能给出的角速率上。没有它，一个大的姿态
% 误差会变成任意大的速率要求：滚转杆到底曾要求 17.73 rad/s，而机体只能维持 1.23，
% 副翼饱和、侧滑到 9 度、飞机翻了。
%
% 接线与受验模型 model/uacs_manual_pid.slx 逐根一致，包括偏航这一路的做法：限幅器
% 串在 r 的误差之后而不是指令之后，rate_cmd 的第 3 路取的是未限幅的 r_c。这是既有
% 行为，改它会让 V6 与 V13 的对照失去意义，所以原样保留。

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

% 姿态环到速率环：误差 → PID → 限幅 → 速率环误差
sim_connect(sub, 'ref/1', 'ref_split/1', 'fb/1', 'fb_split/1');
sim_connect(sub, 'ref_split/1', 'phi_err/1', 'fb_split/1', 'phi_err/2', ...
   'phi_err/1', 'pid_phi/1', 'pid_phi/1', 'p_c_lim/1');
sim_connect(sub, 'ref_split/2', 'theta_err/1', 'fb_split/2', 'theta_err/2', ...
   'theta_err/1', 'pid_theta/1', 'pid_theta/1', 'q_c_lim/1');
sim_connect(sub, 'ref_split/3', 'r_err/1', 'fb_split/5', 'r_err/2', ...
   'r_err/1', 'r_c_lim/1', 'r_c_lim/1', 'pid_r/1');
sim_connect(sub, 'ref_split/4', 'V_err/1', 'fb_split/6', 'V_err/2', 'V_err/1', 'pid_V/1');

% 速率环
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
