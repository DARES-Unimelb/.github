function info = sim_bench_wind(mdl)
% SIM_BENCH_WIND 台架的大气与风扰动子系统：常值风、阵风、画布侧风微调。
%
%   大气与风扰动/   无输入      出 wind_ned 3x1 [北 东 地]
%
% 三项相加，全部来自 uacs_go.m 第 1.10 节的 uacs_dist
%   常值风    sim_dist_wind_ned，NED 三分量 [m/s]
%   阵风      sim_dist_gust_dir 方向的简谐风，幅度 sim_dist_gust_amp、频率
%             sim_dist_gust_freq（弧度每秒，由赫兹换算而来）。幅度为 0 就是关掉。
%   侧风微调  画布上那个"侧风"滑杆绑的变量 sim_dist_wind_e，只加到东向分量上，
%             用来在跑的过程中临时加一股侧风
%
% 阵风用向量增益而不是逐分量拼：Gain 块取 Matrix(K*u)，K 是 3x1 的方向向量，
% u 是标量正弦，出来就是 3x1。这样方向由参数定，不用改模型。

sub = [mdl '/大气与风扰动'];
sim_add_block(mdl, '大气与风扰动', 'simulink/Ports & Subsystems/Subsystem', [1520 300 1680 460]);
sim_clear_model(sub);

sim_add_block(sub, 'wind_base', 'simulink/Sources/Constant', sim_pos(0, 0, 110, 30), ...
   'Value', 'sim_dist_wind_ned');
sim_add_block(sub, 'wind_e', 'simulink/Sources/Constant', sim_pos(0, 2, 90, 30), ...
   'Value', 'sim_dist_wind_e');
sim_add_block(sub, 'e_zero', 'simulink/Sources/Constant', sim_pos(0, 4, 60, 30), ...
   'Value', 'sim_c_zero');
sim_add_block(sub, 'e_mux', 'simulink/Signal Routing/Mux', sim_pos(1, 2, 20, 70), ...
   'Inputs', '3');
sim_add_block(sub, 'gust', 'simulink/Sources/Sine Wave', sim_pos(0, 6, 90, 40), ...
   'Amplitude', 'sim_dist_gust_amp', 'Frequency', 'sim_dist_gust_freq', ...
   'SampleTime', '0');
sim_add_block(sub, 'gust_gain', 'simulink/Math Operations/Gain', sim_pos(1, 6, 90, 40), ...
   'Gain', 'sim_dist_gust_dir', 'Multiplication', 'Matrix(K*u)');
sim_add_block(sub, 'wind_sum', 'simulink/Math Operations/Sum', sim_pos(2, 2, 40, 50), ...
   'Inputs', '+++', 'IconShape', 'rectangular');
sim_add_block(sub, 'wind_ned', 'simulink/Sinks/Out1', sim_pos(3, 2, 40, 14), 'Port', '1');

sim_connect(sub, 'wind_e/1', 'e_mux/1', 'e_zero/1', 'e_mux/2', ...
   'e_zero/1', 'e_mux/3');
sim_connect(sub, 'wind_base/1', 'wind_sum/1', 'e_mux/1', 'wind_sum/2', ...
   'gust/1', 'gust_gain/1', 'gust_gain/1', 'wind_sum/3', ...
   'wind_sum/1', 'wind_ned/1');

sim_name_line(sub, 'wind_sum/1', 'wind_ned 3x1 [北 东 地] m/s');

info = struct('name', '大气与风扰动', 'path', sub);
end
