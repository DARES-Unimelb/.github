function info = sim_bench_wind(mdl)
% SIM_BENCH_WIND Atmosphere and wind subsystem: constant wind, gust, canvas crosswind trim
%
%   out wind_ned 3x1 [north east down] m/s
%
% Three terms are summed
%   constant wind  sim_dist_wind_ned, NED components [m/s]
%   gust           sine of amplitude sim_dist_gust_amp and angular frequency
%                  sim_dist_gust_freq [rad/s], along sim_dist_gust_dir; zero
%                  amplitude disables it
%   crosswind      sim_dist_wind_e, added to the east component only

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
