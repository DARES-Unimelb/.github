function info = sim_bench_cockpit(mdl, opts)
% SIM_BENCH_COCKPIT Canvas cockpit: control desk, status readouts, gauges and scopes
%
% Position arguments are Simulink [left top right bottom], not [x y width height]
%
% Eight Dashboard controls, all portless and driven only by a binding
%   pitch / roll / rudder knobs    bound to the Value parameter of three Constants in the pilot
%   speed command slider           stick position 0..1, an airspeed command and not a throttle
%   crosswind east slider          adds a constant east wind, see sim_bench_wind
%   mode, controller, source       toggles, 0/1
% Three status readouts are classic Display blocks on real lines, because a toggle
% or rotary switch does not show its own position
%
% Gauges bind to signals that also reach a top level Outport: a signal that is only
% bound and never sinked is deleted by block reduction and the gauge loses its data
%
% inst order, as fixed in sim_bench_nav
%   1 V [m/s]   2 h [m]   3 alpha [deg]   4 phi [deg]   5 theta [deg]
%   6 stall 0/1 7 r [deg/s]
%
% A DashboardScope Binding must be a cell array and draws at most four traces

if nargin < 2
   opts = struct();
end

%% Control desk
% bindings target the owner's Value parameter, so those Constants must already hold literals
sim_hmi('knob', mdl, '俯仰杆', [80 90 270 280], [mdl '/驾驶员/k_pitch'], [-1 1]);
sim_hmi('knob', mdl, '滚转杆', [300 90 490 280], [mdl '/驾驶员/k_roll'], [-1 1]);
sim_hmi('knob', mdl, '方向舵', [520 90 710 280], [mdl '/驾驶员/k_yaw'], [-1 1]);
sim_hmi('slider', mdl, '速度指令', [80 330 270 610], [mdl '/驾驶员/k_thr'], [0 1]);
sim_hmi('slider', mdl, '侧风 东向 mps', [300 330 490 610], ...
   [mdl '/大气与风扰动/wind_e'], [-10 10]);
sim_hmi('toggle', mdl, '模式', [540 350 690 420], [mdl '/制导与模式/mode_const'], ...
   {'姿态', '高度'});
sim_hmi('rotary', mdl, '控制器', [540 460 690 530], [mdl '/控制器选择/k_sel'], ...
   {'PID', 'LQR'});
sim_hmi('toggle', mdl, '操纵源', [540 570 690 640], [mdl '/驾驶员/js_on'], ...
   {'虚拟杆', '手柄'});

%% Status readouts
% each subsystem has one state output, which is the selector Constant itself
local_state(mdl, '模式读数', [700 350 800 410], '制导与模式/2');
local_state(mdl, '控制器读数', [700 460 800 520], '控制器选择/2');
local_state(mdl, '操纵源读数', [700 570 800 630], '驾驶员/2');

%% Taps
% split the vectors so gauges and scopes can take single components instead of the whole signal
sim_add_block(mdl, 'fb_split', 'simulink/Signal Routing/Demux', [860 1080 900 1360], ...
   'Outputs', '9');
sim_connect(mdl, '导航与测量/1', 'fb_split/1');
sim_add_block(mdl, 'ref_split', 'simulink/Signal Routing/Demux', [860 1420 900 1530], ...
   'Outputs', '5');
sim_connect(mdl, '制导与模式/1', 'ref_split/1');
sim_add_block(mdl, 'rc_split', 'simulink/Signal Routing/Demux', [860 1600 900 1710], ...
   'Outputs', '3');
sim_connect(mdl, 'PID控制器/2', 'rc_split/1');

sim_add_block(mdl, 'inst_split', 'simulink/Signal Routing/Demux', [760 700 800 960], ...
   'Outputs', '7');
sim_connect(mdl, '导航与测量/3', 'inst_split/1');
names = {'V', 'h', 'alpha_deg', 'phi_deg', 'theta_deg', 'stall', 'r_deg'};
for k = 1:7
   sim_add_block(mdl, names{k}, 'simulink/Sinks/Out1', ...
      [880, 690 + 50 * k, 910, 704 + 50 * k], 'Port', num2str(2 + k));
   sim_connect(mdl, sprintf('inst_split/%d', k), sprintf('%s/1', names{k}));
end

%% Gauges
% twelve gauges: airspeed on the left, the three surfaces on the right, each command
% gauge next to the measured one
gauge = {
   '空速表',          [0 40],    {[mdl '/导航与测量/meas_split'], 1}
   '空速指令表',      [0 40],    {[mdl '/ref_split'], 4}
   '高度表',          [0 400],   {[mdl '/导航与测量/h_neg'], 1}
   '迎角表',          [-15 20],  {[mdl '/导航与测量/alpha_deg'], 1}
   '滚转表',          [-60 60],  {[mdl '/导航与测量/phi_deg'], 1}
   '滚转指令表',      [-60 60],  {[mdl '/ref_split'], 1}
   '俯仰表',          [-30 30],  {[mdl '/导航与测量/theta_deg'], 1}
   '俯仰指令表',      [-30 30],  {[mdl '/ref_split'], 2}
   '偏航率表',        [-60 60],  {[mdl '/导航与测量/r_deg'], 1}
   '方向舵表 右舵正', [-30 30],  {[mdl '/混合与分配/dr_deg'], 1}
   '副翼表 原始偏度', [-30 30],  {[mdl '/混合与分配/da_deg'], 1}
   '升降舵表 原始偏度', [-30 30], {[mdl '/混合与分配/de_deg'], 1}
   };
for k = 1:size(gauge, 1)
   x = 1080 + 190 * (k - 1);
   sim_hmi('gauge', mdl, gauge{k, 1}, [x 700 x + 150 850], gauge{k, 2}, gauge{k, 3});
end
sim_hmi('lamp', mdl, '失速灯', [1080 880 1150 950], {[mdl '/导航与测量/stall_dbl'], 1});

%% Tracking channels
% each pair carries command and measured together, so a scope shows tracking directly
sim_add_block(mdl, 'att_mux', 'simulink/Signal Routing/Mux', [1000 1160 1025 1270], ...
   'Inputs', '4');
sim_connect(mdl, 'ref_split/1', 'att_mux/1', 'fb_split/1', 'att_mux/2', ...
   'ref_split/2', 'att_mux/3', 'fb_split/2', 'att_mux/4');
sim_name_line(mdl, 'ref_split/1', 'phi_c 滚转指令 [rad]');
sim_name_line(mdl, 'ref_split/2', 'theta_c 俯仰指令 [rad]');
sim_name_line(mdl, 'fb_split/1', 'phi 实际滚转 [rad]');
sim_name_line(mdl, 'fb_split/2', 'theta 实际俯仰 [rad]');
sim_add_block(mdl, 'att_track', 'simulink/Sinks/Out1', [1060 1180 1090 1194], 'Port', '10');
sim_connect(mdl, 'att_mux/1', 'att_track/1');

sim_add_block(mdl, 'rate_mux', 'simulink/Signal Routing/Mux', [1000 1360 1025 1520], ...
   'Inputs', '6');
sim_connect(mdl, 'rc_split/1', 'rate_mux/1', 'fb_split/3', 'rate_mux/2', ...
   'rc_split/2', 'rate_mux/3', 'fb_split/4', 'rate_mux/4', ...
   'rc_split/3', 'rate_mux/5', 'fb_split/5', 'rate_mux/6');
sim_name_line(mdl, 'rc_split/1', 'p_c 滚转率指令 [rad/s]');
sim_name_line(mdl, 'rc_split/2', 'q_c 俯仰率指令 [rad/s]');
sim_name_line(mdl, 'rc_split/3', 'r_c 偏航率指令 [rad/s]');
sim_name_line(mdl, 'fb_split/3', 'p 实际滚转率 [rad/s]');
sim_name_line(mdl, 'fb_split/4', 'q 实际俯仰率 [rad/s]');
sim_name_line(mdl, 'fb_split/5', 'r 实际偏航率 [rad/s]');
sim_add_block(mdl, 'rate_track', 'simulink/Sinks/Out1', [1060 1380 1090 1394], 'Port', '11');
sim_connect(mdl, 'rate_mux/1', 'rate_track/1');

% split to five named lines first, then re-mux, so the scope legend shows names not port numbers
sim_add_block(mdl, 'act_split2', 'simulink/Signal Routing/Demux', [1560 1140 1600 1290], ...
   'Outputs', '5');
sim_connect(mdl, '混合与分配/2', 'act_split2/1');
sim_add_block(mdl, 'act_mux', 'simulink/Signal Routing/Mux', [1660 1150 1685 1280], ...
   'Inputs', '5');
sim_connect(mdl, 'act_split2/1', 'act_mux/1', 'act_split2/2', 'act_mux/2', ...
   'act_split2/3', 'act_mux/3', 'act_split2/4', 'act_mux/4', ...
   'act_split2/5', 'act_mux/5');
sim_name_line(mdl, 'act_split2/1', 'da 副翼 [rad]');
sim_name_line(mdl, 'act_split2/2', 'de 升降舵 [rad]');
sim_name_line(mdl, 'act_split2/3', 'dr 方向舵 [rad]');
sim_name_line(mdl, 'act_split2/4', '油门 左 0..1');
sim_name_line(mdl, 'act_split2/5', '油门 右 0..1');
sim_add_block(mdl, 'act_track', 'simulink/Sinks/Out1', [1720 1190 1750 1204], 'Port', '12');
sim_connect(mdl, 'act_mux/1', 'act_track/1');

sim_add_block(mdl, 'dr_deg', 'simulink/Sinks/Out1', [2280 880 2310 894], 'Port', '13');
sim_connect(mdl, '混合与分配/3', 'dr_deg/1');
sim_add_block(mdl, 'da_deg', 'simulink/Sinks/Out1', [2340 880 2370 894], 'Port', '14');
sim_connect(mdl, '混合与分配/4', 'da_deg/1');
sim_add_block(mdl, 'de_deg', 'simulink/Sinks/Out1', [2400 880 2430 894], 'Port', '15');
sim_connect(mdl, '混合与分配/5', 'de_deg/1');

%% Canvas scopes
sim_add_block(mdl, 'rate_mux_pq', 'simulink/Signal Routing/Mux', [-30 1507 -5 1600], ...
   'Inputs', '4');
sim_connect(mdl, 'rc_split/1', 'rate_mux_pq/1', 'fb_split/3', 'rate_mux_pq/2', ...
   'rc_split/2', 'rate_mux_pq/3', 'fb_split/4', 'rate_mux_pq/4');
sim_add_block(mdl, 'rate_pq_track', 'simulink/Sinks/Out1', [-30 1630 -5 1644], 'Port', '16');
sim_connect(mdl, 'rate_mux_pq/1', 'rate_pq_track/1');
sim_name_line(mdl, 'rate_mux_pq/1', 'rate_pq 4x1 [p_c p q_c q]');

sim_add_block(mdl, 'rate_mux_r', 'simulink/Signal Routing/Mux', [-30 1690 -5 1750], ...
   'Inputs', '2');
sim_connect(mdl, 'rc_split/3', 'rate_mux_r/1', 'fb_split/5', 'rate_mux_r/2');
sim_add_block(mdl, 'rate_r_track', 'simulink/Sinks/Out1', [-30 1780 -5 1794], 'Port', '17');
sim_connect(mdl, 'rate_mux_r/1', 'rate_r_track/1');
sim_name_line(mdl, 'rate_mux_r/1', 'rate_r 2x1 [r_c r]');

sim_add_block(mdl, 'surf_mux', 'simulink/Signal Routing/Mux', [420 915 445 1000], ...
   'Inputs', '3');
sim_connect(mdl, 'act_split2/1', 'surf_mux/1', 'act_split2/2', 'surf_mux/2', ...
   'act_split2/3', 'surf_mux/3');
sim_add_block(mdl, 'surf_track', 'simulink/Sinks/Out1', [470 950 500 964], 'Port', '18');
sim_connect(mdl, 'surf_mux/1', 'surf_track/1');
sim_name_line(mdl, 'surf_mux/1', 'surf 3x1 [da de dr]');

sim_add_block(mdl, 'thr_mux', 'simulink/Signal Routing/Mux', [420 1030 445 1080], ...
   'Inputs', '2');
sim_connect(mdl, 'act_split2/4', 'thr_mux/1', 'act_split2/5', 'thr_mux/2');
sim_add_block(mdl, 'thr_track', 'simulink/Sinks/Out1', [470 1050 500 1064], 'Port', '19');
sim_connect(mdl, 'thr_mux/1', 'thr_track/1');
sim_name_line(mdl, 'thr_mux/1', 'thr 2x1 [thr_l thr_r]');

sim_hmi('scope', mdl, '姿态跟踪', [1740 1077 2155 1612], {[mdl '/att_mux'], 1});
sim_hmi('scope', mdl, '速率跟踪 pq', [940 1071 1365 1330], {[mdl '/rate_mux_pq'], 1});
sim_hmi('scope', mdl, '速率跟踪 r 方向舵', [940 1345 1365 1600], {[mdl '/rate_mux_r'], 1});
sim_hmi('scope', mdl, '三个舵面', [1350 1071 1764 1330], {[mdl '/surf_mux'], 1});
sim_hmi('scope', mdl, '左右油门', [1350 1345 1764 1600], {[mdl '/thr_mux'], 1});

% fallback classic Scope: three stacked sections, never opens automatically
sim_add_block(mdl, '兜底示波器 三段', 'simulink/Sinks/Scope', [1780 1400 2110 1650]);
set_param([mdl '/兜底示波器 三段'], 'NumInputPorts', '3', ...
   'LayoutDimensionsString', '[3 1]', 'OpenAtSimulationStart', 'off');
sim_connect(mdl, 'att_mux/1', '兜底示波器 三段/1', 'rate_mux/1', '兜底示波器 三段/2', ...
   'act_mux/1', '兜底示波器 三段/3');

%% Signal logging
% logging is set on the output port, not on the line; the names appear in logsout
local_log(mdl, '导航与测量', 3, 'inst_log');
local_log(mdl, 'att_mux', 1, 'att_log');
local_log(mdl, 'rate_mux', 1, 'rate_log');
local_log(mdl, 'act_mux', 1, 'act_log');

info = struct('name', '操纵台与仪表', 'n_controls', 8, 'n_gauges', size(gauge, 1), ...
   'n_states', 3, 'n_scopes', 5, 'n_fallback', 1);
end

%% Local helpers
function local_state(mdl, name, pos, src)
% status readout: a classic Display block on a real line
blk = [mdl '/' name];
add_block('simulink/Sinks/Display', blk, 'Position', pos);
sim_connect(mdl, src, [name '/1']);
end

function local_log(mdl, blk, port, name)
% enable data logging on one output line and name it, so logsout.get can find it
ph = get_param([mdl '/' blk], 'PortHandles');
o = ph.Outport(port);
lh = get_param(o, 'Line');
assert(lh > 0, 'sim_bench_cockpit:NoLine', '%s 的第 %d 路输出没有线，开不了记录', blk, port);
set_param(lh, 'Name', name);
set_param(o, 'DataLogging', 'on');
end
