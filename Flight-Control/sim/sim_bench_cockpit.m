function info = sim_bench_cockpit(mdl, opts)
% SIM_BENCH_COCKPIT 画布上的操纵台、状态读数、仪表与示波器。
%
% 位置参数一律是 Simulink 的 [左 上 右 下]，不是 [x y 宽 高]。写成后者会报
% Simulink:Commands:InvRectDefn。
%
% 操纵台是八个 Dashboard 控件，全部没有端口，一根线都不连，靠绑定工作
%   俯仰杆 滚转杆 方向舵   三个旋钮，绑到驾驶员里三个 Constant 的 Value 参数
%   速度指令               一个滑杆，绑的也是杆位 0 到 1，它给的是空速指令不是油门开度。
%                          指令空速是 V_min + 杆位 * (V_max - V_min)，见 pilot_to_command
%   侧风 东向 mps          一个滑杆，给地系东向加一股常值风，见 sim_bench_wind
%   模式                   拨动开关，0 姿态 1 高度
%   控制器                 旋转开关，0 PID 1 LQR
%   操纵源                 拨动开关，0 虚拟操纵杆 1 游戏手柄
% 三个开关各配一块状态读数，用经典 Display 块接实线。拨动开关与旋转开关本身不显示当前
% 档位，光看块名分不出现在是 PID 还是 LQR。这里不用 Dashboard 的灯：实测三盏灯绑上信号
% 之后运行中全部保持 ColorDefault 的灰色，收不到数据，而读数块走的是端口，一定有数。
%
% 仪表绑到信号上。信号来源块在导航与测量与混合与分配里，它们的输出同时接到顶层的
% Outport 上。这一步不能省：只被仪表"绑定"而不被任何真正的汇使用的信号，会被块化简
% 连同上游块一起删掉，仪表就收不到数据。Outport 是真正的汇。
%
% inst 的次序与 sim_bench_nav 里定死的一致
%   1 V [m/s]   2 h [m]   3 alpha [deg]   4 phi [deg]   5 theta [deg]
%   6 stall 0/1 7 r [deg/s]
% 三个舵面偏度来自混合与分配的第 3、4、5 路输出，是度。
%
% 指令表的意义是把指令与实况并排摆出来，跟踪好坏不用开窗口也看得出来：空速指令表对空速表，
% 滚转指令表对滚转表，俯仰指令表对俯仰表。指令表绑的是制导与模式那一路参考向量的分量，
% 所以在高度模式下俯仰指令表显示的是高度环算出来的那个有效俯仰指令，不是杆位。
%
% 五条跟踪示波器用 DashboardScope，画布内出图，不弹窗。它的 Binding 按官方契约必须是
% 元胞数组，元素是 Simulink.HMI.SignalSpecification，给单个对象会被拒。
% 一块最多画四路，所以速率拆成 pq 与 r 两块、舵面与油门拆成两块。另有一条经典 Scope 兜底：
% DashboardScope 这条路出不来图时，双击它看三段图。它设成不自动弹出。

if nargin < 2
   opts = struct();
end

% ---------------- 操纵台 ----------------
% 绑的都是 owner 那个块的 Value 参数，所以这八个 Constant 的值必须是字面量。
% 落字面量在 sim_build_bench 里做，绑定在之后，顺序不能反。
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

% ---------------- 三块状态读数 ----------------
% 三个子系统各有一路 state 输出，就是那个档位 Constant 本身。
local_state(mdl, '模式读数', [700 350 800 410], '制导与模式/2');
local_state(mdl, '控制器读数', [700 460 800 520], '控制器选择/2');
local_state(mdl, '操纵源读数', [700 570 800 630], '驾驶员/2');

% ---------------- 分接 ----------------
% 反馈向量、参考向量、速率指令向量各自拆开。指令表与跟踪示波器要从整条向量里取分量，
% 而整条向量接进 Mux 会变成一路上重复的信号，出来的不是几条线而是一堆重复通道。
sim_add_block(mdl, 'fb_split', 'simulink/Signal Routing/Demux', [860 1080 900 1360], ...
   'Outputs', '9');
sim_connect(mdl, '导航与测量/1', 'fb_split/1');
sim_add_block(mdl, 'ref_split', 'simulink/Signal Routing/Demux', [860 1420 900 1530], ...
   'Outputs', '5');
sim_connect(mdl, '制导与模式/1', 'ref_split/1');
sim_add_block(mdl, 'rc_split', 'simulink/Signal Routing/Demux', [860 1600 900 1710], ...
   'Outputs', '3');
sim_connect(mdl, 'PID控制器/2', 'rc_split/1');

% inst 七路要落成顶层的七个 Outport。它们本身没人看，作用是当真正的汇：只被仪表"绑定"
% 而不被任何汇使用的信号，会被块化简连同上游块一起删掉，仪表就收不到数据。
sim_add_block(mdl, 'inst_split', 'simulink/Signal Routing/Demux', [760 700 800 960], ...
   'Outputs', '7');
sim_connect(mdl, '导航与测量/3', 'inst_split/1');
names = {'V', 'h', 'alpha_deg', 'phi_deg', 'theta_deg', 'stall', 'r_deg'};
for k = 1:7
   sim_add_block(mdl, names{k}, 'simulink/Sinks/Out1', ...
      [880, 690 + 50 * k, 910, 704 + 50 * k], 'Port', num2str(2 + k));
   sim_connect(mdl, sprintf('inst_split/%d', k), sprintf('%s/1', names{k}));
end

% ---------------- 仪表 ----------------
% 一排十二块，左边空速、右边三个舵面，指令表紧跟在对应的实况表右边。
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

% ---------------- 跟踪通道 ----------------
% 每一路都是"指令与实际"成对进同一个 Mux，示波器上直接看出跟没跟上。
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

% 舵面与油门。先拆成五路把线名写清楚，再合成一路给示波器，这样图例上写的是
% "dr 方向舵 [deg]" 而不是通道号。
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

% 三个舵面偏度的顶层汇，仪表绑的那一路必须真的有东西用，否则会被块化简删掉。
sim_add_block(mdl, 'dr_deg', 'simulink/Sinks/Out1', [2280 880 2310 894], 'Port', '13');
sim_connect(mdl, '混合与分配/3', 'dr_deg/1');
sim_add_block(mdl, 'da_deg', 'simulink/Sinks/Out1', [2340 880 2370 894], 'Port', '14');
sim_connect(mdl, '混合与分配/4', 'da_deg/1');
sim_add_block(mdl, 'de_deg', 'simulink/Sinks/Out1', [2400 880 2430 894], 'Port', '15');
sim_connect(mdl, '混合与分配/5', 'de_deg/1');

% ---------------- 画布内示波器 ----------------
% 一块 DashboardScope 实测最多画四路：参数逐字相同的三块里，绑四路的出图，绑五路与六路的
% 全空。所以按四路一块拆开，宽的那些另起 Mux，宽信号继续留给兜底经典 Scope 与信号记录。
% 窄 Mux 的输出必须各接一个真正的汇：只被仪表"绑定"而不被任何汇使用的信号会被块化简连同
% 上游一起删掉，仪表就收不到数据。Outport 是真正的汇。
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

% 兜底那条：三段上下排，不自动弹窗。
sim_add_block(mdl, '兜底示波器 三段', 'simulink/Sinks/Scope', [1780 1400 2110 1650]);
set_param([mdl '/兜底示波器 三段'], 'NumInputPorts', '3', ...
   'LayoutDimensionsString', '[3 1]', 'OpenAtSimulationStart', 'off');
sim_connect(mdl, 'att_mux/1', '兜底示波器 三段/1', 'rate_mux/1', '兜底示波器 三段/2', ...
   'act_mux/1', '兜底示波器 三段/3');

% ---------------- 数据留痕 ----------------
% 四路信号开信号记录并命名，跑完之后基础工作区里有 logsout。这是设给端口而不是线的参数。
% verify/bench_report.m 读这四个名字打表，把目测到的现象变成数。
local_log(mdl, '导航与测量', 3, 'inst_log');
local_log(mdl, 'att_mux', 1, 'att_log');
local_log(mdl, 'rate_mux', 1, 'rate_log');
local_log(mdl, 'act_mux', 1, 'act_log');

info = struct('name', '操纵台与仪表', 'n_controls', 8, 'n_gauges', size(gauge, 1), ...
   'n_states', 3, 'n_scopes', 5, 'n_fallback', 1);
end

% ------------------------------------------------------------------------
function local_state(mdl, name, pos, src)
% 状态读数：一块经典 Display 接一根实线，显示那个档位 Constant 的数字。
blk = [mdl '/' name];
add_block('simulink/Sinks/Display', blk, 'Position', pos);
sim_connect(mdl, src, [name '/1']);
end

function local_log(mdl, blk, port, name)
% 在某个输出端口那根线上开信号记录并命名。名字不带空格与方括号，logsout.get 好取。
ph = get_param([mdl '/' blk], 'PortHandles');
o = ph.Outport(port);
lh = get_param(o, 'Line');
assert(lh > 0, 'sim_bench_cockpit:NoLine', '%s 的第 %d 路输出没有线，开不了记录', blk, port);
set_param(lh, 'Name', name);
set_param(o, 'DataLogging', 'on');
end
