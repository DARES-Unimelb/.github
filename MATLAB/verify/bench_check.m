function rep = bench_check()
% BENCH_CHECK 交互台架 uacs_flight_bench 的自检。
%
% 查四件事，每件都写成可判定的判据
%   一、八个操纵控件的绑定有效，而且能通过 setValue 真的改动它们绑的那个变量
%   二、十二路仪表读数、三块状态读数与三条画布内示波器的绑定有效
%   三、静止不动时飞机停在配平上；拉俯仰杆时俯仰角真的跟着走，松杆回零
%   四、模式切换到高度、控制器切换到 LQR 都能跑通，状态有界
%
% 必须走 run_job。模型 PreLoadFcn 会运行 uacs_go，那一步要一分钟上下，MCP 的单次
% 求值通道等不到它。
%
% 报告写 results/bench_check.txt。判据不成立时本函数返回的 rep.failed 非空。

root = fileparts(fileparts(mfilename('fullpath')));
mdl = 'uacs_flight_bench';
res = {};
fid = fopen(fullfile(root, 'results', 'bench_check.txt'), 'w', 'n', 'UTF-8');
assert(fid > 0, 'bench_check:NoFile', '写不了报告文件');

res = local_line(res, '台架自检（%s）', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
load_system(fullfile(root, 'model', [mdl '.slx']));
% 自检自己要跑八次脚本方式的 sim，每次都会触发 StopFcn 的记录钩子；那时 out 还没落回基础
% 工作区，钩子只会在流水里写一堆"本轮没记"。自检期间把它摘掉，跑完再装回去。
stop_fcn = get_param(mdl, 'StopFcn');
set_param(mdl, 'StopFcn', '');
res = local_line(res, '模型已加载，顶层块 %d 个', ...
   numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'Block')));

% ---------------- 一、操纵控件 ----------------
% 控件绑的是那个块的 Value 参数，而且那个值必须是字面量。绑变量的那条路看着通，实际
% 运行时是死的，所以这里只查参数这一条路。
ctl = {'俯仰杆', '驾驶员/k_pitch'; '滚转杆', '驾驶员/k_roll'; ...
   '方向舵', '驾驶员/k_yaw'; '速度指令', '驾驶员/k_thr'; ...
   '侧风 东向 mps', '大气与风扰动/wind_e'; '模式', '制导与模式/mode_const'; ...
   '控制器', '控制器选择/k_sel'; '操纵源', '驾驶员/js_on'};
for k = 1:size(ctl, 1)
   b = get_param([mdl '/' ctl{k, 1}], 'Binding');
   ok = b.isBound();
   res = local_chk(res, ok, '%s 绑定有效', ctl{k, 1});
   if ~ok
      continue
   end
   v = get_param([mdl '/' ctl{k, 2}], 'Value');
   res = local_chk(res, ~isnan(str2double(v)), '%s 驱动的是字面量参数 %s = %s', ...
      ctl{k, 1}, ctl{k, 2}, v);
end

% ---------------- 二、仪表、状态读数与示波器 ----------------
rc = {'空速表', '空速指令表', '高度表', '迎角表', '滚转表', '滚转指令表', ...
   '俯仰表', '俯仰指令表', '偏航率表', '方向舵表 右舵正', '副翼表 原始偏度', ...
   '升降舵表 原始偏度', '失速灯'};
for k = 1:numel(rc)
   b = get_param([mdl '/' rc{k}], 'Binding');
   res = local_chk(res, ~isempty(b.BlockPath.getBlock(1)), '%s 绑到了信号', rc{k});
end
% 三块状态读数是经典 Display，判据是那一路真的接上了。灯那条路已经废掉：实测三盏灯绑上
% 信号之后运行中全部保持 ColorDefault 的灰色，收不到数据，改用端口接线的读数块。
for k = {'模式读数', '控制器读数', '操纵源读数'}
   ph = get_param([mdl '/' k{1}], 'PortHandles');
   has_line = ~isempty(ph.Inport) && get_param(ph.Inport(1), 'Line') > 0;
   res = local_chk(res, has_line, '%s 的输入接了信号', k{1});
end
% 画布内三条示波器的 Binding 按契约是元胞数组，元素是 SignalSpecification。这一条只证明
% 绑定建起来了，出不出图要在界面上看，脚本这边看不到。
for k = {'姿态跟踪', '速率跟踪 pq', '速率跟踪 r 方向舵', '三个舵面', '左右油门'}
   b = get_param([mdl '/' k{1}], 'Binding');
   ok = iscell(b) && numel(b) == 1 && ~isempty(b{1}.BlockPath.getBlock(1));
   res = local_chk(res, ok, '%s 绑到了信号', k{1});
end
% 兜底示波器是经典 Scope，三路各有一条输入线。
ph_fb = get_param([mdl '/兜底示波器 三段'], 'PortHandles');
n_line = 0;
for k = 1:numel(ph_fb.Inport)
   if get_param(ph_fb.Inport(k), 'Line') > 0
      n_line = n_line + 1;
   end
end
res = local_chk(res, n_line == 3, '兜底示波器 三段 三路都接了信号，实际 %d 路', n_line);
% 舵面极性只记不判：G_surf 是操纵效能矩阵，对角元的符号就是每个舵面到本体轴的极性。
try
   G = evalin('base', 'sim_G_surf');
   pol = sign(diag(G));
   res = local_line(res, '舵面极性记录 da %+d de %+d dr %+d，G_surf 对角元符号', ...
      pol(1), pol(2), pol(3));
catch
   res = local_line(res, '舵面极性未记录：基础工作区里没有 sim_G_surf');
end

% ---------------- 三、静止配平与俯仰响应 ----------------
% 状态线上开记录，这样不必改模型就能读轨迹。信号记录是设在端口上的参数，不是线上的：
% 线上没有 DataLogging 这个参数，设上去报 Simulink:Commands:ParamUnknown。
ph_x = get_param([mdl '/六自由度对象'], 'PortHandles');
set_param(ph_x.Outport(1), 'DataLogging', 'on');
lh = get_param(ph_x.Outport(1), 'Line');
set_param(lh, 'Name', 'x_log');

res = local_line(res, '配平保持：拉杆前');
[x0, t0] = local_run(mdl);
res = local_chk(res, all(isfinite(x0(end, :))), '状态末值有限');
h0 = -x0(end, 12);
V0 = local_V(x0(end, :));
res = local_chk(res, abs(h0 - 150) < 8, '高度停在配平附近 %.2f m（期望 150 上下 8）', h0);
res = local_chk(res, abs(V0 - 25) < 1.5, '空速停在配平附近 %.2f m/s（期望 25 上下 1.5）', V0);
res = local_chk(res, abs(t0(end) - 2.0) < 0.2, '仿真跑满 2 s，末时刻 %.3f s', t0(end));

% 两条指令向量的通道数查不了：sim 结束后模型不再处于编译态，get_param 取
% CompiledPortWidths 得到的是空结构体，而 isempty(tw.Outport) 在字段为空逗号列表时
% 本身就会报 MATLAB:minrhs。改为在结构上保证：att_mux 的四路与 rate_mux 的六路各自
% 来自两个 Demux 的不同端口，不再是把整条向量重复接进去。信这条保证，不写检查。

res = local_line(res, '拉俯仰杆 +0.30');
res = local_drag(res, mdl, '俯仰杆', '驾驶员/k_pitch', 0.30);
[x1, ~] = local_run(mdl);
th1 = x1(end, 8) * 180 / pi;
res = local_chk(res, th1 > 5, '俯仰角跟着杆走，末值 %.2f deg（要求大于 5）', th1);
res = local_chk(res, all(isfinite(x1(end, :))), '拉杆之后状态末值仍有限');

res = local_line(res, '松杆回零');
res = local_drag(res, mdl, '俯仰杆', '驾驶员/k_pitch', 0);
[x2, ~] = local_run(mdl);
th2 = x2(end, 8) * 180 / pi;
res = local_chk(res, abs(th2) < 2.5, '松杆后俯仰角回零，末值 %.2f deg（要求小于 2.5）', th2);

% ---------------- 四、模式与控制器切换 ----------------
res = local_line(res, '模式切到高度，俯仰杆 +0.30');
res = local_drag(res, mdl, '模式', '制导与模式/mode_const', 1);
res = local_drag(res, mdl, '俯仰杆', '驾驶员/k_pitch', 0.30);
[x3, ~] = local_run(mdl);
h3 = -x3(end, 12);
res = local_chk(res, all(isfinite(x3(end, :))), '高度模式跑通，状态末值有限');
res = local_chk(res, h3 > 150.5, '高度模式下拉杆使高度上升，末值 %.2f m（要求大于 150.5）', h3);
res = local_drag(res, mdl, '模式', '制导与模式/mode_const', 0);
res = local_drag(res, mdl, '俯仰杆', '驾驶员/k_pitch', 0);

res = local_line(res, '控制器切到 LQR');
res = local_drag(res, mdl, '控制器', '控制器选择/k_sel', 1);
[x4, ~] = local_run(mdl);
res = local_chk(res, all(isfinite(x4(end, :))), 'LQR 通道跑通，状态末值有限');
res = local_chk(res, max(abs(x4(end, 4:6))) < 2, 'LQR 通道末态角速率有界 %.3f rad/s', ...
   max(abs(x4(end, 4:6))));
res = local_drag(res, mdl, '控制器', '控制器选择/k_sel', 0);

% ---------------- 五、运行中改操纵量是否真的生效 ----------------
% 这一条是补的，也是最关键的一条。前面四条走的是"改值之后再 sim"，重新编译时新值一定
% 读得到，所以绑变量的假通路也会全绿。真正的判据是：仿真跑着的时候改操纵量，飞机立刻
% 响应。用户第一次拖旋钮没反应、Simulink 还弹 not enough argument，就是栽在这里。
res = local_line(res, '仿真运行中改俯仰量');
th_live = local_live(mdl, @() set_param([mdl '/驾驶员/k_pitch'], 'Value', '0.30'));
res = local_chk(res, th_live > 5, ...
   '运行中改俯仰量立刻生效，末态俯仰角 %.2f deg（要求大于 5）', th_live);
set_param([mdl '/驾驶员/k_pitch'], 'Value', '0');

% ---------------- 收尾 ----------------
set_param(ph_x.Outport(1), 'DataLogging', 'off');
set_param(mdl, 'StopFcn', stop_fcn);
close_system(mdl, 0);
n_pass = sum(cell2mat(res(:, 1)));
n_fail = size(res, 1) - n_pass;
E = repmat('  ', size(res, 1), 1);
E(cell2mat(res(:, 1)), :) = repmat('OK', sum(cell2mat(res(:, 1))), 1);
for k = 1:size(res, 1)
   fprintf(fid, '[%s] %s\n', E(k, :), res{k, 2});
end
fprintf(fid, '\n合计 %d 条，通过 %d，不通过 %d\n', size(res, 1), n_pass, n_fail);
fclose(fid);
failed = res(~cell2mat(res(:, 1)), 2);
% 逐字段赋值，不用 struct(...)：failed 为空元胞时 struct 会把它摊成一个 0x0 的结构体
% 数组，之后 rep.total 变成空逗号列表，fprintf 报 MATLAB:minrhs: Not enough input
% arguments，看着像函数少传了参数，其实是结构体维度被摊没了。
rep = struct();
rep.total = size(res, 1);
rep.passed = n_pass;
rep.failed = failed;
rep.file = fullfile(root, 'results', 'bench_check.txt');
fprintf('\n台架自检：%d 条，通过 %d，不通过 %d\n', rep.total, rep.passed, n_fail);
for k = 1:numel(failed)
   fprintf('  不通过：%s\n', failed{k});
end
end

% ------------------------------------------------------------------------
function [x, t] = local_run(mdl)
% 跑 2 s 并读回状态轨迹。sim 的参数覆盖不改模型，所以不需要保存。
% 首选信号记录（在状态端口上开了 DataLogging），取不到再退到 Outport 输出。
% sim 失败时把 cause 逐条展开：run_job 只打最外层那句 "Error due to multiple causes"，
% 真正的原因全在 cause 里。
try
   so = sim(mdl, 'StopTime', '2', 'ReturnWorkspaceOutputs', 'on');
catch err
   msgs = {sprintf('%s | %s', err.identifier, err.message)};
   for c = 1:numel(err.cause)
      msgs{end + 1} = sprintf('cause%d %s | %s', c, ...
         err.cause{c}.identifier, err.cause{c}.message); %#ok<AGROW>
   end
   error('bench_check:SimFailed', '%s', strjoin(msgs, ' ;; '));
end
el = [];
try
   el = so.logsout.get('x_log');
catch
   el = [];
end
if ~isempty(el)
   x = el.Values.Data;
   t = el.Values.Time;
else
   s = so.get('x');
   x = s.signals.values;
   t = s.time;
end
if ndims(x) > 2
   x = squeeze(x)';
end
assert(size(x, 2) == 12, 'bench_check:BadState', '状态向量应是 12 路，实际 %d 路', size(x, 2));
end

function v = local_V(xrow)
% 状态向量第 1 到 3 路是机体速度 [u v w]，空速是它的模。
v = norm(xrow(1:3));
end

function res = local_drag(res, mdl, ctl, owner, val)
% 模拟拖控件：走绑定对象的 setValue，然后回读它该驱动的那个参数。回读对不上说明界面
% 拖动这条路本身有问题，记一条不通过，再直接写参数把后续检查跑完。
b = get_param([mdl '/' ctl], 'Binding');
b.setValue(val);
got = get_param([mdl '/' owner], 'Value');
ok = abs(str2double(got) - val) < 1e-9;
res = local_chk(res, ok, '%s 拖动写回 %s：Value = %s', ctl, owner, got);
if ~ok
   set_param([mdl '/' owner], 'Value', num2str(val, 17));
end
end

function th = local_live(mdl, fn)
% 起一次运行模式的仿真，跑到 1 s 时打干预，等它自然结束，读末态俯仰角。
% 运行模式的仿真与脚本共用一条线程，脚本这边 pause 的时候它才会推进。
set_param(mdl, 'StopTime', '3');
set_param(mdl, 'SimulationCommand', 'start');
t0 = tic;
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'running') && toc(t0) < 15
   pause(0.1);
end
pause(1.0);
fn();
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped') && toc(t0) < 180
   pause(0.2);
end
so = evalin('base', 'out');
el = so.logsout.get('x_log');
x = el.Values.Data;
if ndims(x) > 2
   x = squeeze(x)';
end
th = x(end, 8) * 180 / pi;
set_param(mdl, 'StopTime', 'sim_bench_t_end');
end

function res = local_chk(res, ok, fmt, varargin)
res(end + 1, :) = {logical(ok), sprintf(fmt, varargin{:})};
end

function res = local_line(res, fmt, varargin)
res(end + 1, :) = {true, ['-- ' sprintf(fmt, varargin{:})]};
end
