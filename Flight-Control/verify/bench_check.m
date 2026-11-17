function rep = bench_check()
% BENCH_CHECK Self checks for the interactive bench model uacs_flight_bench.
%
%   rep = bench_check()
%
% Call it through run_job: the model PreLoadFcn runs uacs_go and takes about a minute.
% Writes results/bench_check.txt; rep.failed lists the criteria that did not hold.

root = fileparts(fileparts(mfilename('fullpath')));
mdl = 'uacs_flight_bench';
res = {};
fid = fopen(fullfile(root, 'results', 'bench_check.txt'), 'w', 'n', 'UTF-8');
assert(fid > 0, 'bench_check:NoFile', '写不了报告文件');

res = local_line(res, '台架自检（%s）', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
load_system(fullfile(root, 'model', [mdl '.slx']));
stop_fcn = get_param(mdl, 'StopFcn');
set_param(mdl, 'StopFcn', '');
res = local_line(res, '模型已加载，顶层块 %d 个', ...
   numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'Block')));

% Stick controls
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

% Instruments, status readouts and scopes
rc = {'空速表', '空速指令表', '高度表', '迎角表', '滚转表', '滚转指令表', ...
   '俯仰表', '俯仰指令表', '偏航率表', '方向舵表 右舵正', '副翼表 原始偏度', ...
   '升降舵表 原始偏度', '失速灯'};
for k = 1:numel(rc)
   b = get_param([mdl '/' rc{k}], 'Binding');
   res = local_chk(res, ~isempty(b.BlockPath.getBlock(1)), '%s 绑到了信号', rc{k});
end
% Classic Display blocks: the input port must be wired
for k = {'模式读数', '控制器读数', '操纵源读数'}
   ph = get_param([mdl '/' k{1}], 'PortHandles');
   has_line = ~isempty(ph.Inport) && get_param(ph.Inport(1), 'Line') > 0;
   res = local_chk(res, has_line, '%s 的输入接了信号', k{1});
end
% Binding only: whether a scope actually draws is not visible from a script
for k = {'姿态跟踪', '速率跟踪 pq', '速率跟踪 r 方向舵', '三个舵面', '左右油门'}
   b = get_param([mdl '/' k{1}], 'Binding');
   ok = iscell(b) && numel(b) == 1 && ~isempty(b{1}.BlockPath.getBlock(1));
   res = local_chk(res, ok, '%s 绑到了信号', k{1});
end
% Classic Scope, one input line per channel
ph_fb = get_param([mdl '/兜底示波器 三段'], 'PortHandles');
n_line = 0;
for k = 1:numel(ph_fb.Inport)
   if get_param(ph_fb.Inport(k), 'Line') > 0
      n_line = n_line + 1;
   end
end
res = local_chk(res, n_line == 3, '兜底示波器 三段 三路都接了信号，实际 %d 路', n_line);
% Recorded, not judged: the sign of each diagonal entry of G_surf is the polarity
try
   G = evalin('base', 'sim_G_surf');
   pol = sign(diag(G));
   res = local_line(res, '舵面极性记录 da %+d de %+d dr %+d，G_surf 对角元符号', ...
      pol(1), pol(2), pol(3));
catch
   res = local_line(res, '舵面极性未记录：基础工作区里没有 sim_G_surf');
end

% Trim hold and pitch response
% Logging is switched on at the port: DataLogging is a port parameter, not a line
% parameter, so setting it on the line raises Simulink:Commands:ParamUnknown
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

% Mode and controller switching
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

% Changing a stick value while the simulation runs, which is a different path from
% setting the value and then calling sim
res = local_line(res, '仿真运行中改俯仰量');
th_live = local_live(mdl, @() set_param([mdl '/驾驶员/k_pitch'], 'Value', '0.30'));
res = local_chk(res, th_live > 5, ...
   '运行中改俯仰量立刻生效，末态俯仰角 %.2f deg（要求大于 5）', th_live);
set_param([mdl '/驾驶员/k_pitch'], 'Value', '0');

% Teardown
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
% Assign field by field: struct() flattens an empty failed cell into a 0x0 struct array
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

% Local helpers
function [x, t] = local_run(mdl)
% Run 2 s and read the state trajectory back; signal logging first, Outport output
% as fallback
% Expand err.cause: sim reports only the outer "Error due to multiple causes" line
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
% States 1 to 3 are body velocity [u v w]; airspeed is their norm
v = norm(xrow(1:3));
end

function res = local_drag(res, mdl, ctl, owner, val)
% Drag the control through its binding, then read the driven parameter back; on a
% mismatch write the parameter directly so the later checks still run
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
% Start a running-mode simulation, intervene after 1 s, wait for it to finish and
% read the final pitch angle
% A running-mode simulation shares the thread with the script and only advances
% while the script pauses
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
