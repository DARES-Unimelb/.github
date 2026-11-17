function rep = bench_live_check()
% BENCH_LIVE_CHECK Check that changing a stick value during a running simulation really
% moves the aircraft, which the set-value-then-sim checks cannot see.
%
%   rep = bench_live_check()
%
% A changes the base variable sim_stick_pitch, B changes the Constant parameter Value.
% Writes results/bench_live_check.txt.

root = fileparts(fileparts(mfilename('fullpath')));
mdl = 'uacs_flight_bench';
report = fullfile(root, 'results', 'bench_live_check.txt');
fid = fopen(report, 'w', 'n', 'UTF-8');
assert(fid > 0, 'bench_live_check:NoFile', '写不了 %s', report);

load_system(fullfile(root, 'model', [mdl '.slx']));
ph = get_param([mdl '/六自由度对象'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on');
set_param(get_param(ph.Outport(1), 'Line'), 'Name', 'x_log');
set_param(mdl, 'StopTime', '4');

k_pitch = [mdl '/驾驶员/k_pitch'];
res = {};

% A: change the base variable during a run
local_reset(mdl, 0);
[a, ta] = local_live(mdl, @() assignin('base', 'sim_stick_pitch', 0.30));
res(end + 1, :) = {a, sprintf('A 运行中改基础工作区变量：末态俯仰角 %.3f deg，干预耗时 %.2f s', a, ta)};

% B: change the literal parameter during a run
local_reset(mdl, 0);
[b, tb] = local_live(mdl, @() set_param(k_pitch, 'Value', '0.30'));
res(end + 1, :) = {b, sprintf('B 运行中改 Constant 字面值参数：末态俯仰角 %.3f deg，干预耗时 %.2f s', b, tb)};

% C: control, change the value and then call sim
local_reset(mdl, 0);
get_param([mdl '/俯仰杆'], 'Binding').setValue(0.30);
c = local_plain(mdl);
get_param([mdl '/俯仰杆'], 'Binding').setValue(0);
res(end + 1, :) = {c, sprintf('C 对照：停跑后改值再 sim：末态俯仰角 %.3f deg', c)};

local_reset(mdl, 0);
set_param(ph.Outport(1), 'DataLogging', 'off');
close_system(mdl, 0);

fprintf(fid, '运行中改操纵量是否生效（%s）\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '俯仰杆给 +0.30，俯仰环约 1 s 跟上，末态俯仰角应明显大于 5 deg。\n');
fprintf(fid, 'A 与 B 都接近 0 说明运行中拖动这条路整体不通；只有一个接近 0 就说明是那条\n');
fprintf(fid, '绑定机制的问题，换另一种即可。\n\n');
ok = {};
for k = 1:size(res, 1)
   tag = res{k, 1};
   if tag > 5
      mark = 'OK  ';
      ok{end + 1} = res{k, 2}; %#ok<AGROW>
   else
      mark = 'FAIL';
   end
   fprintf(fid, '[%s] %s\n', mark, res{k, 2});
end
fclose(fid);
rep = struct();
rep.a_deg = a;
rep.b_deg = b;
rep.c_deg = c;
rep.file = report;
fprintf('运行中检查：A %.3f deg / B %.3f deg / C %.3f deg\n', a, b, c);
fprintf('报告 %s\n', report);
end

% Local helpers
function [th, dt] = local_live(mdl, fn)
% Start a running-mode simulation, intervene after 1.2 s, wait for it to finish and
% read the final pitch angle
% A running-mode simulation shares the thread with the script and only advances
% while the script pauses
set_param(mdl, 'SimulationCommand', 'start');
t0 = tic;
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'running') && toc(t0) < 15
   pause(0.1);
end
pause(1.2);
t1 = tic;
fn();
dt = toc(t1);
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped') && toc(t0) < 180
   pause(0.2);
end
th = local_theta();
end

function th = local_plain(mdl)
so = sim(mdl, 'StopTime', '4', 'ReturnWorkspaceOutputs', 'on');
th = local_read(so);
end

function th = local_theta()
so = evalin('base', 'out');
th = local_read(so);
end

function th = local_read(so)
el = [];
try
   el = so.logsout.get('x_log');
catch
   el = [];
end
assert(~isempty(el), 'bench_live_check:NoLog', '取不到 x_log，运行模式下的输出没落进 base 的 out');
x = el.Values.Data;
if ndims(x) > 2
   x = squeeze(x)';
end
th = x(end, 8) * 180 / pi;
end

function local_reset(mdl, val)
% Reset both paths: the base variable and the Constant parameter expression
assignin('base', 'sim_stick_pitch', val);
set_param([mdl '/驾驶员/k_pitch'], 'Value', 'sim_stick_pitch');
end
