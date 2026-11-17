function rep = bench_report(ws)
% BENCH_REPORT Read one bench run's signal logging and write the table to
% results/bench_report.txt.
%
%   bench_report           read the base workspace
%   bench_report(name)     read the variable called name, default 'logsout'
%
% inst_log, att_log, rate_log and act_log are logged by default, so one Run in the
% model is enough. Workspace and model parameters are only read, never written.

if nargin < 1 || isempty(ws)
   ws = '';
end
if ~isempty(ws)
   assert(evalin('base', sprintf('exist(''%s'', ''var'')', ws)) == 1, 'bench_report:NoLog', ...
      '基础工作区里没有变量 %s', ws);
   ls = evalin('base', ws);
else
   % Prefer out: a leftover logsout may describe an earlier run
   ls = [];
   if evalin('base', 'exist(''out'', ''var'')') == 1
      o = evalin('base', 'out');
      if isa(o, 'Simulink.SimulationOutput')
         ls = o.logsout;
      end
   end
   if isempty(ls) && evalin('base', 'exist(''logsout'', ''var'')') == 1
      ls = evalin('base', 'logsout');
   end
   assert(~isempty(ls), 'bench_report:NoLog', ...
      ['基础工作区里既没有 out.logsout 也没有 logsout。先在界面上按一次 Run，' ...
       '模型停止时会自己记一份，也可以读 results/bench_last_run.mat']);
end

inst = local_get(ls, 'inst_log');
att = local_get(ls, 'att_log');
rate = local_get(ls, 'rate_log');
act = local_get(ls, 'act_log');
assert(~isempty(inst), 'bench_report:NoInst', ...
   'logsout 里没有 inst_log，模型可能是旧版建的，重跑 run/build_bench_models.m');

root = fileparts(fileparts(mfilename('fullpath')));
fid = fopen(fullfile(root, 'results', 'bench_report.txt'), 'w', 'n', 'UTF-8');
assert(fid > 0, 'bench_report:NoFile', '写不了 results/bench_report.txt');

local_sec(fid, '空速与高度');
t = inst.t;
V = inst.v(:, 1);
h = inst.v(:, 2);
local_row(fid, '空速 V', V, 'm/s');
local_row(fid, '高度 h', h, 'm');
k = t >= t(end) - 3;
spread = max(V(k)) - min(V(k));
fprintf(fid, '  末 3 秒空速变化幅度        %8.3f m/s\n', spread);
Vmin = local_par('P.limits.V_min_mps', NaN);
Vmax = local_par('P.limits.V_max_mps', NaN);
if isfinite(Vmin)
   fprintf(fid, '  速度指令下限 V_min         %8.3f m/s\n', Vmin);
end
if isfinite(Vmax)
   fprintf(fid, '  速度指令上限 V_max         %8.3f m/s\n', Vmax);
end

local_sec(fid, '迎角与失速');
al = inst.v(:, 3);
local_row(fid, '迎角 alpha', al, 'deg');
res = local_par('sim_alpha_stall_abs', NaN);
if isfinite(res)
   % The threshold is in rad: the model compares it against |alpha| in rad
   fprintf(fid, '  失速迎角门限               %8.3f rad（%.2f deg）\n', res, res * 180 / pi);
end
if size(inst.v, 2) >= 6
   fprintf(fid, '  失速标志出现次数           %8d 次\n', sum(inst.v(:, 6) > 0.5));
end

local_sec(fid, '姿态跟踪（指令与实际，度）');
if ~isempty(att)
   d = att.v * 180 / pi;
   local_pair(fid, '滚转 phi', d(:, 1), d(:, 2));
   local_pair(fid, '俯仰 theta', d(:, 3), d(:, 4));
else
   fprintf(fid, '  没有 att_log，跳过\n');
end

local_sec(fid, '速率跟踪（指令与实际，度每秒，只有 PID 模式有指令）');
if ~isempty(rate)
   d = rate.v * 180 / pi;
   local_pair(fid, '滚转率 p', d(:, 1), d(:, 2));
   local_pair(fid, '俯仰率 q', d(:, 3), d(:, 4));
   local_pair(fid, '偏航率 r', d(:, 5), d(:, 6));
else
   fprintf(fid, '  没有 rate_log，跳过\n');
end

local_sec(fid, '舵面与油门');
if ~isempty(act)
   d = act.v(:, 1:3) * 180 / pi;
   local_row(fid, '副翼 da', d(:, 1), 'deg');
   local_row(fid, '升降舵 de', d(:, 2), 'deg');
   local_row(fid, '方向舵 dr（右舵正）', d(:, 3), 'deg');
   local_row(fid, '油门 左', act.v(:, 4), '0..1');
   local_row(fid, '油门 右', act.v(:, 5), '0..1');
else
   fprintf(fid, '  没有 act_log，跳过\n');
end

local_sec(fid, '结论');
if spread < 0.3
   fprintf(fid, '  空速在末段是平的，幅度 %.3f m/s。\n', spread);
   if isfinite(Vmin) && abs(V(end) - Vmin) < 0.8
      fprintf(fid, ['  末速 %.2f m/s 等于速度指令下限 V_min，这是速度环按指令把速度按住，' ...
         '不是失速，也不是操纵没反应。\n'], V(end));
   end
else
   fprintf(fid, '  空速在末段还在变，幅度 %.3f m/s，说明还没到稳态。\n', spread);
end
fprintf(fid, '  空速范围 %.2f 到 %.2f m/s，迎角范围 %.2f 到 %.2f deg。\n', ...
   min(V), max(V), min(al), max(al));
if ~isempty(att)
   d = att.v * 180 / pi;
   fprintf(fid, '  姿态最大跟踪偏差：滚转 %.2f deg，俯仰 %.2f deg。\n', ...
      max(abs(d(:, 1) - d(:, 2))), max(abs(d(:, 3) - d(:, 4))));
end

fclose(fid);
rep = struct();
rep.file = fullfile(root, 'results', 'bench_report.txt');
rep.t_end = t(end);
rep.V_end = V(end);
rep.alpha_max = max(al);
fprintf('\n刚才这份表也写进了 %s\n', rep.file);
end

% Local helpers
function s = local_get(ls, name)
% Look a signal up by name; return empty when it is absent so one missing section
% does not abort the table
s = [];
try
   el = ls.get(name);
catch
   return
end
if isempty(el)
   return
end
if isa(el, 'timeseries')
   tv = el.Time;
   dv = el.Data;
else
   tv = el.Values.Time;
   dv = el.Values.Data;
end
if ndims(dv) > 2
   dv = squeeze(dv)';
end
s = struct('t', tv, 'v', dv);
end

function val = local_par(name, dflt)
% Read a parameter from the base workspace; the default stands in when it is absent
val = dflt;
try
   val = evalin('base', name);
catch
end
end

function local_sec(fid, txt)
fprintf(fid, '\n[%s]\n', txt);
end

function local_row(fid, name, v, unit)
fprintf(fid, '  %-22s 最小 %9.3f  最大 %9.3f  末值 %9.3f  %s\n', ...
   name, min(v), max(v), v(end), unit);
end

function local_pair(fid, name, cmd, act)
% Two deviations: the whole run, and the last quarter without the step transient
n = numel(cmd);
k = max(1, n - round(0.25 * n)):n;
fprintf(fid, ['  %-12s 指令末值 %8.2f  实际末值 %8.2f  最大偏差 %8.2f' ...
   '  末段偏差 %8.2f\n'], ...
   name, cmd(end), act(end), max(abs(cmd - act)), max(abs(cmd(k) - act(k))));
end
