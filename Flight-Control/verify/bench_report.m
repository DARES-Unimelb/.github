function rep = bench_report(ws)
% BENCH_REPORT 读一次台架飞行的信号记录，打一张表，写一份 results/bench_report.txt。
%
% 用法
%   bench_report             读基础工作区里的 logsout
%   bench_report(name)       name 是记录所在的变量名，默认就是 'logsout'
%
% 台架模型里 inst_log / att_log / rate_log / act_log 四路信号默认开着记录，所以在界面上按
% 一次 Run 之后直接跑这个函数就有数。它只读工作区与模型参数，不改模型。
%
% 打出来的每一节对应画布上的一条图或一块表，末尾给几句结论。结论那一段是给目测用的：
% 速度停住、迎角到顶、跟踪不上，这些现象是设计使然还是真出问题，看数比看表针准。

if nargin < 1 || isempty(ws)
   ws = '';
end
if ~isempty(ws)
   assert(evalin('base', sprintf('exist(''%s'', ''var'')', ws)) == 1, 'bench_report:NoLog', ...
      '基础工作区里没有变量 %s', ws);
   ls = evalin('base', ws);
else
   % 两种落法都认。先取 out：模型把输出写成 out 时它是本轮的新数据；logsout 可能是更早留下
   % 的陈旧副本，先取它会让自动报表描述上一轮，那就比没有报表更坏。
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
   % 门限是弧度：模型里它是跟 |alpha| 的弧度值比的。
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

% ------------------------------------------------------------------------
function s = local_get(ls, name)
% 按名字取一路记录。取不到就返回空，后面的表少一节而不是整个函数报错。
% 记录元素通常是 Simulink.SimulationData.Signal，数据在 .Values 里；元素直接是 timeseries
% 时也认，省得为了换个来源改表。
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
% 从基础工作区取一个参数，取不到就用默认值。报告要能在参数不在场时也打完。
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
% 偏差给两个：全程最大与末段最大。开头那段是阶跃瞬态，只看全程最大值容易被它顶掉。
n = numel(cmd);
k = max(1, n - round(0.25 * n)):n;
fprintf(fid, ['  %-12s 指令末值 %8.2f  实际末值 %8.2f  最大偏差 %8.2f' ...
   '  末段偏差 %8.2f\n'], ...
   name, cmd(end), act(end), max(abs(cmd - act)), max(abs(cmd(k) - act(k))));
end
