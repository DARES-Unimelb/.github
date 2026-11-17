% UACS_JOB_BENCH_CASE Job entry: throttle 0 with full and with minimum pitch, to see what
% the elevator is doing.
%
% Run with:  addpath('<repo>/run'); run_job('bench_case')
% Output: results/bench_case.txt
%
% Besides the extremes it counts elevator reversals over the last quarter of the run:
% dozens in steady state mean chatter, a few mean normal recovery.
%
% Model parameters are edited in memory and restored, nothing is saved. The StopFcn
% recording hook is detached first so these runs write no trace lines.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_case');
end
root = fileparts(fileparts(this));
addpath(root, fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);

mdl = 'uacs_flight_bench';
load_system(fullfile(root, 'model', [mdl '.slx']));
stop_fcn = get_param(mdl, 'StopFcn');
set_param(mdl, 'StopFcn', '');

fid = fopen(fullfile(root, 'results', 'bench_case.txt'), 'w', 'n', 'UTF-8');
assert(fid > 0, 'uacs_job_bench_case:NoFile', '写不了 results/bench_case.txt');

name = {'速度指令 0 + 满俯仰', '速度指令 0 + 俯仰到底'};
thr = [0, 0];
pit = [1, -1];
for c = 1:2
   set_param([mdl '/驾驶员/k_thr'], 'Value', num2str(thr(c)));
   set_param([mdl '/驾驶员/k_pitch'], 'Value', num2str(pit(c)));
   so = sim(mdl, 'StopTime', '20', 'ReturnWorkspaceOutputs', 'on');
   ls = so.logsout;
   act = ls.get('act_log');
   att = ls.get('att_log');
   ins = ls.get('inst_log');
   d = act.Values.Data * 180 / pi;
   t = act.Values.Time;
   a = att.Values.Data * 180 / pi;
   g = ins.Values.Data;
   de = d(:, 2);
   fprintf(fid, '\n[%s]  速度指令杆位 %.2f，俯仰杆位 %+.2f，跑 %.1f s\n', ...
      name{c}, thr(c), pit(c), t(end));
   fprintf(fid, '  升降舵 de           最小 %8.2f  最大 %8.2f  末值 %8.2f deg\n', ...
      min(de), max(de), de(end));
   fprintf(fid, '  副翼 da             最小 %8.2f  最大 %8.2f deg\n', min(d(:, 1)), max(d(:, 1)));
   fprintf(fid, '  方向舵 dr 右舵正     最小 %8.2f  最大 %8.2f deg\n', min(d(:, 3)), max(d(:, 3)));
   fprintf(fid, '  左油门             最小 %8.3f  最大 %8.3f  末值 %8.3f\n', ...
      min(act.Values.Data(:, 4)), max(act.Values.Data(:, 4)), act.Values.Data(end, 4));
   fprintf(fid, '  俯仰指令            末值 %8.2f deg\n', a(end, 3));
   fprintf(fid, '  俯仰实际            末值 %8.2f deg，末段偏差 %6.2f deg\n', ...
      a(end, 4), abs(a(end, 3) - a(end, 4)));
   fprintf(fid, '  空速                最小 %8.2f  最大 %8.2f  末值 %8.2f m/s\n', ...
      min(g(:, 1)), max(g(:, 1)), g(end, 1));
   fprintf(fid, '  迎角                最小 %8.2f  最大 %8.2f deg\n', min(g(:, 3)), max(g(:, 3)));
   fprintf(fid, '  高度                最小 %8.2f  最大 %8.2f  末值 %8.2f m\n', ...
      min(g(:, 2)), max(g(:, 2)), g(end, 2));
   % Reversals over the last quarter of the run.
   k0 = find(t >= t(end) * 0.75, 1);
   if isempty(k0)
      k0 = 1;
   end
   seg = de(k0:end);
   dseg = diff(seg);
   dseg(abs(dseg) < 1e-6) = 0;
   flips = sum(diff(sign(dseg)) ~= 0);
   fprintf(fid, '  末段 %.1f s 内 de 换向 %d 次（抖振指示：几十次是抖振，几次是正常回调）\n', ...
      t(end) - t(k0), flips);
end
fclose(fid);

set_param([mdl '/驾驶员/k_thr'], 'Value', 'sim_stick_thr');
set_param([mdl '/驾驶员/k_pitch'], 'Value', 'sim_stick_pitch');
set_param(mdl, 'StopFcn', stop_fcn);
close_system(mdl, 0);
fprintf('result written: %s\n', fullfile(root, 'results', 'bench_case.txt'));
fprintf('JOB BODY DONE\n');
