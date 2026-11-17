function uacs_bench_record(mdl)
% UACS_BENCH_RECORD 每次仿真停下来就把这一轮的数据归档，并打一份带名字的表。
%
% 挂在台架模型的 StopFcn 里。用户在界面上按 Run 或 Start，跑完自动留证据，不必记得敲任何
% 命令；下一次要排查时直接读 results/bench_last_run.mat 与 results/bench_report.txt，或者
% 从 results/runs/ 里挑历史轮次。
%
% 归档办法：最近一轮永远叫 bench_last_run.mat 与 bench_report.txt（固定名字，工具好找），
% 同时按时间戳在 results/runs/ 下留一份 bench_<yyyymmdd_HHMMSS>.mat 与同名 .txt。只覆盖
% 固定名字那份，历史轮次不会被下一轮冲掉。runs/ 下只保留最近 keep 轮，超出的按时间戳删旧。
%
% 数据落法有两种：模型把输出写成 out 时取 out.logsout，直接写 logsout 时取 logsout。
% 两个都不在就写一条说明到 results/bench_run_trace.txt。脚本方式跑的仿真（verify 里的自检）
% 走这一支，因为那时 out 还没落回基础工作区。
%
% 全程 try/catch：挂在 StopFcn 上的东西报错，用户会以为自己的仿真失败了，而这只是个记录
% 钩子。它失败只留痕，绝不打断飞行。

if nargin < 1 || isempty(mdl)
   mdl = bdroot;
end
keep = 5;                     % runs/ 下保留最近几轮
root = fileparts(mfilename('fullpath'));
out_dir = fullfile(root, 'results');
run_dir = fullfile(out_dir, 'runs');
trace = fullfile(out_dir, 'bench_run_trace.txt');
stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
tag = datestr(now, 'yyyymmdd_HHMMSS');
fid = fopen(trace, 'a', 'n', 'UTF-8');
try
   ls = [];
   src = '';
   if evalin('base', 'exist(''out'', ''var'')') == 1
      o = evalin('base', 'out');
      if isa(o, 'Simulink.SimulationOutput')
         try
            ls = o.logsout;
            src = 'out.logsout';
         catch
         end
      end
   end
   if isempty(ls) && evalin('base', 'exist(''logsout'', ''var'')') == 1
      ls = evalin('base', 'logsout');
      src = 'logsout';
   end
   if isempty(ls)
      fprintf(fid, '%s  %s  停止时工作区里既没有 out 也没有 logsout，本轮没记\n', stamp, mdl);
   else
      if ~exist(run_dir, 'dir')
         mkdir(run_dir);
      end
      last = fullfile(out_dir, 'bench_last_run.mat');
      arch = fullfile(run_dir, ['bench_' tag '.mat']);
      logsout = ls;
      save(last, 'logsout');
      save(arch, 'logsout');
      fprintf(fid, '%s  %s  已记 %s，归档 %s，来源 %s\n', stamp, mdl, last, arch, src);
      % 顺手把带名字的表也打一份，固定名字与归档各留一份。
      try
         rep = bench_report();
         copyfile(rep.file, fullfile(run_dir, ['bench_' tag '.txt']));
      catch e
         fprintf(fid, '   打表失败：%s | %s\n', e.identifier, e.message);
      end
      % 只留最近 keep 轮。时间戳名字按字典序就是按时间序。
      d = dir(fullfile(run_dir, 'bench_*.mat'));
      if numel(d) > keep
         [~, ix] = sort({d.name});
         for k = 1:(numel(d) - keep)
            old = fullfile(run_dir, d(ix(k)).name);
            delete(old);
            old_txt = strrep(old, '.mat', '.txt');
            if exist(old_txt, 'file')
               delete(old_txt);
            end
         end
         fprintf(fid, '   归档清理：删掉最旧的 %d 轮，保留最近 %d 轮\n', ...
            numel(d) - keep, keep);
      end
   end
catch e
   try
      fprintf(fid, '   记录失败：%s | %s\n', e.identifier, e.message);
   catch
   end
end
if fid > 0
   fclose(fid);
end
end
