% UACS_JOB_BENCH_RECORD_TEST Job entry: 验证 StopFcn 记录钩子在界面式跑法下真的落数据。
%
% Run with:  addpath('C:\Ash\Projects\DARES_UACS\MATLAB\run'); run_job('bench_record_test')
%
% 为什么不能只看代码：StopFcn 执行的时候，仿真输出（out 或 logsout）有没有已经回到基础
% 工作区，是这一步成不成立的关键，而这一点只有真跑一次才知道。这里用 SimulationCommand
% 走界面按 Run 的同一条路，跑完检查两个产物是否生成、流水里写的是哪条来源。
%
% 本作业会改内存里的模型参数（俯仰杆、StopTime），跑完改回去，不保存。

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_record_test');
end
root = fileparts(fileparts(this));
addpath(root, fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);

mdl = 'uacs_flight_bench';
mat = fullfile(root, 'results', 'bench_last_run.mat');
rep = fullfile(root, 'results', 'bench_report.txt');
if exist(mat, 'file')
   delete(mat);
end
load_system(fullfile(root, 'model', [mdl '.slx']));

set_param([mdl '/驾驶员/k_pitch'], 'Value', '0.30');
set_param(mdl, 'StopTime', '8');
t0 = tic;
set_param(mdl, 'SimulationCommand', 'start');
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'running') && toc(t0) < 30
   pause(0.1);
end
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped') && toc(t0) < 300
   pause(0.2);
end
pause(1.5);
fprintf('仿真状态 %s，总用时 %.1f s\n', get_param(mdl, 'SimulationStatus'), toc(t0));

fprintf('bench_last_run.mat 存在 %d\n', exist(mat, 'file') == 2);
if exist(mat, 'file')
   d = dir(mat);
   fprintf('  写于 %s，%d 字节\n', d.date, d.bytes);
end
fprintf('bench_report.txt 存在 %d\n', exist(rep, 'file') == 2);
if exist(rep, 'file')
   d = dir(rep);
   fprintf('  写于 %s，%d 字节\n', d.date, d.bytes);
end
tr = fullfile(root, 'results', 'bench_run_trace.txt');
if exist(tr, 'file')
   t = splitlines(fileread(tr));
   fprintf('--- 流水末尾 ---\n');
   lo = max(1, numel(t) - 6);
   for k = lo:numel(t)
      if ~isempty(t{k})
         fprintf('  %s\n', t{k});
      end
   end
else
   fprintf('流水文件不存在\n');
end

set_param([mdl '/驾驶员/k_pitch'], 'Value', '0');
set_param(mdl, 'StopTime', 'sim_bench_t_end');
close_system(mdl, 0);
fprintf('JOB BODY DONE\n');
