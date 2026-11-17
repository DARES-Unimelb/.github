% UACS_JOB_BENCH_REPORT Job entry: fly the bench for a few seconds and print the report.
%
% Run with:  addpath('C:\Ash\Projects\DARES_UACS\MATLAB\run'); run_job('bench_report')
% Success is the marker === JOB DONE in results/job_bench_report.log.
%
% Why this is a job: PreLoadFcn runs uacs_go, and that takes a minute or so; the MCP
% single-call evaluate channel cannot wait for it.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_report');
end
root = fileparts(fileparts(this));
addpath(root, fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);

mdl = 'uacs_flight_bench';
% 全程记一份 diary。作业日志 results/job_<名字>.log 平时只有三行状态，失败时才带错误；
% 模型回调里的打印与警告只有 diary 抓得到。文件名不能叫 job_bench_report.txt，那是
% run_job 自己用的。
diary(fullfile(root, 'results', 'bench_report_diary.txt'));
diary on;
% load_system 就会跑 PreLoadFcn 与 PostLoadFcn，参数链不必在这里再跑一遍。
load_system(fullfile(root, 'model', [mdl '.slx']));
fprintf('seed check: k_pitch = %s, sim_stick_pitch = %s\n', ...
   get_param([mdl '/驾驶员/k_pitch'], 'Value'), ...
   num2str(evalin('base', 'sim_stick_pitch'), 17));
fprintf('seed check: k_thr   = %s, sim_stick_thr   = %s\n', ...
   get_param([mdl '/驾驶员/k_thr'], 'Value'), ...
   num2str(evalin('base', 'sim_stick_thr'), 17));
% 打一次俯仰杆再跑，报告里才有东西可看。
set_param([mdl '/驾驶员/k_pitch'], 'Value', '0.30');
so = sim(mdl, 'StopTime', '12', 'ReturnWorkspaceOutputs', 'on');
% 不要把结果放进基础工作区的 logsout：那个名字是自动记录钩子与 bench_report 的来源之一，
% 留一个陈旧副本会让下一次界面飞行的自动报表读到旧数据。用专属名字显式传进去。
assignin('base', 'bench_job_logsout', so.logsout);
rep = bench_report('bench_job_logsout');
fprintf('report t_end %.2f s, V_end %.3f m/s, alpha_max %.3f deg\n', ...
   rep.t_end, rep.V_end, rep.alpha_max);
set_param([mdl '/驾驶员/k_pitch'], 'Value', '0');
close_system(mdl, 0);
diary off;
fprintf('JOB BODY DONE\n');
