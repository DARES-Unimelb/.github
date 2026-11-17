% UACS_JOB_BENCH_REPORT Job entry: fly the bench for a few seconds and print the report.
%
% Run with:  addpath('<repo>/run'); run_job('bench_report')
% Success is the marker === JOB DONE in results/job_bench_report.log.

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
% Diary the whole run: prints and warnings from model callbacks reach no other log.
% Do not name the file job_bench_report.txt, run_job owns that name.
diary(fullfile(root, 'results', 'bench_report_diary.txt'));
diary on;
% load_system runs PreLoadFcn and PostLoadFcn, so the parameter chain need not run again here.
load_system(fullfile(root, 'model', [mdl '.slx']));
fprintf('seed check: k_pitch = %s, sim_stick_pitch = %s\n', ...
   get_param([mdl '/驾驶员/k_pitch'], 'Value'), ...
   num2str(evalin('base', 'sim_stick_pitch'), 17));
fprintf('seed check: k_thr   = %s, sim_stick_thr   = %s\n', ...
   get_param([mdl '/驾驶员/k_thr'], 'Value'), ...
   num2str(evalin('base', 'sim_stick_thr'), 17));
% Move the pitch stick so the report has something to show.
set_param([mdl '/驾驶员/k_pitch'], 'Value', '0.30');
so = sim(mdl, 'StopTime', '12', 'ReturnWorkspaceOutputs', 'on');
% Use a dedicated name: a stale logsout in base would feed the next report old data.
assignin('base', 'bench_job_logsout', so.logsout);
rep = bench_report('bench_job_logsout');
fprintf('report t_end %.2f s, V_end %.3f m/s, alpha_max %.3f deg\n', ...
   rep.t_end, rep.V_end, rep.alpha_max);
set_param([mdl '/驾驶员/k_pitch'], 'Value', '0');
close_system(mdl, 0);
diary off;
fprintf('JOB BODY DONE\n');
