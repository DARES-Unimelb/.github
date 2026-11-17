% UACS_JOB_BENCH_CHECK Job entry: run the interactive bench self-check.
%
% Run with:  addpath('<repo>/run'); run_job('bench_check')
% Success is the marker === JOB DONE in results/job_bench_check.log.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_check');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
rehash path;
rep = bench_check();
n_fail = numel(rep.failed);
if n_fail > 0
   error('uacs_job_bench_check:Failed', '台架自检有 %d 条不通过', n_fail);
end
fprintf('JOB BODY DONE\n');
