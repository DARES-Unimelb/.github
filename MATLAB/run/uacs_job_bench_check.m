% UACS_JOB_BENCH_CHECK Job entry: run the interactive bench self-check.
%
% Run with:  addpath('C:\Ash\Projects\DARES_UACS\MATLAB\run'); run_job('bench_check')
% Success is the marker === JOB DONE in results/job_bench_check.log.
%
% Why this is a job and not a direct call: the model's PreLoadFcn runs uacs_go, which
% takes about a minute, and the check itself runs four two-second simulations. Neither
% fits in a single MCP evaluate call.

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
