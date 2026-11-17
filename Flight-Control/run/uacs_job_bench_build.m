% UACS_JOB_BENCH_BUILD Job entry: build the two interactive models.
%
% Run with:  addpath('<repo>/run'); run_job('bench_build')
% Success is the marker === JOB DONE in results/job_bench_build.log.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_build');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);
rep = build_bench_models();
fprintf('bench reload callback: %s\n', rep.reload_fcn);
fprintf('JOB BODY DONE\n');
