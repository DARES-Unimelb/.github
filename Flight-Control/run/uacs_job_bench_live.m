% UACS_JOB_BENCH_LIVE Job entry: does changing a stick value mid-run take effect?
%
% Run with:  addpath('<repo>/run'); run_job('bench_live')
% Success is the marker === JOB DONE in results/job_bench_live.log. The verdict itself is
% in results/bench_live_check.txt; a negative verdict is a finding, not a job failure.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_live');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
rehash path;
rep = bench_live_check();
fprintf('JOB BODY DONE a=%.3f b=%.3f c=%.3f\n', rep.a_deg, rep.b_deg, rep.c_deg);
