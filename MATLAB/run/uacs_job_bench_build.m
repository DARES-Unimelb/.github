% UACS_JOB_BENCH_BUILD Job entry: build the two interactive models.
%
% Run with:  addpath('C:\Ash\Projects\DARES_UACS\MATLAB\run'); run_job('bench_build')
% Success is the marker === JOB DONE in results/job_bench_build.log.
%
% Why this is a job and not a direct call: save_system asks the SLX part handlers for a
% model thumbnail, and that path reaches Simulink's Live Editor bridge, which fails
% with "This statement is incomplete" when the build is started from an MCP evaluate
% call. The same save succeeds inside run_job, which evaluates the job in the base
% workspace the way the other model-building jobs do.

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
