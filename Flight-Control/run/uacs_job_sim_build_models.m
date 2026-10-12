% UACS_JOB_SIM_BUILD_MODELS Job entry: build, save and census the three models.
% Owner: ash-sim (task-3). Deliberately does NOT simulate: the build has to be
% provable on its own, and a job that builds, saves and simulates at once is
% exactly the shape that hung the shared session on 2026-10-08.
this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_build_models');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);
rep = build_uacs_models();
fprintf('BUILD SUMMARY %s\n', rep.stamp);
fprintf('  trim source %s, lin source %s, pid %s, lqr %s\n', ...
   rep.design.trim_source, rep.design.lin_source, rep.design.pid_source, rep.design.lqr_source);
fprintf('  plant probe %s, deps placeholder %d\n', rep.plant_probe.source, rep.deps.placeholder_used);
for k = 1:numel(rep.models)
   m = rep.models(k);
   fprintf('  %-18s top %3d total %3d lines %3d in %d out %d\n', m.name, ...
      m.top_count, m.total_count, m.line_count, m.inports, m.outports);
end
