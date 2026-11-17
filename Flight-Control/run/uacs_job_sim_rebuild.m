% UACS_JOB_SIM_REBUILD Job entry: rebuild the three models after a control change.
%
% Run with:  addpath('<repo>/run'); run_job('sim_rebuild')
% Success is the marker === JOB DONE in results/job_sim_rebuild.log.
%
% Builds only, no simulation. Reads the gains from results/plant_baseline.mat and
% results/gains_baseline.mat when they are current, and recomputes them from plant/ and
% control/ otherwise.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_sim_rebuild');
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
fprintf('  trim de %.4f deg, alpha %.4f deg, throttle %.4f\n', ...
   rep.trim_de_deg, rep.trim_alpha_deg, rep.trim_throttle);
for k = 1:numel(rep.models)
   m = rep.models(k);
   fprintf('  %-18s top %3d total %3d lines %3d in %d out %d\n', m.name, ...
      m.top_count, m.total_count, m.line_count, m.inports, m.outports);
end
% Print the polarity the models carry, one line per loop, so a sign that did not reach
% the .slx shows up in the job log.
loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
for k = 1:numel(loops)
   fprintf('  polarity sim_pid_%s_pol = %+d\n', loops{k}, ...
      evalin('base', sprintf('sim_pid_%s_pol', loops{k})));
end
v = v_sim_v11_models();
fprintf('V11 after rebuild: %d models checked\n', numel(v));
fprintf('JOB BODY DONE\n');
