function uacs_job_sim_v12()
% UACS_JOB_SIM_V12 Job entry: build the Aerospace Blockset reference plant, prove its port
% map by measurement, then cross check the two 6DOF models.
%
% Stages, each logged separately so a failure says which one, and none of them aborted by an
% earlier one:
%   build_ref    build and save model/uacs_plant_ref.slx
%   calibrate    identify the 6DOF output ports and check the block against the
%                documented equations, the ISA block and our own force code
%   crosscheck   run both models on one command sequence, four configurations
%   verify       run verify/v_sim_v12_crosscheck.m, which reads the results back
%                from disk and asserts
%
% The job raises at the end when any stage failed, so the log ends with JOB FAILED and a
% failed criterion is never reported as met.
%
% Deliberately a function with the stages nested inside it: nested functions share this
% function's workspace, so no stage result has to travel through the base workspace.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);

[P, params_src] = sim_init_params();
sim_deps_reset();
sim_plant_source('eom_6dof');
assignin('base', 'sim_ts_ctrl', P.sim.Ts_ctrl);
assignin('base', 'sim_t_end', P.sim.t_end_s);

tr = sim_v12_trim(P);
assignin('base', 'trim', tr);
assignin('base', 'sim_x0', tr.x_star(:));

fprintf('parameters      revision %d, version %s, self check %s\n', ...
   P.meta.revision, P.meta.version, params_src);
fprintf('plant source    %s\n', sim_plant_source());
fprintf('trim            V %.3f m/s, alpha %.4f deg, de %.4f deg, throttle %.4f, |r|max %.3e\n', ...
   tr.V_mps, tr.alpha_deg, tr.de_deg, tr.throttle, tr.residual_norm_max);
fprintf('MATLAB          %s\n', version());
v = ver('aeroblks');
assert(~isempty(v), 'uacs_job_sim_v12:NoToolbox', 'the Aerospace Blockset is not installed');
fprintf('aerospace       %s %s\n', v.Name, v.Version);

build = [];
calib = [];
meas = [];
verify_out = [];
ok1 = sim_stage_run('build_ref', @() stage_build(P));
if ~isempty(build)
   fprintf('built %s: top %d, total %d, lines %d, in %d, out %d\n', build.model, ...
      build.census.top_count, build.census.total_count, build.census.line_count, ...
      build.census.num_inports, build.census.num_outports);
   fprintf('  %s\n', strjoin(build.census.top_names, ' '));
   fprintf('  file %s\n', build.file);
end

ok2 = sim_stage_run('calibrate', @() stage_calib(P));
if ~isempty(calib)
   fprintf('calibration pass %d, criterion %s\n', calib.pass, ...
      mat2str(struct2array(calib.criterion)));
end

ok3 = sim_stage_run('crosscheck', @() stage_measure(P));
if ~isempty(meas)
   for k = 1:numel(meas.cfgs)
      fprintf('%-14s max rel %.6e over %.1f s (%s)\n', meas.cfgs(k).name, ...
         meas.cfgs(k).max_rel, meas.cfgs(k).t(end), meas.cfgs(k).interp);
   end
end

ok4 = sim_stage_run('verify', @() stage_verify());
if ~isempty(verify_out)
   fprintf('verify max rel %.6e at %s, limit %.3e\n', verify_out.max_rel, ...
      verify_out.worst_state, verify_out.tol);
end

ok = ok1 && ok2 && ok3 && ok4;
if ok
   verdict = 'PASS';
else
   verdict = 'FAIL';
end

local_summary(root, P, v, tr, verdict, [ok1, ok2, ok3, ok4], meas);
fprintf('V12 JOB VERDICT %s (build %d, calibrate %d, crosscheck %d, verify %d)\n', ...
   verdict, ok1, ok2, ok3, ok4);
assert(ok, 'uacs_job_sim_v12:Failed', ...
   ['V12 failed; see results/sim_v12_calib.txt, results/sim_v12_crosscheck.mat ' ...
    'and results/sim_v12_report.txt']);

%% Stages, nested so their results cannot be cleared
   function stage_build(P)
      build = sim_build_plant_ref(P);
      save(fullfile(sim_root(), 'results', 'sim_v12_ref_census.mat'), 'build');
   end

   function stage_calib(P)
      calib = sim_ref_calibrate(P, struct('t_end', 2.0));
      if ~calib.pass
         error('uacs_job_sim_v12:Calibration', ...
            'the calibration did not pass, see results/sim_v12_calib.txt');
      end
   end

   function stage_measure(P)
      meas = sim_v12_measure(P);
   end

   function stage_verify()
      verify_out = v_sim_v12_crosscheck();
   end
end

%% Summary
function local_summary(root, P, v, tr, verdict, stage_ok, meas)
file = fullfile(root, 'results', 'sim_v12_report.txt');
fid = fopen(file, 'w');
if fid < 0
   return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'V12 cross validation summary\n');
fprintf(fid, 'generated      %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, 'parameters     revision %d, version %s\n', P.meta.revision, P.meta.version);
fprintf(fid, 'MATLAB         %s\n', version());
fprintf(fid, 'Aerospace      %s %s\n', v.Name, v.Version);
fprintf(fid, 'trim           V %.3f m/s, alpha %.4f deg, de %.4f deg, throttle %.4f\n', ...
   tr.V_mps, tr.alpha_deg, tr.de_deg, tr.throttle);
fprintf(fid, 'stages         build %d, calibrate %d, crosscheck %d, verify %d\n', stage_ok);
fprintf(fid, 'verdict        %s\n', verdict);
for f = {'sim_v12_calib.txt', 'sim_v12_calib.mat', 'sim_v12_crosscheck.mat', ...
      'sim_v12_ref_census.mat'}
   p = fullfile(root, 'results', f{1});
   if exist(p, 'file')
      fprintf(fid, 'artifact       %s\n', p);
   end
end
if ~isempty(meas)
   fprintf(fid, '\n[configurations]\n');
   for k = 1:numel(meas.cfgs)
      fprintf(fid, '%-14s max rel %.6e over %.1f s, %s\n', meas.cfgs(k).name, ...
         meas.cfgs(k).max_rel, meas.cfgs(k).t(end), meas.cfgs(k).desc);
   end
end
end
