function rep = export_flight_control()
% EXPORT_FLIGHT_CONTROL Copy the runnable tree into the Flight-Control directory at the repo root.
%
%   rep = export_flight_control()
%   rep = export_flight_control('dst', '<somewhere>')
%
% Exports params, plant, control, sim, run, verify, sizing, the root .m files, the seven
% models under model/, the design caches in results/ and one English report in docs/.
% Chinese reports, handover notes, report_sources, _scratch, slprj, editor backups and the
% large trajectories and logs are left out.
%
% The target directory is deleted and rebuilt. README.md comes from
% docs/flight_control_readme.md, so edit that file and rerun this function.

ip = inputParser;
ip.addParameter('dst', '', @(x) ischar(x) || isstring(x));
ip.parse();
root = fileparts(fileparts(mfilename('fullpath')));   % MATLAB directory
repo = fileparts(root);                                % repository root
dst = char(ip.Results.dst);
if isempty(dst)
   dst = fullfile(repo, 'Flight-Control');
end

%% Rebuild the target directory
if exist(dst, 'dir') == 7
   rmdir(dst, 's');
end
mkdir(dst);

dirs = {'params', 'plant', 'control', 'sim', 'run', 'verify', 'sizing'};
for k = 1:numel(dirs)
   copyfile(fullfile(root, dirs{k}), fullfile(dst, dirs{k}));
end
copyfile(fullfile(root, '*.m'), dst);
mkdir(fullfile(dst, 'model'));
copyfile(fullfile(root, 'model', '*.slx'), fullfile(dst, 'model'));
mkdir(fullfile(dst, 'results'));
copyfile(fullfile(root, 'results', '*.txt'), fullfile(dst, 'results'));

% Design caches: without them the first model open recomputes trim, linearisation and gains
% in place, which takes minutes. copyfile preserves modification times, which the cache
% freshness test relies on.
keep = {'plant_baseline.mat', 'gains_baseline.mat', 'sim_design_cache.mat', ...
   'plant_stage1_trim.mat', 'plant_stage2_lin.mat', 'control_design_snapshot.mat', ...
   'diag_gains_v3.mat', 'sim_build_census.mat', 'sim_v10_stepsize.mat', ...
   'sim_v12_ref_census.mat', 'sim_v12_calib.mat', 'atmo_regress_check.mat'};
n_keep = 0;
for k = 1:numel(keep)
   f = fullfile(root, 'results', keep{k});
   if exist(f, 'file') > 0
      copyfile(f, fullfile(dst, 'results'));
      n_keep = n_keep + 1;
   end
end

copyfile(fullfile(root, 'docs', 'design_tutorial_report_en.md'), fullfile(dst, 'docs'));
copyfile(fullfile(root, 'docs', 'flight_control_readme.md'), fullfile(dst, 'README.md'));

%% Point the callbacks at the model location
% Both callbacks add the path first, so one press of Run recovers when PreLoadFcn did not run.
boot = ['p=fileparts(fileparts(get_param(bdroot,''FileName'')));' ...
   'addpath(p,fullfile(p,''params''),fullfile(p,''plant''),fullfile(p,''control''),' ...
   'fullfile(p,''sim''),fullfile(p,''run''),fullfile(p,''verify''),''-begin'');'];
n_cb = 0;
n_cb_fail = 0;
for m = {'uacs_flight_bench', 'uacs_flight_bench_en', 'uacs_aero_test'}
   mdl = m{1};
   try
      if bdIsLoaded(mdl)
         close_system(mdl, 0);
      end
      load_system(fullfile(dst, 'model', [mdl '.slx']));
      set_param(mdl, 'PreLoadFcn', [boot 'uacs_go']);
      set_param(mdl, 'InitFcn', [boot 'uacs_ensure']);
      save_system(mdl);
      if bdIsLoaded(mdl)
         close_system(mdl, 0);
      end
      n_cb = n_cb + 1;
   catch me
      % One model failing must not stop the rest.
      n_cb_fail = n_cb_fail + 1;
      fprintf('!! 回调没改成：%s（%s | %s）\n', mdl, me.identifier, me.message);
      if bdIsLoaded(mdl)
         close_system(mdl, 0);
      end
   end
end

%% Remove copied editor and backup files
junk = [dir(fullfile(dst, '**', '*.asv')); dir(fullfile(dst, '**', '*.autosave')); ...
   dir(fullfile(dst, '**', '*.original')); dir(fullfile(dst, 'results', '_*.log'))];
for k = 1:numel(junk)
   delete(fullfile(junk(k).folder, junk(k).name));
end

s = dir(fullfile(dst, '**', '*'));
s = s(~[s.isdir]);
rep = struct('dst', dst, 'files', numel(s), ...
   'bytes', sum([s.bytes]), 'results_kept', n_keep, ...
   'callbacks_patched', n_cb, 'callbacks_failed', n_cb_fail);
fprintf('导出到 %s\n', dst);
fprintf('  %d 个文件，%.1f MB，保留设计缓存 %d 份，回调改成相对定位的模型 %d 个\n', ...
   rep.files, rep.bytes / 1024 / 1024, rep.results_kept, rep.callbacks_patched);
if n_cb_fail > 0
   error('export_flight_control:Callback', ...
      '有 %d 个模型的回调没改成，交付目录不能发出去', n_cb_fail);
end
end
