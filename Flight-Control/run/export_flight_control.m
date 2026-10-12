function rep = export_flight_control()
% EXPORT_FLIGHT_CONTROL 把可运行的整套东西导出成仓库根下的 Flight-Control 目录。
%
%   rep = export_flight_control()
%
% 为什么要有个导出动作而不是手工复制：交付目录必须是「别人克隆下来就能跑」的，而模型里
% 的 PreLoadFcn 与 InitFcn 原来写的是 run('<绝对路径>\uacs_go.m')，换台机器就断，按 Run
% 时报 Unrecognized function or variable 'uacs_ensure'。导出时把三个交互模型的回调改成从
% .slx 自己的位置往上两级定位工程根，这条只能程序化做，手工做迟早漏。
%
% 导出内容
%   <仓库根>/Flight-Control/README.md    使用说明，源件是 docs/flight_control_readme.md
%   <仓库根>/Flight-Control/model/*.slx  七个模型，三个交互模型的回调已改成相对定位
%   <仓库根>/Flight-Control/{params,plant,control,sim,run,verify,sizing}/  代码
%   <仓库根>/Flight-Control/*.m          uacs_go 等根级函数
%   <仓库根>/Flight-Control/results/     设计缓存与各份报告，不含大轨迹与日志
%   <仓库根>/Flight-Control/docs/        中英文报告与各类设计说明
%
% 不导出：slprj、*.asv、*.autosave、*.original、results 下的大 .mat 与 _*.log、
% docs/_scratch（联网调研的临时件）。
%
% 本函数会先删掉目标目录再重建。README.md 的正文不在这里，改说明改
% docs/flight_control_readme.md，重跑本函数即可。

ip = inputParser;
ip.addParameter('dst', '', @(x) ischar(x) || isstring(x));
ip.parse();
root = fileparts(fileparts(mfilename('fullpath')));   % MATLAB 目录
repo = fileparts(root);                                % 仓库根
dst = char(ip.Results.dst);
if isempty(dst)
   dst = fullfile(repo, 'Flight-Control');
end

% ---------------- 重建目标目录 ----------------
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

% 设计缓存：没有它们，第一次打开模型会就地重算配平、线性化与增益，要几分钟且要多一个工具箱。
% copyfile 保留修改时间，那份缓存的时间戳判据（比 params/plant/control 下所有 .m 都新）
% 才不会因为复制而失效。
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

copyfile(fullfile(root, 'docs'), fullfile(dst, 'docs'));
if exist(fullfile(dst, 'docs', '_scratch'), 'dir') == 7
   rmdir(fullfile(dst, 'docs', '_scratch'), 's');
end
copyfile(fullfile(root, 'docs', 'flight_control_readme.md'), fullfile(dst, 'README.md'));

% ---------------- 回调改成相对定位 ----------------
% PreLoadFcn 与 InitFcn 都先补路径再干活，所以 PreLoadFcn 没跑成时按一次 Run 也能站起来。
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
      % 单个模型失败不能把后面的连坐掉：实测有一次 uacs_aero_test 在这一步抛错，
      % 结果它带着旧的绝对路径留在交付目录里，前两个已经改好了。
      n_cb_fail = n_cb_fail + 1;
      fprintf('!! 回调没改成：%s（%s | %s）\n', mdl, me.identifier, me.message);
      if bdIsLoaded(mdl)
         close_system(mdl, 0);
      end
   end
end

% ---------------- 清掉复制过来的编辑器与备份件 ----------------
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
