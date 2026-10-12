function D = sim_load_design(P, opts)
% SIM_LOAD_DESIGN Trim, linearisation and gains for the Simulink models.
%
%   D = sim_load_design(P)
%   D = sim_load_design(P, struct('refresh', true))
%
%   D.trim, D.lin, D.gains_pid, D.gains_lqr, D.src
%
% Ladder, most authoritative first
%   1. results/plant_baseline.mat and results/gains_baseline.mat, when they are
%      newer than every .m file under params/, plant/ and control/. Those files
%      are the plant and control deliverables, so using them keeps one
%      authoritative trim and gain set instead of a second opinion.
%   2. recompute with trim_cruise, linearize_at, design_pid and design_lqr.
%   3. the placeholder trim, linearisation and gains in this directory, so the
%      Simulink chain stays runnable while a delivered module is broken.
%   4. results/sim_design_cache.mat, keyed by a byte and date fingerprint of the
%      model sources, so repeated Simulink jobs do not repeat a trim solve.
%
% D.src names the branch and the source of each piece. Every result file carries
% it, so a placeholder run is never read as a delivered-plant run.

if nargin < 2
   opts = struct();
end
if ~isfield(opts, 'refresh')
   opts.refresh = false;
end

root = sim_root();
res_dir = fullfile(root, 'results');
if ~exist(res_dir, 'dir')
   mkdir(res_dir);
end
fp = local_fingerprint(root);
D = struct('trim', [], 'lin', [], 'gains_pid', [], 'gains_lqr', [], 'src', struct());
D.src.fingerprint = fp.stamp;
D.src.branch = '';

deliverable = local_try_deliverables(res_dir, fp);
if deliverable.ok && ~opts.refresh
   D.trim = deliverable.trim;
   D.lin = deliverable.lin;
   D.gains_pid = deliverable.gains_pid;
   D.gains_lqr = deliverable.gains_lqr;
   D.src.branch = 'deliverable_mats';
   D.src.trim_source = 'deliverable';
   D.src.lin_source = 'deliverable';
   D.src.gains_source = 'deliverable';
   % pid_source and lqr_source are set here as well as on the computed branch. They
   % used to exist only when the design had to be recomputed, so
   % run/uacs_job_sim_rebuild.m, which reads rep.design.pid_source, threw
   % "Unrecognized field name" whenever the deliverable mats were current. That is
   % the healthy state, so the rebuild job only worked while the design was stale.
   D.src.pid_source = 'deliverable';
   D.src.lqr_source = 'deliverable';
   D.src.detail = deliverable.detail;
   return;
end
D.src.deliverable_note = deliverable.detail;

cache_file = fullfile(res_dir, 'sim_design_cache.mat');
if exist(cache_file, 'file') && ~opts.refresh
   S = load(cache_file);
   if isfield(S, 'fingerprint') && strcmp(S.fingerprint, fp.stamp) && ...
         isfield(S, 'D') && local_complete(S.D)
      D = S.D;
      D.src.branch = [D.src.branch, '+cache'];
      D.src.cache_hit = true;
      return;
   end
end
D.src.cache_hit = false;

% ---------------- 2. compute from the delivered modules ----------------
D.src.branch = 'computed';
if sim_have('trim_cruise')
   try
      D.trim = trim_cruise(P, P.op.cruise);
      D.src.trim_source = 'computed';
   catch err
      D.src.trim_error = sprintf('%s | %s', err.identifier, err.message);
   end
else
   D.src.trim_error = 'plant/trim_cruise.m is not on the path';
end
if isempty(D.trim)
   D.trim = sim_trim_ph(P);
   D.src.trim_source = 'placeholder';
   D.src.branch = [D.src.branch, '+ph_trim'];
end

if sim_have('linearize_at')
   try
      [D.lin, lin_info] = linearize_at(P, D.trim);
      D.src.lin_source = 'computed';
      D.src.lin_info = lin_info;
   catch err
      D.src.lin_error = sprintf('%s | %s', err.identifier, err.message);
   end
else
   D.src.lin_error = 'plant/linearize_at.m is not on the path';
end
if isempty(D.lin)
   [D.lin, lin_info] = sim_lin_ph(P, D.trim);
   D.src.lin_source = 'placeholder';
   D.src.lin_info = lin_info;
   D.src.branch = [D.src.branch, '+ph_lin'];
end

if sim_have('design_pid')
   try
      D.gains_pid = design_pid(P, D.lin);
      D.src.pid_source = 'computed';
   catch err
      D.src.pid_error = sprintf('%s | %s', err.identifier, err.message);
   end
else
   D.src.pid_error = 'control/design_pid.m is not on the path';
end
if isempty(D.gains_pid)
   D.gains_pid = sim_gains_ph(P, D.lin, 'pid', D.trim);
   D.src.pid_source = 'placeholder';
   D.src.branch = [D.src.branch, '+ph_pid'];
end

if sim_have('design_lqr')
   try
      D.gains_lqr = design_lqr(P, D.lin);
      D.src.lqr_source = 'computed';
   catch err
      D.src.lqr_error = sprintf('%s | %s', err.identifier, err.message);
   end
else
   D.src.lqr_error = 'control/design_lqr.m is not on the path';
end
if isempty(D.gains_lqr)
   D.gains_lqr = sim_gains_ph(P, D.lin, 'lqr', D.trim);
   D.src.lqr_source = 'placeholder';
   D.src.branch = [D.src.branch, '+ph_lqr'];
end

fingerprint = fp.stamp;
save(cache_file, 'D', 'fingerprint');
end

%% ---------------- local ----------------
function ok = local_complete(D)
ok = isstruct(D) && isfield(D, 'trim') && ~isempty(D.trim) && ...
   isfield(D, 'lin') && ~isempty(D.lin) && ...
   isfield(D, 'gains_pid') && ~isempty(D.gains_pid) && ...
   isfield(D, 'gains_lqr') && ~isempty(D.gains_lqr);
end

function r = local_try_deliverables(res_dir, fp)
r = struct('ok', false, 'detail', '', 'trim', [], 'lin', [], 'gains_pid', [], 'gains_lqr', []);
plant_mat = fullfile(res_dir, 'plant_baseline.mat');
gains_mat = fullfile(res_dir, 'gains_baseline.mat');
if ~exist(plant_mat, 'file') || ~exist(gains_mat, 'file')
   r.detail = 'one or both deliverable mats are missing';
   return;
end
age_plant = local_age_days(plant_mat);
if age_plant > fp.newest_m_days
   r.detail = sprintf(['plant_baseline.mat is %.3f days old and the newest model ' ...
      'source is %.3f days old, so the mats may predate the sources'], ...
      age_plant, fp.newest_m_days);
   return;
end
try
   Sp = load(plant_mat);
   Sg = load(gains_mat);
catch err
   r.detail = sprintf('load failed: %s', err.message);
   return;
end
if ~isfield(Sp, 'trim') || ~isfield(Sp, 'lin')
   r.detail = sprintf('plant_baseline.mat holds %s', strjoin(fieldnames(Sp)', ','));
   return;
end
if ~isfield(Sg, 'gains_pid') || ~isfield(Sg, 'gains_lqr')
   r.detail = sprintf('gains_baseline.mat holds %s', strjoin(fieldnames(Sg)', ','));
   return;
end
r.trim = Sp.trim;
r.lin = Sp.lin;
r.gains_pid = Sg.gains_pid;
r.gains_lqr = Sg.gains_lqr;
r.ok = true;
r.detail = sprintf('plant_baseline.mat %.3f days, gains_baseline.mat %.3f days', ...
   age_plant, local_age_days(gains_mat));
end

function d = local_age_days(f)
info = dir(f);
if isempty(info)
   d = Inf;
else
   d = now - info.datenum;
end
end

function fp = local_fingerprint(root)
% Byte counts and timestamps of every model source file, so a cache entry is
% used only while the sources that produced it are unchanged.
dirs = {'params', 'plant', 'control'};
items = {};
newest = Inf;
for k = 1:numel(dirs)
   d = dir(fullfile(root, dirs{k}, '*.m'));
   for j = 1:numel(d)
      items{end + 1} = sprintf('%s/%s:%d:%.6f', dirs{k}, d(j).name, ...
         d(j).bytes, d(j).datenum); %#ok<AGROW>
      newest = min(newest, now - d(j).datenum);
   end
end
fp.items = sort(items);
fp.newest_m_days = newest;
fp.stamp = sprintf('%d files | %s', numel(fp.items), strjoin(fp.items, ' | '));
end
