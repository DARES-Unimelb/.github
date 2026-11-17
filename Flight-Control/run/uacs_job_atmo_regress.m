% UACS_JOB_ATMO_REGRESS Before and after comparison of the rev 9 and rev 10 plant.
%
% Job for run_job('atmo_regress'). The rev 9 baseline is the copy
% results/atmo_before_plant_baseline.mat, taken before the density and wind frame edits;
% the rev 10 baseline is results/plant_baseline.mat. Both came from the same plant baseline
% job, so a difference between them is a difference the edits made and nothing else.
%
% Compares the three trims, the linearisation, the modes, the resolved limits, the envelope
% numbers and every numeric leaf of P, then writes results/atmo_regress_report.txt.
%
% Manual run: run_job('atmo_regress', 'owner', 'ash-atmo') with run/ on the MATLAB path.

here = fileparts(mfilename('fullpath'));
if isempty(here)
   here = fileparts(which('uacs_job_atmo_regress'));
end
if isempty(here)
   here = pwd;
end
root = fileparts(here);
addpath(fullfile(root, 'run'));
addpath(fullfile(root, 'plant'), fullfile(root, 'params'), fullfile(root, 'sizing'), '-begin');
res_dir = fullfile(root, 'results');
stamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
pass_words = {'FAIL', 'PASS'};
pass_text = @(tf) pass_words{double(tf) + 1};

f_before = fullfile(res_dir, 'atmo_before_plant_baseline.mat');
f_after = fullfile(res_dir, 'plant_baseline.mat');
assert(exist(f_before, 'file') == 2, 'atmo_regress:NoBefore', ...
   '%s is missing, it is the rev 9 baseline copy', f_before);
assert(exist(f_after, 'file') == 2, 'atmo_regress:NoAfter', ...
   '%s is missing, run run_job(''plant_baseline'') first', f_after);

Sa = load(f_before);
Sb = load(f_after);
fid = fopen(fullfile(res_dir, 'atmo_regress_report.txt'), 'w');
fprintf(fid, 'UACS twin: rev 9 against rev 10 plant baseline, before and after\n');
fprintf(fid, 'generated %s, MATLAB %s\n', stamp, version());
fprintf(fid, 'before %s, params rev %d\n', f_before, Sa.P.meta.revision);
fprintf(fid, 'after  %s, params rev %d\n', f_after, Sb.P.meta.revision);
fprintf(fid, 'criterion: every rev 9 design number must be identical, not merely close\n');

R = struct();
R.rev_before = Sa.P.meta.revision;
R.rev_after = Sb.P.meta.revision;

%% Trims
fprintf(fid, '\n[trims]\n');
fprintf(fid, '  %-8s %-16s %18s %18s %12s\n', 'point', 'quantity', 'rev 9', 'rev 10', 'difference');
trim_names = {'cruise', 'climb', 'slow'};
trim_vars = {'trim', 'trim_climb', 'trim_slow'};
R.trim = struct();
for k = 1:numel(trim_vars)
   ta = Sa.(trim_vars{k});
   tb = Sb.(trim_vars{k});
   q = {'alpha_deg', 'theta_deg', 'de_deg', 'throttle', 'residual_norm_max', 'qbar', 'CL', 'CD', 'rho'};
   for j = 1:numel(q)
      d = abs(ta.(q{j}) - tb.(q{j}));
      R.trim.(trim_names{k}).(q{j}) = d;
      fprintf(fid, '  %-8s %-16s %18.12f %18.12f %12.3e\n', trim_names{k}, q{j}, ...
         ta.(q{j}), tb.(q{j}), d);
   end
   R.trim.(trim_names{k}).x_star_max = max(abs(ta.x_star - tb.x_star));
   R.trim.(trim_names{k}).u_star_max = max(abs(ta.u_star - tb.u_star));
   R.trim.(trim_names{k}).residual_max = max(abs(ta.residual - tb.residual));
   fprintf(fid, '  %-8s %-16s %18s %18s %12.3e\n', trim_names{k}, 'x_star max', '', '', ...
      R.trim.(trim_names{k}).x_star_max);
   fprintf(fid, '  %-8s %-16s %18s %18s %12.3e\n', trim_names{k}, 'u_star max', '', '', ...
      R.trim.(trim_names{k}).u_star_max);
   fprintf(fid, '  %-8s %-16s %18s %18s %12.3e\n', trim_names{k}, 'residual max', '', '', ...
      R.trim.(trim_names{k}).residual_max);
end

%% Limits and envelope
fprintf(fid, '\n[limits and envelope]\n');
lim = {'V_stall_mps', 'V_min_stall_mps', 'V_min_trim_mps', 'V_min_mps', ...
   'V_max_mps', 'V_top_thrust_limited_mps'};
R.limits = struct();
for j = 1:numel(lim)
   a = Sa.P.limits.(lim{j});
   b = Sb.P.limits.(lim{j});
   R.limits.(lim{j}) = abs(a - b);
   fprintf(fid, '  %-24s rev 9 %14.9f   rev 10 %14.9f   difference %10.3e\n', ...
      lim{j}, a, b, abs(a - b));
end
% Single engine control speed and the ratio that sets it, recomputed per parameter set the
% way init_uacs_twin does.
Cn_dr_full_a = abs(Sa.P.aero.Cn_dr) * deg2rad(Sa.P.act.rudder.max_deg);
Cn_dr_full_b = abs(Sb.P.aero.Cn_dr) * deg2rad(Sb.P.act.rudder.max_deg);
V_mc_a = sqrt(2 * Sa.P.prop.T_static_max * Sa.P.prop.y_motor / ...
   (Sa.P.env.rho_sl * Sa.P.geom.S * Sa.P.geom.b * Cn_dr_full_a));
V_mc_b = sqrt(2 * Sb.P.prop.T_static_max * Sb.P.prop.y_motor / ...
   (Sb.P.env.rho_sl * Sb.P.geom.S * Sb.P.geom.b * Cn_dr_full_b));
R.V_mc_before = V_mc_a;
R.V_mc_after = V_mc_b;
fprintf(fid, '  %-24s rev 9 %14.9f   rev 10 %14.9f   difference %10.3e\n', ...
   'V_mc_mps (recomputed)', V_mc_a, V_mc_b, abs(V_mc_a - V_mc_b));
% Static margin, the same expression init_uacs_twin asserts on.
SM_a = -Sa.P.aero.Cm_alpha / Sa.P.aero.CL_alpha - (Sa.P.mass.x_cg - Sa.P.geom.x_ac) / Sa.P.geom.cbar;
SM_b = -Sb.P.aero.Cm_alpha / Sb.P.aero.CL_alpha - (Sb.P.mass.x_cg - Sb.P.geom.x_ac) / Sb.P.geom.cbar;
R.SM_before = SM_a;
R.SM_after = SM_b;
fprintf(fid, '  %-24s rev 9 %14.9f   rev 10 %14.9f   difference %10.3e\n', ...
   'static margin (fraction)', SM_a, SM_b, abs(SM_a - SM_b));
% Trim envelope scan.
R.V_trim_min_before = Sa.sweep.V_trim_min_mps;
R.V_trim_min_after = Sb.sweep.V_trim_min_mps;
fprintf(fid, '  %-24s rev 9 %14.9f   rev 10 %14.9f   difference %10.3e\n', ...
   'sweep V_trim_min_mps', Sa.sweep.V_trim_min_mps, Sb.sweep.V_trim_min_mps, ...
   abs(Sa.sweep.V_trim_min_mps - Sb.sweep.V_trim_min_mps));

%% Linearisation and modes
fprintf(fid, '\n[linearisation and modes]\n');
R.A_max = max(abs(Sa.lin.A(:) - Sb.lin.A(:)));
R.B_max = max(abs(Sa.lin.B(:) - Sb.lin.B(:)));
R.eig_max = max(abs(sort(Sa.crit.V3_max_real_eig) - sort(Sb.crit.V3_max_real_eig)));
fprintf(fid, '  max|A rev 10 - A rev 9| %.3e\n', R.A_max);
fprintf(fid, '  max|B rev 10 - B rev 9| %.3e\n', R.B_max);
R.modes = struct();
for j = 1:numel(Sa.lin.modes_long)
   a = Sa.lin.modes_long(j);
   b = Sb.lin.modes_long(j);
   R.modes.(a.name) = abs(a.lambda - b.lambda);
   fprintf(fid, '  longitudinal %-14s lambda rev 9 %+.9f%+.9fi, rev 10 %+.9f%+.9fi, difference %.3e\n', ...
      a.name, real(a.lambda), imag(a.lambda), real(b.lambda), imag(b.lambda), abs(a.lambda - b.lambda));
end
for j = 1:numel(Sa.lin.modes_lat)
   a = Sa.lin.modes_lat(j);
   b = Sb.lin.modes_lat(j);
   R.modes.(a.name) = abs(a.lambda - b.lambda);
   fprintf(fid, '  lateral      %-14s lambda rev 9 %+.9f%+.9fi, rev 10 %+.9f%+.9fi, difference %.3e\n', ...
      a.name, real(a.lambda), imag(a.lambda), real(b.lambda), imag(b.lambda), abs(a.lambda - b.lambda));
end

%% Every numeric leaf of P
fprintf(fid, '\n[parameter struct]\n');
diffs = local_diff(Sa.P, Sb.P, 'P', {});
R.n_param_diffs = numel(diffs);
R.changed_paths = cellfun(@(c) c{1}, diffs, 'UniformOutput', false);
% Only the revision number and the fields the density and wind work added may differ;
% anything else is a design number that moved.
expected = {'P.meta.revision', 'P.env.T_sl', 'P.env.p_sl', 'P.env.L_isa', ...
   'P.env.R_air', 'P.env.gamma_air', 'P.env.h_trop_m', 'P.atmo.model', ...
   'P.atmo.h_from_state'};
R.param_diff_expected = numel(diffs) == numel(expected) ...
   && all(ismember(expected, R.changed_paths)) && all(ismember(R.changed_paths, expected));
fprintf(fid, '  %d leaves differ between the two parameter sets, all of them accounted for %d\n', ...
   numel(diffs), R.param_diff_expected);
for k = 1:numel(diffs)
   fprintf(fid, '    %-40s rev 9 %-24s rev 10 %s\n', diffs{k}{1}, ...
      local_str(diffs{k}{2}), local_str(diffs{k}{3}));
end
R.param_diffs = diffs;

%% Verdict
tol_exact = 0;
names = fieldnames(R.trim);
R.max_trim_diff = 0;
for k = 1:numel(names)
   for q = {'x_star_max', 'u_star_max', 'residual_max'}
      R.max_trim_diff = max(R.max_trim_diff, R.trim.(names{k}).(q{1}));
   end
end
R.max_limit_diff = 0;
lim_names = fieldnames(R.limits);
for k = 1:numel(lim_names)
   R.max_limit_diff = max(R.max_limit_diff, R.limits.(lim_names{k}));
end
R.max_mode_diff = 0;
mode_names = fieldnames(R.modes);
for k = 1:numel(mode_names)
   R.max_mode_diff = max(R.max_mode_diff, R.modes.(mode_names{k}));
end
R.pass = R.max_trim_diff <= tol_exact && R.max_limit_diff <= tol_exact ...
   && R.max_mode_diff <= tol_exact && R.A_max <= tol_exact && R.B_max <= tol_exact ...
   && R.V_mc_before == R.V_mc_after && R.SM_before == R.SM_after ...
   && R.V_trim_min_before == R.V_trim_min_after && R.param_diff_expected ...
   && R.rev_before == 9 && R.rev_after == 10;

fprintf(fid, '\n[verdict]\n');
fprintf(fid, '  trims identical            %s (max difference %.3e)\n', ...
   pass_text(R.max_trim_diff <= tol_exact), R.max_trim_diff);
fprintf(fid, '  limits identical           %s (max difference %.3e)\n', ...
   pass_text(R.max_limit_diff <= tol_exact), R.max_limit_diff);
fprintf(fid, '  V_min_trim_mps             %.12f against %.12f\n', ...
   Sa.P.limits.V_min_trim_mps, Sb.P.limits.V_min_trim_mps);
fprintf(fid, '  V_mc_mps                   %.12f against %.12f\n', R.V_mc_before, R.V_mc_after);
fprintf(fid, '  static margin              %.12f against %.12f\n', R.SM_before, R.SM_after);
fprintf(fid, '  modes identical            %s (max difference %.3e)\n', ...
   pass_text(R.max_mode_diff <= tol_exact), R.max_mode_diff);
fprintf(fid, '  A and B identical          %s (%.3e, %.3e)\n', ...
   pass_text(R.A_max <= tol_exact && R.B_max <= tol_exact), R.A_max, R.B_max);
fprintf(fid, '  params rev %d -> %d, %d changed leaves, overall %s\n', ...
   R.rev_before, R.rev_after, R.n_param_diffs, pass_text(R.pass));
fclose(fid);

save(fullfile(res_dir, 'atmo_regress_check.mat'), 'R', 'stamp');
fprintf('[atmo_regress] trims %.3e, limits %.3e, modes %.3e, A %.3e, B %.3e\n', ...
   R.max_trim_diff, R.max_limit_diff, R.max_mode_diff, R.A_max, R.B_max);
fprintf('[atmo_regress] V_min_trim %.12f -> %.12f, V_mc %.9f -> %.9f, SM %.9f -> %.9f\n', ...
   Sa.P.limits.V_min_trim_mps, Sb.P.limits.V_min_trim_mps, R.V_mc_before, R.V_mc_after, ...
   R.SM_before, R.SM_after);
fprintf('[atmo_regress] overall %s\n', pass_text(R.pass));
if ~R.pass
   error('atmo_regress:Failed', 'a rev 9 number moved, see results/atmo_regress_report.txt');
end

%% Local functions
function out = local_diff(a, b, path, out)
% Collect every leaf that differs between two structs, recursively. out is returned as well
% as taken in: a cell array grown inside a function does not reach the caller otherwise.
if isstruct(a) && isstruct(b)
   fa = fieldnames(a);
   fb = fieldnames(b);
   for k = 1:numel(fa)
      if ~isfield(b, fa{k})
         out{end + 1} = {[path '.' fa{k}], 'present', 'absent'}; %#ok<AGROW>
      else
         out = local_diff(a.(fa{k}), b.(fa{k}), [path '.' fa{k}], out);
      end
   end
   for k = 1:numel(fb)
      if ~isfield(a, fb{k})
         out{end + 1} = {[path '.' fb{k}], 'absent', 'present'}; %#ok<AGROW>
      end
   end
   return;
end
if iscell(a) || iscell(b)
   if ~isequal(size(a), size(b)) || ~isequal(a, b)
      out{end + 1} = {path, local_str(a), local_str(b)};
   end
   return;
end
if ischar(a) || ischar(b)
   if ~strcmp(a, b)
      out{end + 1} = {path, a, b};
   end
   return;
end
if isnumeric(a) && isnumeric(b) && isequal(size(a), size(b))
   if ~isequal(a, b)
      out{end + 1} = {path, mat2str(a(:)', 6), mat2str(b(:)', 6)};
   end
   return;
end
if ~isequal(a, b)
   out{end + 1} = {path, local_str(a), local_str(b)};
end
end

function s = local_str(v)
if ischar(v)
   s = v;
elseif iscell(v)
   s = sprintf('cell %s', mat2str(size(v)));
elseif isnumeric(v) && isscalar(v)
   s = sprintf('%.12g', v);
elseif isnumeric(v)
   s = mat2str(v(:)', 6);
else
   s = class(v);
end
end
