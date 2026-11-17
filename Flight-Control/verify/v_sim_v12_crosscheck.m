function out = v_sim_v12_crosscheck(varargin)
% V_SIM_V12_CROSSCHECK V12: our 6DOF and the Aerospace Blockset reference agree.
%
%   out = v_sim_v12_crosscheck()
%   out = v_sim_v12_crosscheck('tol', 1e-6)
%
% Reads results/sim_v12_crosscheck.mat, which sim_v12_measure wrote by running
% model/uacs_plant.slx and model/uacs_plant_ref.slx on the same command sequence, and
% compares them state by state. Two things are asserted, and the second is what makes
% the first mean anything
%
%   1. results/sim_v12_calib.mat records that the 6DOF block's output ports were
%      identified by measurement and that the block reproduces the documented equations
%      under an independent RK4; without that, a matching trajectory could be two models
%      wrong in the same way
%   2. the per-state relative difference in the const_density configuration is below
%      'tol'. That configuration evaluates the ISA block at sea level, so both models
%      fly in P.env.rho_sl and what is left is the equations of motion
%
% Both models are the same RK4 on the same right-hand side in two code paths, so the
% difference is floating point round-off and nothing else. The measured value is
% reported next to the limit.
%
% A missing measurement or toolbox raises v_sim_v12:NoData, which v_sim_all reads as a
% skip.

ip = inputParser;
ip.addParameter('tol', 1e-4, @(x) isnumeric(x) && x > 0);
ip.addParameter('verbose', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
verbose = logical(ip.Results.verbose);

root = sim_root();
file = fullfile(root, 'results', 'sim_v12_crosscheck.mat');
assert(exist(file, 'file') == 2, 'v_sim_v12:NoData', ...
   '%s is missing; run run_job(''sim_v12'') first', file);
S = load(file);
M = S.M;

v = ver('aeroblks');
assert(~isempty(v), 'v_sim_v12:NoToolbox', 'the Aerospace Blockset is not installed');
ref_model = fullfile(root, 'model', 'uacs_plant_ref.slx');
% exist with an ABSOLUTE path returns 2 for a file that is there and 4 only when the same
% file is also reachable by NAME, so the presence test must not be == 4
assert(exist(ref_model, 'file') >= 2, 'v_sim_v12:NoModel', ...
   '%s is missing; run sim_build_plant_ref first', ref_model);

calib_file = fullfile(root, 'results', 'sim_v12_calib.mat');
assert(exist(calib_file, 'file') == 2, 'v_sim_v12:Calibration', ...
   '%s is missing, so the port map of the 6DOF block is unverified', calib_file);
Sc = load(calib_file);
% The verdict must come from data this run produced: a cross-check file older than the
% model it judges, or than the calibration it depends on, is not evidence
info_x = dir(file);
info_m = dir(ref_model);
info_c = dir(calib_file);
assert(info_x.datenum >= info_m.datenum, 'v_sim_v12:StaleData', ...
   ['%s is older than %s, so it was measured against a different reference model; ' ...
    'run run_job(''sim_v12'')'], file, ref_model);
assert(info_x.datenum >= info_c.datenum, 'v_sim_v12:StaleData', ...
   ['%s is older than %s, so the port map it rests on was verified afterwards; ' ...
    'run run_job(''sim_v12'')'], file, calib_file);
assert(Sc.C.criterion.ports_identified, 'v_sim_v12:Calibration', ...
   'the 6DOF output ports were not identified by measurement');
assert(Sc.C.criterion.equations_match, 'v_sim_v12:Calibration', ...
   'the 6DOF block does not reproduce the documented equations under an independent RK4');
assert(Sc.C.criterion.forces_identical, 'v_sim_v12:Calibration', ...
   'the reference force block does not reproduce plant/eom_6dof at rho = P.env.rho_sl');
assert(Sc.C.criterion.rho_sl_exact, 'v_sim_v12:Calibration', ...
   ['the ISA density at sea level is not exactly P.env.rho_sl, so the criterion ' ...
    'run would compare two different fluids']);

c = M.cfgs(M.criterion_cfg);
out = struct();
out.file = file;
out.calib_file = calib_file;
out.params_revision = M.params_revision;
out.trim_source = M.trim_source;
out.solver = c.meta_our.solver;
out.fixed_step = M.fixed_step;
out.t_end = M.cfgs(M.criterion_cfg).t(end);
out.n_samples = numel(c.t);
out.interp = c.interp;
out.tol = ip.Results.tol;
out.max_rel = c.max_rel;
out.worst_state = M.state_names{c.worst_state};
out.max_abs = max(c.diff_max_abs);
out.pass = c.max_rel < ip.Results.tol;
out.calib_pass = Sc.C.pass;
out.wind_frame_residual = NaN;
if isfield(M, 'wind_frame')
   out.wind_frame_residual = M.wind_frame.residual;
end
% Carried through for the record only; the criterion is the const_density run
out.max_rel_isa_frozen = NaN;
out.max_rel_isa_state = NaN;
out.max_rel_crosswind = NaN;
if isfield(M, 'max_rel_isa_frozen')
   out.max_rel_isa_frozen = M.max_rel_isa_frozen;
end
if isfield(M, 'max_rel_isa_state')
   out.max_rel_isa_state = M.max_rel_isa_state;
end
if isfield(M, 'max_rel_crosswind')
   out.max_rel_crosswind = M.max_rel_crosswind;
end
out.table = table(M.state_names(:), c.diff_max_abs(:), c.state_ptp(:), ...
   c.rel(:), c.denom(:), 'VariableNames', {'state', 'max_diff', 'ptp', 'rel', 'denom'});
out.configs = struct('name', {}, 'max_rel', {}, 'max_rel_scale', {}, 't_end', {});
for k = 1:numel(M.cfgs)
   out.configs(end + 1) = struct('name', M.cfgs(k).name, 'max_rel', M.cfgs(k).max_rel, ...
      'max_rel_scale', M.cfgs(k).max_rel_scale, 't_end', M.cfgs(k).t(end));
end

if verbose
   fprintf('V12 Aerospace Blockset %s, reference model %s\n', v.Version, ref_model);
   fprintf('V12 %s at %s, %d samples, %.1f s\n', c.meta_our.solver, ...
      Sc.C.trim_source, out.n_samples, out.t_end);
   fprintf('V12 criterion configuration %s: %s\n', c.name, c.desc);
   fprintf('V12 density both models fly in %.12f kg/m^3 (ISA block at h = %.0f m)\n', ...
      M.rho_sl, c.ref_h_isa);
   fprintf('V12 denominator: peak to peak of the uacs_plant trajectory, reference scale for %d\n', ...
      sum(c.denom_small));
   disp(out.table);
   for k = 1:numel(out.configs)
      fprintf('V12 %-14s max rel against the excursion %.6e, against the state scale %.6e, %.1f s\n', ...
         out.configs(k).name, out.configs(k).max_rel, out.configs(k).max_rel_scale, ...
         out.configs(k).t_end);
   end
   if isfield(M, 'wind_frame')
      w = M.wind_frame;
      fprintf(['V12 wind frame: sim_plant_rhs dx - ground frame equation = %s,\n' ...
         '    worst %.3e over all twelve; the term that used to be missing is %s\n'], ...
         mat2str(w.diff', 3), w.residual, mat2str(w.omega_cross_RW', 6));
   end
   fprintf('V12 max rel %.6e at state %s, limit %.3e, calibration %s\n', ...
      out.max_rel, out.worst_state, out.tol, local_v(out.calib_pass));
   fprintf('V12 -> %s\n', local_v(out.pass));
end

assert(Sc.C.pass, 'v_sim_v12:Calibration', ...
   'the Aerospace Blockset calibration did not pass; see %s', ...
   fullfile(root, 'results', 'sim_v12_calib.txt'));
assert(out.pass, 'v_sim_v12:Failed', ...
   ['uacs_plant and uacs_plant_ref differ by %.4e of the trajectory peak to peak at ' ...
    'state %s over %.1f s, limit %.4e'], out.max_rel, out.worst_state, out.t_end, ...
   ip.Results.tol);
end

function s = local_v(tf)
if tf
   s = 'PASS';
else
   s = 'FAIL';
end
end
