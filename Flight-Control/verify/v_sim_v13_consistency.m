function out = v_sim_v13_consistency(varargin)
% V_SIM_V13_CONSISTENCY The built model and the design-time simulation are one controller.
%
%   out = v_sim_v13_consistency()
%
% The controller is delivered twice. control/pid_cascade_sim.m is the linear
% simulation every V6 number in results/control_pid_report.txt comes from;
% model/uacs_manual_pid.slx is what actually flies. Two implementations of one law
% drift apart silently, and before rev 11 they had, in three places:
%
%   theta integral-state clamp   0.174532925 in the model, 0.069813170 in the simulation
%   V     integral-state clamp   5.000000000 in the model, 6.250000000 in the simulation
%   q_c   command clamp          1.167830602 in the model, 80.000000000 in the simulation
%
% The first two are one quantity read two ways, in opposite directions. The third is
% the serious one: the simulation clamped the attitude loop's rate command at ten
% times the loop bandwidth, which never acts, so V6 validated a controller WITHOUT
% the limit that keeps the roll and yaw loops out of inversion, while the model flew
% with it. A 13.5 deg bank command asks the roll loop for 1.40 rad/s against a
% 0.478 rad/s clamp.
%
% Four checks, in increasing strength:
%   1. the model's block parameters NAME the base variables, so nothing is inlined
%   2. the pushed values EQUAL the design fields
%   3. the limits the linear simulation APPLIES equal the same fields
%   4. one bank command through both, from the saved roll_step evidence, peaks at
%      the same commanded roll rate and settles at the same bank
%
% Check 2 alone cannot catch the q_c divergence, because both sides reading a field
% satisfies it. Check 4 catches it: a clamp that acts on one side and not the other
% cannot command the same peak rate.
%
% This case reads saved evidence and the .slx from disk. It runs the linear
% simulation, which is seconds, but never the model, so sim_verify stays an
% independent check of what the scenario jobs produced.

ip = inputParser;
ip.addParameter('tol_rel', 1e-12, @(x) isnumeric(x) && x > 0);
ip.addParameter('clamp_tol_frac', 0.01, @(x) isnumeric(x) && x > 0);
ip.addParameter('phi_tol_deg', 2.0, @(x) isnumeric(x) && x > 0);
ip.addParameter('verbose', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
opt = ip.Results;
verbose = logical(opt.verbose);

root = sim_root();
mdl = 'uacs_manual_pid';
model_file = fullfile(root, 'model', [mdl '.slx']);
traj_file = fullfile(root, 'results', 'sim_manual_pid_traj.mat');
assert(exist(model_file, 'file') == 2, 'v_sim_v13:NoData', ...
   '%s is missing; run the sim_rebuild job first', model_file);
assert(exist(traj_file, 'file') == 2, 'v_sim_v13:NoData', ...
   '%s is missing; run the sim_manual_pid job first', traj_file);

[P_init, params_src] = sim_init_params(); %#ok<ASGLU>
P = evalin('base', 'P');
D = sim_load_design(P);
% pid_cascade_sim takes the contract-order linear model, not the raw linearisation.
lc = lin_contract(D.lin);
sim_push_design_vars(P, D);
if ~bdIsLoaded(mdl)
   load_system(model_file);
end

loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
surf_of = struct('p', 'aileron', 'q', 'elevator', 'r', 'rudder');
out = struct('part', {}, 'quantity', {}, 'model', {}, 'design', {}, 'rel', {}, 'pass', {});

% ---------------- 1. the model names the variables, it does not inline numbers ----------------
for k = 1:numel(loops)
   L = loops{k};
   sub = sprintf('%s/pid_%s', mdl, L);
   out(end + 1) = local_text('names', sprintf('pid_%s/Kp', L), ...
      local_param(sub, 'Kp', 'Gain'), sprintf('sim_pid_%s_Kp_abs', L)); %#ok<AGROW>
   out(end + 1) = local_text('names', sprintf('pid_%s/Ki_gainval', L), ...
      local_param(sub, 'Ki', 'gainval'), sprintf('sim_pid_%s_Ki_abs', L)); %#ok<AGROW>
   out(end + 1) = local_text('names', sprintf('pid_%s/Ki_limit', L), ...
      local_param(sub, 'Ki', 'UpperSaturationLimit'), sprintf('sim_pid_%s_Isat', L)); %#ok<AGROW>
   out(end + 1) = local_text('names', sprintf('pid_%s/pol', L), ...
      local_param(sub, 'pol', 'Gain'), sprintf('sim_pid_%s_pol', L)); %#ok<AGROW>
end
out(end + 1) = local_text('names', 'p_c_lim', ...
   get_param([mdl '/p_c_lim'], 'UpperLimit'), 'sim_rate_cmd_lim_p');
out(end + 1) = local_text('names', 'q_c_lim', ...
   get_param([mdl '/q_c_lim'], 'UpperLimit'), 'sim_rate_cmd_lim_q');
out(end + 1) = local_text('names', 'r_c_lim', ...
   get_param([mdl '/r_c_lim'], 'UpperLimit'), 'sim_rate_cmd_lim_r');

% ---------------- 2. the pushed value equals the design field ----------------
for k = 1:numel(loops)
   L = loops{k};
   g = D.gains_pid.(L);
   out(end + 1) = local_cmp('pushed', sprintf('pid_%s_Kp_abs', L), ...
      local_base(sprintf('sim_pid_%s_Kp_abs', L)), abs(g.Kp), opt.tol_rel); %#ok<AGROW>
   out(end + 1) = local_cmp('pushed', sprintf('pid_%s_Ki_abs', L), ...
      local_base(sprintf('sim_pid_%s_Ki_abs', L)), max(abs(g.Ki), 1e-12), opt.tol_rel); %#ok<AGROW>
   out(end + 1) = local_cmp('pushed', sprintf('pid_%s_pol', L), ...
      local_base(sprintf('sim_pid_%s_pol', L)), local_pol(g.Kp), opt.tol_rel); %#ok<AGROW>
   if isfield(P.ctrl.int_state_limit, L)
      ilim = P.ctrl.int_state_limit.(L);
   else
      ilim = deg2rad(P.act.(surf_of.(L)).max_deg);
   end
   out(end + 1) = local_cmp('pushed', sprintf('pid_%s_Isat', L), ...
      local_base(sprintf('sim_pid_%s_Isat', L)), ilim, opt.tol_rel); %#ok<AGROW>
end
out(end + 1) = local_cmp('pushed', 'rate_cmd_lim_p', ...
   local_base('sim_rate_cmd_lim_p'), P.ctrl.rate_cmd_limit.p_rad_s, opt.tol_rel);
out(end + 1) = local_cmp('pushed', 'rate_cmd_lim_q', ...
   local_base('sim_rate_cmd_lim_q'), P.ctrl.rate_cmd_limit.q_rad_s, opt.tol_rel);
out(end + 1) = local_cmp('pushed', 'rate_cmd_lim_r', ...
   local_base('sim_rate_cmd_lim_r'), P.ctrl.rate_cmd_limit.r_rad_s, opt.tol_rel);

% ---------------- 3. the limits the linear simulation applies ----------------
lin_probe = pid_cascade_sim(P, D.gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', 0, 'V_step', 0, 'phi_step', deg2rad(20), 't_end_s', 8));
lm = lin_probe.meta;
out(end + 1) = local_cmp('applied', 'lin_p_lim', lm.p_lim, ...
   P.ctrl.rate_cmd_limit.p_rad_s, opt.tol_rel);
out(end + 1) = local_cmp('applied', 'lin_q_lim', lm.q_lim, ...
   P.ctrl.rate_cmd_limit.q_rad_s, opt.tol_rel);
out(end + 1) = local_cmp('applied', 'lin_r_lim', lm.r_lim, ...
   P.ctrl.rate_cmd_limit.r_rad_s, opt.tol_rel);
out(end + 1) = local_cmp('applied', 'lin_lim_ith', lm.lim_ith, ...
   P.ctrl.int_state_limit.theta, opt.tol_rel);
out(end + 1) = local_cmp('applied', 'lin_lim_iV', lm.lim_iV, ...
   P.ctrl.int_state_limit.V, opt.tol_rel);
out(end + 1) = local_cmp('applied', 'lin_lim_ih', lm.lim_ih, ...
   P.ctrl.int_state_limit.h, opt.tol_rel);
out(end + 1) = local_cmp('applied', 'lin_lim_iphi', lm.lim_iphi, ...
   P.ctrl.int_state_limit.phi, opt.tol_rel);

% ---------------- 4. one bank command through both, from saved evidence ----------------
S = load(traj_file);
traj = S.traj;
idx = find(strcmp({traj.name}, 'roll_step'), 1);
assert(~isempty(idx), 'v_sim_v13:NoData', 'the roll_step scenario is not in %s', traj_file);
tr = traj(idx);
n = numel(tr.t);
i_tail = max(n - round(0.05 * n), 1):n;
phi_cmd_model = mean(tr.cmd(i_tail, 1));
phi_ss_model = mean(tr.x(i_tail, 7));
p_cmd_model = max(abs(tr.rate_cmd(:, 1)));

% The linear run is driven by the model's OWN settled bank command, so a difference
% in the stick-to-bank mapping cannot be mistaken for a controller difference.
lin = pid_cascade_sim(P, D.gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', 0, 'V_step', 0, 'phi_step', phi_cmd_model, 't_end_s', tr.t(end)));
nl = numel(lin.t);
phi_ss_lin = mean(lin.phi(max(nl - round(0.05 * nl), 1):nl));
p_cmd_lin = max(abs(lin.p_c));

out(end + 1) = local_cmp('behaviour', 'peak_rate_cmd_p', p_cmd_model, ...
   P.ctrl.rate_cmd_limit.p_rad_s, opt.clamp_tol_frac);
out(end + 1) = local_cmp('behaviour', 'peak_rate_cmd_p_linear', p_cmd_lin, ...
   P.ctrl.rate_cmd_limit.p_rad_s, opt.clamp_tol_frac);
out(end + 1) = local_cmp('behaviour', 'settled_phi_deg', rad2deg(phi_ss_model), ...
   rad2deg(phi_ss_lin), opt.phi_tol_deg / rad2deg(abs(phi_ss_lin)));

lines = {};
lines{end + 1} = 'UACS twin: model against design-report consistency (V13)';
lines{end + 1} = sprintf('generated %s by verify/v_sim_v13_consistency.m', ...
   datestr(now, 'yyyy-mm-dd HH:MM:SS'));
lines{end + 1} = sprintf('MATLAB %s, parameter set rev %g, P %s, design from %s', ...
   version('-release'), P.meta.revision, P.meta.version, D.src.branch);
lines{end + 1} = '';
lines{end + 1} = 'part names    the model block parameter, against the base variable it must name';
lines{end + 1} = 'part pushed   the value pushed to that variable, against its design field';
lines{end + 1} = 'part applied  the limit the linear simulation applied, against the same field';
lines{end + 1} = 'part behaviour the saved roll_step evidence, against a linear run on its own command';
lines{end + 1} = 'rel is |model - design| / max(|design|, 1e-12)';
lines{end + 1} = '';
lines{end + 1} = sprintf('%-10s %-24s %30s %30s %11s  %s', ...
   'part', 'quantity', 'model', 'design', 'rel', 'verdict');
for k = 1:numel(out)
   lines{end + 1} = sprintf('%-10s %-24s %30s %30s %11.2e  %s', out(k).part, ...
      out(k).quantity, local_show(out(k).model), local_show(out(k).design), ...
      out(k).rel, local_verdict(out(k).pass)); %#ok<AGROW>
end
lines{end + 1} = '';
lines{end + 1} = sprintf('roll_step: the model settled at %.4f deg and commanded a peak roll rate of %.6f rad/s against the %.6f rad/s clamp', ...
   rad2deg(phi_cmd_model), p_cmd_model, P.ctrl.rate_cmd_limit.p_rad_s);
lines{end + 1} = sprintf('the linear simulation on that same command peaked at %.6f rad/s and settled at %.4f deg against the model''s %.4f deg', ...
   p_cmd_lin, rad2deg(phi_ss_lin), rad2deg(phi_ss_model));
lines{end + 1} = sprintf('V13 verdict: %d of %d comparisons pass', ...
   sum([out.pass]), numel(out));

evidence = fullfile(root, 'results', 'model_report_consistency.txt');
fid = fopen(evidence, 'w');
if fid > 0
   fprintf(fid, '%s\n', lines{:});
   fclose(fid);
end
if verbose
   fprintf('%s\n', lines{:});
end

bad = out(~[out.pass]);
assert(isempty(bad), 'v_sim_v13:Failed', ...
   'model and report disagree on: %s', strjoin({bad.quantity}, ', '));
end

%% ---------------- local ----------------
function v = local_param(sub, block, param)
v = get_param([sub '/' block], param);
end

function e = local_text(part, quantity, model, design)
e = struct('part', part, 'quantity', quantity, 'model', model, 'design', design, ...
   'rel', double(~strcmp(model, design)), 'pass', strcmp(model, design));
end

function e = local_cmp(part, quantity, model, design, tol)
% One numeric comparison, with the relative difference reported whatever the outcome.
% The tolerance is a relative band with an absolute floor, so a quantity that is
% legitimately zero does not divide by zero.
den = max(abs(design), 1e-12);
e = struct('part', part, 'quantity', quantity, 'model', model, 'design', design, ...
   'rel', abs(model - design) / den, 'pass', abs(model - design) / den <= tol);
end

function s = local_show(v)
if ischar(v) || (isstring(v) && isscalar(v))
   s = char(v);
elseif isscalar(v)
   s = sprintf('%.9g', v);
else
   s = mat2str(v);
end
end

function v = local_base(name)
v = evalin('base', name);
end

function p = local_pol(kp)
p = sign(kp);
if p == 0
   p = 1;
end
end

function s = local_verdict(tf)
if tf
   s = 'PASS';
else
   s = 'FAIL';
end
end
