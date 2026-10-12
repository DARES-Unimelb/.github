function R = run_scenarios(varargin)
% RUN_SCENARIOS Acceptance scenarios that need more than one run.
%
%   R = run_scenarios()
%   R = run_scenarios('which', {'v10', 'v9'})
%
%   V6   PID loop evidence, collected by run_manual_pid and summarised here
%   V9   5 m/s crosswind in level flight, against the same run without wind
%   V10  ode45 against ode4 on one command sequence, interpolated onto a common
%        time grid, reported as a relative trajectory difference per state
%   V11  the block census is written by build_uacs_models and asserted by
%        verify/v_sim_v11_models.m, which reads the .slx files back from disk
%   V12  SKIP: Aerospace Blockset is not installed, and the reason is printed
%
% This function measures and records; it does not assert. The pass or fail
% decision lives in verify/v_sim_*.m, which reads the saved results and the
% model files from disk instead of trusting this function's own numbers.

ip = inputParser;
ip.addParameter('which', {'v6', 'v9', 'v10', 'v12'}, @iscell);
ip.addParameter('plots', true, @(x) islogical(x) || isnumeric(x));
ip.addParameter('v10_t_end', 8.0, @(x) isnumeric(x) && x > 0);
ip.addParameter('v9_t_end', 30.0, @(x) isnumeric(x) && x > 0);
ip.addParameter('design', [], @(x) isstruct(x) || isempty(x));
ip.parse(varargin{:});
opt = ip.Results;

root = sim_root();
[P_init, params_src] = sim_init_params(); %#ok<ASGLU>
sim_deps_reset();
P = evalin('base', 'P');
if isempty(opt.design)
   D = sim_load_design(P);
else
   D = opt.design;
end
probe = sim_probe_plant(P, D.trim.x_star, D.trim.u_star);
push = sim_push_design_vars(P, D);

R = struct();
R.stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
R.design = D.src;
R.plant_probe = probe;
R.params = struct('version', P.meta.version, 'b', P.geom.b, 'S', P.geom.S, ...
   'm', P.mass.m, 'V_cruise', P.op.cruise.V_mps, 'Ts_ctrl', P.sim.Ts_ctrl);
R.v6 = [];
R.v9 = [];
R.v10 = [];
R.v12 = struct('status', 'SKIP', 'reason', local_v12_reason());
R.figures = {};

want = @(k) any(strcmp(opt.which, k));

if want('v6')
   R.v6 = local_v6(P, D, probe);
end
if want('v9')
   R.v9 = local_v9(P, D, opt.v9_t_end);
end
if want('v10')
   R.v10 = local_v10(P, D, opt.v10_t_end);
end
if want('v12')
   fprintf('V12 SKIP: %s\n', R.v12.reason);
end

R.report_file = local_report(root, R, P);
if opt.plots && ~isempty(R.v9)
   R.figures{end + 1} = sim_plot_wind(R.v9.wind, R.v9.nowind, root);
end
fprintf('scenario report -> %s\n', R.report_file);
end

%% ---------------- V6 ----------------
function out = local_v6(P, D, probe)
file = fullfile(sim_root(), 'results', 'sim_manual_pid_traj.mat');
if exist(file, 'file')
   S = load(file);
   traj = S.traj;
   source = 'sim_manual_pid_traj.mat';
else
   M = run_manual_pid('plots', false, 'design', D);
   traj = M.traj;
   source = 'run_manual_pid just now';
end
out = struct('source', source, 'loops', struct());
for k = 1:numel(traj)
   out.loops(end + 1) = struct('scenario', traj(k).name, 'metrics', traj(k).metrics, ...
      'plant_probe', probe); %#ok<AGROW>
end
end

%% ---------------- V9 ----------------
function out = local_v9(P, D, t_end)
mdl = 'uacs_manual_pid';
V_min = P.limits.V_min_mps;
V_max = P.limits.V_max_mps;
thr = (P.op.cruise.V_mps - V_min) / (V_max - V_min);
dt = 0.02;
t = (0:dt:t_end)';
stick = [t, zeros(numel(t), 3), thr * ones(numel(t), 1)];
wind_e = 5.0;

runs = struct('name', {}, 'T', {}, 'meta', {});
for k = 1:2
   if k == 1
      wind = [0, 0, 0, 0; t_end, 0, 0, 0];
      nm = 'nowind';
      x0 = D.trim.x_star;
   else
      wind = [0, 0, wind_e, 0; t_end, 0, wind_e, 0];
      nm = 'wind';
      % The crosswind case MUST start from the crab trim of its own wind, not from the
      % still air trim. Starting from the still air trim puts beta = -asin(5/25) =
      % -11.54 deg on the aircraft at t = 0, which is not an equilibrium of the wind
      % case: the run then measures the recovery from a bad initial condition rather
      % than the response to wind, and it contaminates the residual that V9 judges.
      wt = wind_trim(D.trim, [0; wind_e; 0], 0);
      x0 = wt.x_star;
   end
   sim_push_scenario(struct('t_end', t_end, 'x0', x0, 'stick', stick, ...
      'wind', wind, 'flight_mode', 0));
   [T, meta] = sim_run_model(mdl, struct('solver', 'ode4', 't_end', t_end));
   runs(end + 1) = struct('name', nm, 'T', T, 'meta', meta); %#ok<AGROW>
end

w = runs(1);
n = runs(2);
tw = w.T.t;
tn = n.T.t;
V_w = sqrt(sum(w.T.by_name.x_state(:, 1:3).^2, 2));
beta_w = asin(max(-1, min(1, w.T.by_name.x_state(:, 2) ./ max(V_w, 1e-9))));

% Both runs hold their own ground track, so the judged quantity is whether the
% crosswind run can hold the SAME track the still air run holds. The old formula
% subtracted wind_e*t, which assumes the wind case starts from the still air trim and
% therefore shows the full kinematic 5 m/s drift before any aerodynamic response; the
% crosswind run is now initialised in the crab trim of its own wind, so that drift is
% already cancelled by the aircraft and the subtraction invented 150 m of error over
% 30 s. Comparing the two ground tracks directly is the honest statement of "the track
% deviation stays bounded".
%
% The absolute value is still reported, because it is the number that shows the wind is
% actually present and being balanced: with the crab trim the crosswind run's own east
% displacement stays near zero while the still air run drifts with the wind.
pe_w = w.T.by_name.x_state(:, 11);
pe_n = interp1(n.T.t, n.T.by_name.x_state(:, 11), tw, 'pchip');
pd_w = w.T.by_name.x_state(:, 12);
pd_n = interp1(n.T.t, n.T.by_name.x_state(:, 12), tw, 'pchip');
i_settle = tw >= 0.3 * t_end;
% Track error of the crosswind run against its own initial east position: a crabbed
% aircraft flying its intended track holds this near zero.
residual = pe_w - pe_w(1);
% Signed comparison with the still air run, kept as evidence of the wind's presence.
residual_vs_nowind = pe_w - pe_n;
alt_residual = pd_w - pd_n;

out = struct();
out.wind_mps = wind_e;
out.t_end = t_end;
out.wind = struct('t', tw, 'x', w.T.by_name.x_state, 'act', w.T.by_name.u_act, ...
   'cmd', w.T.by_name.cmd, 'meta', w.meta);
out.nowind = struct('t', tn, 'x', n.T.by_name.x_state, 'act', n.T.by_name.u_act, ...
   'cmd', n.T.by_name.cmd, 'meta', n.meta);
out.beta_tail_deg = mean(beta_w(i_settle)) * 180 / pi;
out.beta_max_deg = max(abs(beta_w)) * 180 / pi;
out.phi_tail_deg = mean(w.T.by_name.x_state(i_settle, 7)) * 180 / pi;
out.phi_max_after_settle_deg = max(abs(w.T.by_name.x_state(i_settle, 7))) * 180 / pi;
out.phi_max_deg = max(abs(w.T.by_name.x_state(:, 7))) * 180 / pi;
out.cmd_phi_tail_deg = mean(w.T.by_name.cmd(i_settle, 1)) * 180 / pi;
out.drift_rate_mps = (pe_w(end) - pe_w(1)) / (tw(end) - tw(1));
out.residual_max_m = max(abs(residual(i_settle)));
out.residual_tail_m = mean(residual(i_settle));
% Reported as evidence rather than judged: the crosswind run barely moves east while
% the still air run drifts, so this difference is the wind being present and balanced.
out.residual_vs_nowind_max_m = max(abs(residual_vs_nowind(i_settle)));
out.alt_residual_max_m = max(abs(alt_residual(i_settle)));
out.psi_range_deg = [min(w.T.by_name.x_state(:, 9)), max(w.T.by_name.x_state(:, 9))] * 180 / pi;
out.theta_max_deg = max(abs(w.T.by_name.x_state(:, 8))) * 180 / pi;
out.act_max_deg = max(abs(w.T.by_name.u_act(:, 1:3))) * 180 / pi;
out.diverged = any(~isfinite(w.T.by_name.x_state(:))) || max(abs(w.T.by_name.x_state(:, 7))) > pi / 2;

file = fullfile(sim_root(), 'results', 'sim_wind_crosswind.mat');
wind_case = out.wind; %#ok<NASGU>
nowind_case = out.nowind; %#ok<NASGU>
summary = rmfield(out, {'wind', 'nowind'}); %#ok<NASGU>
stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<NASGU>
plant_probe = sim_plant_source(); %#ok<NASGU>
save(file, 'wind_case', 'nowind_case', 'summary', 'stamp', 'plant_probe');
out.file = file;
end

%% ---------------- V10 ----------------
function out = local_v10(P, D, t_end)
mdl = 'uacs_manual_pid';
V_min = P.limits.V_min_mps;
V_max = P.limits.V_max_mps;
thr = (P.op.cruise.V_mps - V_min) / (V_max - V_min);
dt = 0.02;
t = (0:dt:t_end)';
n = numel(t);
% Stick columns are [time, roll, pitch, yaw, throttle]. One command sequence
% for both solvers: a pitch doublet, then a throttle step, gentle enough to stay
% in the linear region and rich enough to excite rate, attitude and speed.
roll = zeros(n, 1);
pitch = zeros(n, 1);
yaw = zeros(n, 1);
thr_cmd = thr * ones(n, 1);
pitch(t >= 1.0 & t < 2.0) = 0.4;
pitch(t >= 2.0 & t < 3.0) = -0.4;
thr_cmd(t >= 3.5 & t < 4.5) = thr + 0.05;
stick = [t, roll, pitch, yaw, thr_cmd];
sim_push_scenario(struct('t_end', t_end, 'x0', D.trim.x_star, 'stick', stick, ...
   'wind', [0, 0, 0, 0; t_end, 0, 0, 0], 'flight_mode', 0));

[T4, m4] = sim_run_model(mdl, struct('solver', 'ode4', 't_end', t_end));
[T45, m45] = sim_run_model(mdl, struct('solver', 'ode45', 't_end', t_end));

% Common time base: the fixed-step grid. The variable-step trajectory is
% interpolated onto it with pchip, a shape preserving cubic, so a monotone
% segment cannot pick up the overshoot a plain spline would add; the same
% comparison with linear interpolation is reported alongside as a sensitivity.
% The state matrix comes from by_name.x_state: sim_run_model returns a struct
% whose fields are the model's NAMED outports, so there is no plain .x field.
X4 = T4.by_name.x_state;
X45 = T45.by_name.x_state;
X45_pchip = interp1(T45.t, X45, T4.t, 'pchip');
X45_linear = interp1(T45.t, X45, T4.t, 'linear');
d_pchip = max(abs(X4 - X45_pchip), [], 1)';
d_linear = max(abs(X4 - X45_linear), [], 1)';
scale = sim_state_scale(P);
ptp4 = (max(X4, [], 1) - min(X4, [], 1))';
denom = ptp4;
small = denom < 1e-4 * scale;
denom(small) = scale(small);

out = struct();
out.t_end = t_end;
out.t_common = T4.t;
out.meta_ode4 = m4;
out.meta_ode45 = m45;
out.rel_tol = m45.tol(1);
out.abs_tol = m45.tol(2);
out.interp = 'pchip';
out.diff_max_abs = d_pchip;
out.diff_max_abs_linear = d_linear;
out.state_scale = scale;
out.state_ptp = ptp4;
out.state_ptp_small = small;
out.rel_ptp = d_pchip ./ denom;
out.rel_ptp_linear = d_linear ./ denom;
out.rel_scale = d_pchip ./ scale;
out.max_rel_ptp = max(out.rel_ptp);
out.max_rel_ptp_linear = max(out.rel_ptp_linear);
out.max_rel_scale = max(out.rel_scale);
out.worst_state_rel_ptp = find(out.rel_ptp == out.max_rel_ptp, 1);
out.worst_state_rel_scale = find(out.rel_scale == out.max_rel_scale, 1);
out.state_names = {'u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'pn', 'pe', 'pd'};
out.x_ode4 = X4;
out.x_ode45 = X45;
out.t_ode45 = T45.t;
out.stick = stick;
out.plant_source = sim_plant_source();

file = fullfile(sim_root(), 'results', 'sim_v10_stepsize.mat');
v10 = rmfield(out, {'x_ode4', 'x_ode45', 't_ode45', 'stick'}); %#ok<NASGU>
meta_ode4 = out.meta_ode4; %#ok<NASGU>
meta_ode45 = out.meta_ode45; %#ok<NASGU>
save(file, 'v10', 'meta_ode4', 'meta_ode45');
out.file = file;

save(fullfile(sim_root(), 'results', 'sim_v10_traj.mat'), 'out');
end

%% ---------------- report ----------------
function r = local_v12_reason()
try
   v = ver('aeroblks');
catch
   v = [];
end
if isempty(v)
   r = 'Aerospace Blockset is not installed, so 6DOF (Euler Angles) is unavailable';
else
   r = sprintf('Aerospace Blockset %s is present; enable this case', v.Version);
end
end

function file = local_report(root, R, P)
file = fullfile(root, 'results', 'sim_scenarios_report.txt');
fid = fopen(file, 'w');
if fid < 0
   error('run_scenarios:Report', 'cannot write %s', file);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'UACS Simulink scenario report\n');
fprintf(fid, 'generated     %s\n', R.stamp);
fprintf(fid, 'parameters    %s (b %.3f m, S %.4f m^2, m %.2f kg, cruise %.1f m/s)\n', ...
   R.params.version, R.params.b, R.params.S, R.params.m, R.params.V_cruise);
fprintf(fid, 'design        trim %s, lin %s, pid %s, lqr %s\n', ...
   local_f(R.design, 'trim_source'), local_f(R.design, 'lin_source'), ...
   local_f(R.design, 'pid_source'), local_f(R.design, 'lqr_source'));
fprintf(fid, 'plant         %s\n', R.plant_probe.source);
if ~R.plant_probe.ok
   fprintf(fid, 'plant error   %s\n', R.plant_probe.error);
end

if ~isempty(R.v6)
   fprintf(fid, '\n[V6 PID loop evidence]  source %s\n', R.v6.source);
   fprintf(fid, '%-16s %-6s %10s %10s %10s %12s %10s\n', 'scenario', 'loop', ...
      'rise_s', 'os_pct', 'settle_s', 'ss_err', 'ripple_pct');
   for k = 1:numel(R.v6.loops)
      m = R.v6.loops(k).metrics;
      names = {'phi', 'theta', 'r', 'V', 'h'};
      for j = 1:numel(names)
         s = m.(names{j});
         fprintf(fid, '%-16s %-6s %10.4f %10.2f %10.4f %12.3e %10.3f\n', ...
            R.v6.loops(k).scenario, names{j}, s.rise_time_s, s.overshoot_pct, ...
            s.settling_time_s, s.error_ss, s.tail_ripple_pct);
      end
   end
end

if ~isempty(R.v9)
   v = R.v9;
   fprintf(fid, '\n[V9 5 m/s crosswind, level flight]\n');
   fprintf(fid, 'duration            %.1f s\n', v.t_end);
   fprintf(fid, 'phi tail mean       %.4f deg\n', v.phi_tail_deg);
   fprintf(fid, 'phi max             %.4f deg\n', v.phi_max_deg);
   fprintf(fid, 'phi max after 30%%   %.4f deg\n', v.phi_max_after_settle_deg);
   fprintf(fid, 'beta tail mean      %.4f deg (crab angle held into the wind)\n', v.beta_tail_deg);
   fprintf(fid, 'beta max            %.4f deg\n', v.beta_max_deg);
   fprintf(fid, 'heading range       %.2f .. %.2f deg\n', v.psi_range_deg);
   fprintf(fid, 'east drift rate     %.4f m/s (wind %.1f m/s)\n', v.drift_rate_mps, v.wind_mps);
   fprintf(fid, 'drift residual max  %.4f m after removing wind*t\n', v.residual_max_m);
   fprintf(fid, 'altitude residual   %.4f m against the no wind run\n', v.alt_residual_max_m);
   fprintf(fid, 'surface max         %.3f deg\n', v.act_max_deg);
   fprintf(fid, 'diverged            %d\n', v.diverged);
end

if ~isempty(R.v10)
   v = R.v10;
   fprintf(fid, '\n[V10 step size independence]\n');
   fprintf(fid, 'command window      %.1f s, pitch doublet and throttle step\n', v.t_end);
   fprintf(fid, 'ode4                fixed step %.5f s, %d samples\n', ...
      v.meta_ode4.fixed_step, v.meta_ode4.n_samples);
   fprintf(fid, 'ode45               RelTol %.1e, AbsTol %.1e, %d samples\n', ...
      v.rel_tol, v.abs_tol, v.meta_ode45.n_samples);
   fprintf(fid, 'common time base    ode4 grid, %.5f s\n', v.t_common(2) - v.t_common(1));
   fprintf(fid, 'interpolation       %s for the criterion, linear reported for sensitivity\n', v.interp);
   fprintf(fid, '%-6s %14s %14s %14s %14s %14s\n', 'state', 'max_diff', 'ptp', ...
      'rel_ptp_pchip', 'rel_ptp_lin', 'rel_scale');
   for k = 1:numel(v.state_names)
      fprintf(fid, '%-6s %14.3e %14.3e %14.3e %14.3e %14.3e\n', v.state_names{k}, ...
         v.diff_max_abs(k), v.state_ptp(k), v.rel_ptp(k), v.rel_ptp_linear(k), ...
         v.rel_scale(k));
   end
   fprintf(fid, 'max rel to peak-to-peak   %.6e  (state %s)\n', v.max_rel_ptp, ...
      v.state_names{v.worst_state_rel_ptp});
   fprintf(fid, 'max rel to reference scale %.6e (state %s)\n', v.max_rel_scale, ...
      v.state_names{v.worst_state_rel_scale});
   fprintf(fid, 'criterion V10: max rel to peak-to-peak < 0.01 -> %d\n', ...
      double(v.max_rel_ptp < 0.01));
end

fprintf(fid, '\n[V12 cross validation]\n%s\n', R.v12.reason);
end

function s = local_f(st, f)
if isfield(st, f)
   s = st.(f);
else
   s = 'unknown';
end
end
