function R = run_manual_pid(varargin)
% RUN_MANUAL_PID Scenario runner for model/uacs_manual_pid.slx.
%
%   R = run_manual_pid()
%   R = run_manual_pid('scenarios', {'roll_doublet'}, 't_end', 6)
%
% Scenarios, each a stick schedule written into the base workspace through
% sim_push_scenario so the model is driven only by From Workspace blocks
%   roll_step       roll stick step, attitude mode, phi evidence
%   pitch_step      pitch stick step, attitude mode, theta evidence
%   yaw_rate_step   yaw stick step, direct rate command, r evidence
%   speed_step      throttle stick step, airspeed loop evidence
%   altitude_step   altitude mode with a pitch stick pulse, h and theta evidence
%   roll_doublet    roll doublet, tracking evidence rather than a step metric
%   pitch_doublet   pitch doublet, tracking evidence
%   level_trim      no input at all: the trim must be held
%
% The doublet scenarios are deliberately separate from the step scenarios: a
% doublet returns to its starting value, so the settled value equals the initial
% value and a 10-90 rise time against it is meaningless. Step metrics come from
% the step scenarios, tracking metrics from all of them.
%
% Every scenario saves its trajectory before anything is plotted, so a plotting
% failure cannot lose a run. Variables are task suffixed and local: sim dumps
% the model workspace into the caller's base workspace, so a plot script that
% reuses a name such as t or y would collide with it.
%
% Output R.traj is a struct array with t, x, act, cmd, rate_cmd, M_cmd, T_cmd,
% meta and metrics per scenario.

ip = inputParser;
ip.addParameter('scenarios', {'roll_step', 'pitch_step', 'yaw_rate_step', ...
   'speed_step', 'altitude_step', 'roll_doublet', 'pitch_doublet', 'level_trim'}, @iscell);
ip.addParameter('solver', 'ode4', @(x) ischar(x) || isstring(x));
ip.addParameter('plots', true, @(x) islogical(x) || isnumeric(x));
ip.addParameter('save', true, @(x) islogical(x) || isnumeric(x));
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

mdl = 'uacs_manual_pid';
model_file = fullfile(root, 'model', [mdl '.slx']);
if ~exist(model_file, 'file')
   build_uacs_models();
end
% sim_run_model loads by bare model name, which resolves only when the model is
% already loaded or model/ is on the path. Neither holds in a fresh session, so
% the load has to happen here or a clean run fails with "Unable to find system".
if ~bdIsLoaded(mdl)
   load_system(model_file);
end

specs = local_specs(P, D);
sel = opt.scenarios;
R = struct();
R.stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
R.model = mdl;
R.design = D.src;
R.plant_probe = probe;
R.push_count = push.count;
R.traj = struct('name', {}, 't', {}, 'x', {}, 'act', {}, 'cmd', {}, ...
   'rate_cmd', {}, 'M_cmd', {}, 'T_cmd', {}, 'metrics', {}, 'meta', {}, 'spec', {});
R.figures = {};

for k = 1:numel(sel)
   name = sel{k};
   idx = find(strcmp({specs.name}, name), 1);
   if isempty(idx)
      error('run_manual_pid:Scenario', 'unknown scenario %s', name);
   end
   sp = specs(idx);
   S = sim_push_scenario(struct('t_end', sp.t_end, 'x0', D.trim.x_star, ...
      'stick', sp.stick, 'wind', sp.wind, 'flight_mode', sp.mode));
   [T, meta] = sim_run_model(mdl, struct('solver', opt.solver, 't_end', sp.t_end));
   tr = local_unpack(T);
   tr.name = name;
   tr.spec = sp;
   tr.metrics = local_metrics(tr, sp);
   tr.meta = meta;
   R.traj(end + 1) = tr; %#ok<AGROW>
   fprintf('  %-16s %5d samples  %s\n', name, meta.n_samples, local_summary(tr.metrics));
end

if opt.save
   out_file = fullfile(root, 'results', 'sim_manual_pid_traj.mat');
   traj = R.traj; %#ok<NASGU>
   design_src = R.design; %#ok<NASGU>
   plant_probe = R.plant_probe; %#ok<NASGU>
   stamp = R.stamp; %#ok<NASGU>
   save(out_file, 'traj', 'design_src', 'plant_probe', 'stamp');
   R.file = out_file;
   fprintf('saved %s\n', out_file);
end

if opt.plots
   R.figures = sim_plot_traj(R.traj, 'manual_pid', sim_root());
end
end

%% ---------------- scenario definitions ----------------
function specs = local_specs(P, D)
V_min = P.limits.V_min_mps;
V_max = P.limits.V_max_mps;
thr_cruise = (P.op.cruise.V_mps - V_min) / (V_max - V_min);
dt = 0.02;
t_step_att = 10;
t_step_speed = 20;
t_step_alt = 40;
t_doublet = 8;
t_trim = 20;
% Stick amplitudes stay inside the linear region: 0.3 of the bank authority is
% about 13.5 deg and 0.3 of the pitch authority about 9 deg.
a_roll = 0.3;
a_pitch = 0.3;

% Field order and set must match local_pack exactly: MATLAB refuses a subscripted
% assignment between dissimilar structures, so the empty template is built to the
% same shape the packer returns.
specs = struct('name', {}, 't_end', {}, 'mode', {}, 'wind', {}, 'stick', {}, 'thr_base', {});

% Steps, held to the end so a settled value exists.
specs(end + 1) = local_pack('roll_step', t_step_att, 0, dt, ...
   {{'roll', 2.0, t_step_att, a_roll}}, thr_cruise, [0 0 0 0]);
specs(end + 1) = local_pack('pitch_step', t_step_att, 0, dt, ...
   {{'pitch', 2.0, t_step_att, a_pitch}}, thr_cruise, [0 0 0 0]);
% Yaw stick is the one channel the pilot commands as a rate directly.
specs(end + 1) = local_pack('yaw_rate_step', t_step_att, 0, dt, ...
   {{'yaw', 2.0, t_step_att, 1.0}}, thr_cruise, [0 0 0 0]);
% Throttle step of 2 m/s on the airspeed command, through pilot_to_command.
thr_step = 2.0 / (V_max - V_min);
specs(end + 1) = local_pack('speed_step', t_step_speed, 0, dt, ...
   {{'throttle', 2.0, t_step_speed, thr_cruise + thr_step}}, thr_cruise, [0 0 0 0]);
% Altitude mode: a pitch stick pulse commands a climb of about 24 m and then
% holds the new altitude, so the height loop gets a ramp and a hold.
specs(end + 1) = local_pack('altitude_step', t_step_alt, 1, dt, ...
   {{'pitch', 2.0, 6.0, 1.0}}, thr_cruise, [0 0 0 0]);
% Doublets: same channels, back to the starting value, tracking evidence.
specs(end + 1) = local_pack('roll_doublet', t_doublet, 0, dt, ...
   {{'roll', 2.0, 3.0, a_roll}, {'roll', 3.0, 4.0, -a_roll}}, thr_cruise, [0 0 0 0]);
specs(end + 1) = local_pack('pitch_doublet', t_doublet, 0, dt, ...
   {{'pitch', 2.0, 3.0, a_pitch}, {'pitch', 3.0, 4.0, -a_pitch}}, thr_cruise, [0 0 0 0]);
% No input at all: the aircraft must hold its trim.
specs(end + 1) = local_pack('level_trim', t_trim, 0, dt, {{}}, thr_cruise, [0 0 0 0]);
end

function s = local_pack(name, t_end, mode, dt, segs, thr_base, wind)
t = (0:dt:t_end)';
n = numel(t);
roll = zeros(n, 1);
pitch = zeros(n, 1);
yaw = zeros(n, 1);
thr = thr_base * ones(n, 1);
for k = 1:numel(segs)
   seg = segs{k};
   % A segment is a 1x4 cell {channel, t_start, t_end, value}. The empty list
   % {{}} that level_trim passes arrives as one segment holding an empty array,
   % not as zero segments, so the shape is checked rather than the count.
   if ~iscell(seg) || numel(seg) < 4
      continue;
   end
   m = t >= seg{2} & t < seg{3};
   switch seg{1}
      case 'roll', roll(m) = seg{4};
      case 'pitch', pitch(m) = seg{4};
      case 'yaw', yaw(m) = seg{4};
      case 'throttle', thr(m) = seg{4};
      otherwise, error('run_manual_pid:Channel', 'unknown channel %s', seg{1});
   end
end
s = struct('name', name, 't_end', t_end, 'mode', mode, 'wind', [0, wind; t_end, wind], ...
   'stick', [t, roll, pitch, yaw, thr], 'thr_base', thr_base);
end

%% ---------------- unpacking and metrics ----------------
function tr = local_unpack(T)
i_x = local_index(T, 'x_state');
i_act = local_index(T, 'u_act');
i_cmd = local_index(T, 'cmd');
i_rate = local_index(T, 'rate_cmd');
i_M = local_index(T, 'M_cmd');
i_T = local_index(T, 'T_cmd');
tr = struct();
tr.t = T.t;
tr.x = T.by_name.(i_x);
tr.act = T.by_name.(i_act);
tr.cmd = T.by_name.(i_cmd);
tr.rate_cmd = T.by_name.(i_rate);
tr.M_cmd = T.by_name.(i_M);
tr.T_cmd = T.by_name.(i_T);
end

function name = local_index(T, want)
name = matlab.lang.makeValidName(want);
assert(isfield(T.by_name, name), 'run_manual_pid:Output', ...
   'model output %s is missing; outputs are %s', want, strjoin(T.names, ', '));
end

function m = local_metrics(tr, sp)
% Per-loop rise time, overshoot, settling time and steady-state error, each
% against the model's own recorded command, plus a tail ripple measure so a
% sustained oscillation cannot pass as a settled response.
t = tr.t;
m = struct();
% V is the norm of the body velocity, the same definition plant/flow_state.m
% uses, so no extra model output is needed to read it back.
V = sqrt(sum(tr.x(:, 1:3).^2, 2));
m.phi = local_step(t, tr.x(:, 7), tr.cmd(:, 1), 2.0);
m.theta = local_step(t, tr.x(:, 8), tr.cmd(:, 2), 2.0);
m.r = local_step(t, tr.x(:, 6), tr.cmd(:, 3), 2.0);
m.V = local_step(t, V, tr.cmd(:, 4), 2.0);
m.h = local_step(t, -tr.x(:, 12), tr.cmd(:, 5), 2.0);
m.p_track = local_track(t, tr.x(:, 4), tr.rate_cmd(:, 1));
m.q_track = local_track(t, tr.x(:, 5), tr.rate_cmd(:, 2));
m.r_track = local_track(t, tr.x(:, 6), tr.rate_cmd(:, 3));
% Attitude, airspeed and altitude tracking against the model's own command, so
% a doublet scenario has a number even though it has no settled value.
m.phi_track = local_track(t, tr.x(:, 7), tr.cmd(:, 1));
m.theta_track = local_track(t, tr.x(:, 8), tr.cmd(:, 2));
m.V_track = local_track(t, V, tr.cmd(:, 4));
m.h_track = local_track(t, -tr.x(:, 12), tr.cmd(:, 5));
m.rate_cmd_peak = max(abs(tr.rate_cmd), [], 1);
m.cmd_peak = max(abs(tr.cmd), [], 1);
m.act_max_deg = max(abs(tr.act(:, 1:3))) * 180 / pi;
m.throttle_range = [min(tr.act(:, 4:5), [], 'all'), max(tr.act(:, 4:5), [], 'all')];
m.T_cmd_range = [min(tr.T_cmd), max(tr.T_cmd)];
m.theta_excursion_deg = [min(tr.x(:, 8)), max(tr.x(:, 8))] * 180 / pi;
m.theta_over_90_deg = any(abs(tr.x(:, 8)) > pi / 2);
m.wind = sp.wind(2:4);
end

function s = local_step(t, y, ref, t_step)
target = local_tail_mean(ref);
s = step_metrics(t, y, t_step, target);
s.ref_tail = target;
seg = t >= t_step;
s.tail_ripple_pct = local_ripple(y(seg), s.y_ss, s.delta);
s.command_reach_s = local_ramp_time(t, ref);
end

function s = local_track(t, y, ref)
s = struct();
e = y - ref;
s.err_max = max(abs(e));
s.err_rms = sqrt(mean(e.^2));
i = t >= 0.5 * max(t);
s.err_rms_tail = sqrt(mean(e(i).^2));
s.corr = local_corr(y, ref);
end

function v = local_tail_mean(y)
n = numel(y);
v = mean(y(max(n - round(0.02 * n), 1):n));
end

function p = local_ripple(seg, y_ss, delta)
if isempty(seg) || abs(delta) < 1e-12
   p = NaN;
   return;
end
n = numel(seg);
tail = seg(max(n - round(0.2 * n), 1):n);
p = (max(tail) - min(tail)) / abs(delta) * 100;
end

function dt = local_ramp_time(t, ref)
% Time from the start of the record until the command has essentially reached
% its final value. The stick path is slew limited by pilot_to_command, so a
% commanded step is really a ramp and the reference trace is the honest one.
ref = ref(:);
mag = ref(end) - ref(1);
if abs(mag) < 1e-12
   dt = NaN;
   return;
end
i = find(abs(ref - ref(1)) >= 0.98 * abs(mag), 1);
if isempty(i)
   dt = NaN;
else
   dt = t(i) - t(1);
end
end

function c = local_corr(a, b)
a = a(:) - mean(a);
b = b(:) - mean(b);
if norm(a) < eps || norm(b) < eps
   c = NaN;
else
   c = a' * b / (norm(a) * norm(b));
end
end

function s = local_summary(m)
s = sprintf('phi tr %.2fs os %.1f%% e %.2e | V tr %.2fs os %.1f%% e %.2e | h e %.2f m', ...
   m.phi.rise_time_s, m.phi.overshoot_pct, m.phi.error_ss, ...
   m.V.rise_time_s, m.V.overshoot_pct, m.V.error_ss, m.h.error_ss);
end
