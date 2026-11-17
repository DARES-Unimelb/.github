function R = run_lqr(varargin)
% RUN_LQR Scenario runner for model/uacs_lqr.slx.
%
%   R = run_lqr()
%   R = run_lqr('scenarios', {'bank_step'}, 'solver', 'ode4')
%
% Scenarios drive the commanded deviations of control/lqr_reference.m
%   speed_step     dV_c step of +3 m/s, the regulated output of the long loop
%   bank_step      dphi_c step of 10 deg, the regulated output of the lat loop
%   speed_doublet  dV_c doublet, no steady state to reach
%   bank_doublet   dphi_c doublet
%   level_trim     no command at all: the aircraft must hold trim
%
% Options: scenarios, solver ('ode4'), plots (true), save (true), design ([]).
%
% The reference is a deviation from the design point, so the absolute target of a metric is
% the operating point value plus the commanded deviation.

ip = inputParser;
ip.addParameter('scenarios', {'speed_step', 'bank_step', 'speed_doublet', ...
   'bank_doublet', 'level_trim'}, @iscell);
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

mdl = 'uacs_lqr';
model_file = fullfile(root, 'model', [mdl '.slx']);
if ~exist(model_file, 'file')
   build_uacs_models();
end

specs = local_specs(P, D);
sel = opt.scenarios;
R = struct();
R.stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
R.model = mdl;
R.design = D.src;
R.plant_probe = probe;
R.refmap_linearity = push.refmap.linearity;
R.refmap_residual = [push.refmap.residual_long, push.refmap.residual_lat];
R.traj = struct('name', {}, 't', {}, 'x', {}, 'act', {}, 'ref', {}, 'u_dev', {}, ...
   'xi', {}, 'M_cmd', {}, 'T_cmd', {}, 'metrics', {}, 'meta', {}, 'spec', {});
R.figures = {};

for k = 1:numel(sel)
   name = sel{k};
   idx = find(strcmp({specs.name}, name), 1);
   if isempty(idx)
      error('run_lqr:Scenario', 'unknown scenario %s', name);
   end
   sp = specs(idx);
   sim_push_scenario(struct('t_end', sp.t_end, 'x0', D.trim.x_star, ...
      'ref', sp.ref, 'wind', sp.wind, 'flight_mode', 0));
   [T, meta] = sim_run_model(mdl, struct('solver', opt.solver, 't_end', sp.t_end));
   tr = local_unpack(T);
   tr.name = name;
   tr.spec = sp;
   tr.metrics = local_metrics(tr, sp, D);
   tr.meta = meta;
   R.traj(end + 1) = tr; %#ok<AGROW>
   fprintf('  %-16s %5d samples  %s\n', name, meta.n_samples, local_summary(tr.metrics));
end

if opt.save
   out_file = fullfile(root, 'results', 'sim_lqr_traj.mat');
   traj = R.traj; %#ok<NASGU>
   design_src = R.design; %#ok<NASGU>
   plant_probe = R.plant_probe; %#ok<NASGU>
   refmap_linearity = R.refmap_linearity; %#ok<NASGU>
   refmap_residual = R.refmap_residual; %#ok<NASGU>
   stamp = R.stamp; %#ok<NASGU>
   save(out_file, 'traj', 'design_src', 'plant_probe', 'refmap_linearity', ...
      'refmap_residual', 'stamp');
   R.file = out_file;
   fprintf('saved %s\n', out_file);
end
if opt.plots
   R.figures = sim_plot_lqr(R.traj, sim_root());
end
end

%% Scenarios
function specs = local_specs(P, D)
dt = 0.02;
t_short = 8;
t_long = 20;
dV = 3.0;
dphi = deg2rad(10);
specs = struct('name', {}, 't_end', {}, 'ref', {}, 'wind', {});
specs(end + 1) = local_pack('speed_step', t_short, dt, {{'dV', 2.0, t_short, dV}}, [0 0 0 0]);
specs(end + 1) = local_pack('bank_step', t_short, dt, {{'dphi', 2.0, t_short, dphi}}, [0 0 0 0]);
specs(end + 1) = local_pack('speed_doublet', t_long, dt, ...
   {{'dV', 2.0, 6.0, dV}, {'dV', 6.0, 10.0, -dV}}, [0 0 0 0]);
specs(end + 1) = local_pack('bank_doublet', t_long, dt, ...
   {{'dphi', 2.0, 6.0, dphi}, {'dphi', 6.0, 10.0, -dphi}}, [0 0 0 0]);
specs(end + 1) = local_pack('level_trim', t_long, dt, {{}}, [0 0 0 0]);
end

function s = local_pack(name, t_end, dt, segs, wind)
t = (0:dt:t_end)';
n = numel(t);
dV = zeros(n, 1);
dth = zeros(n, 1);
dphi = zeros(n, 1);
db = zeros(n, 1);
for k = 1:numel(segs)
   seg = segs{k};
   m = t >= seg{2} & t < seg{3};
   switch seg{1}
      case 'dV', dV(m) = seg{4};
      case 'dtheta', dth(m) = seg{4};
      case 'dphi', dphi(m) = seg{4};
      case 'dbeta', db(m) = seg{4};
      otherwise, error('run_lqr:Channel', 'unknown channel %s', seg{1});
   end
end
s = struct('name', name, 't_end', t_end, 'wind', wind, ...
   'ref', [t, dV, dth, dphi, db], ...
   'cmd', struct('dV', dV, 'dtheta', dth, 'dphi', dphi, 'dbeta', db));
end

%% Unpacking and metrics
function tr = local_unpack(T)
tr = struct();
tr.t = T.t;
tr.x = T.by_name.x_state;
tr.act = T.by_name.u_act;
tr.ref = T.by_name.ref;
tr.u_dev = T.by_name.u_dev;
tr.xi = T.by_name.xi;
tr.M_cmd = T.by_name.M_cmd;
tr.T_cmd = T.by_name.T_cmd;
end

function m = local_metrics(tr, sp, D)
t = tr.t;
xo = D.gains_lqr.long.x_op;
V = sqrt(sum(tr.x(:, 1:3).^2, 2));
V_ref = xo(1) + tr.ref(:, 1);
phi_ref = xo(4) + tr.ref(:, 3);
m = struct();
m.V = step_metrics(t, V, 2.0, mean(V_ref(t > 0.8 * t(end))));
m.phi = step_metrics(t, tr.x(:, 7), 2.0, mean(phi_ref(t > 0.8 * t(end))));
m.V_track = local_track(V, V_ref);
m.phi_track = local_track(tr.x(:, 7), phi_ref);
m.z_long_tail = mean(tr.xi(max(1, end - round(0.05 * numel(t))):end, 1));
m.z_lat_tail = mean(tr.xi(max(1, end - round(0.05 * numel(t))):end, 2));
m.xi_max = max(abs(tr.xi), [], 1);
m.act_max_deg = max(abs(tr.act(:, 1:3))) * 180 / pi;
m.throttle_range = [min(tr.act(:, 4:5), [], 'all'), max(tr.act(:, 4:5), [], 'all')];
m.u_dev_max = max(abs(tr.u_dev), [], 1);
m.wind = sp.wind(2:4);
end

function s = local_track(y, ref)
e = y(:) - ref(:);
s = struct('err_max', max(abs(e)), 'err_rms', sqrt(mean(e.^2)));
end

function s = local_summary(m)
s = sprintf('V ss err %.3e m/s, phi ss err %.3e rad, z tail [%.3e %.3e], act max %.2f deg', ...
   m.V.error_ss, m.phi.error_ss, m.z_long_tail, m.z_lat_tail, m.act_max_deg);
end
