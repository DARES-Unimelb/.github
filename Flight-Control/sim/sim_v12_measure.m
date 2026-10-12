function M = sim_v12_measure(P, opts)
% SIM_V12_MEASURE Run the hand written plant and the Aerospace Blockset reference
% plant on the same command sequence and record the per-state difference.
%
%   M = sim_v12_measure(P)
%   M = sim_v12_measure(P, struct('t_end', 9.0, 'tol', 1e-6, 'quiet', false))
%
% Measures, does not assert: the pass or fail decision is
% verify/v_sim_v12_crosscheck.m, which reads results/sim_v12_crosscheck.mat back
% from disk. The one thing it does refuse to do is compare a diverged trajectory.
%
% Five configurations, all driven through sim_run_plant_cmd so the two models see
% bit-identical inputs. Each sets the density law on BOTH sides, so every
% configuration is an apples to apples test and a non-zero difference means the
% two implementations disagree rather than that they are modelling different air.
%
%   const_density  our P.atmo.model 'const' against the ISA block evaluated at
%                  sea level, so both fly in P.env.rho_sl. This is the criterion
%                  run: the only remaining difference is the integrator and the
%                  kinematics.
%   const_cmd      the same density, one fixed elevator and throttle setting.
%   crosswind      the same density plus 5 m/s of east wind.
%   isa_frozen     our plant/atmo_isa.m at a frozen altitude of h_ref against the
%                  ISA block evaluated at that same altitude.
%   isa_state      the altitude law as well: -pd on our side, -Xe(3) on the
%                  block's, both moving with the trajectory.
%
% Writes results/sim_v12_crosscheck.mat.

if nargin < 2
   opts = struct();
end
if ~isfield(opts, 't_end')
   opts.t_end = 9.0;
end
if ~isfield(opts, 'tol')
   opts.tol = 1e-4;
end
quiet = isfield(opts, 'quiet') && opts.quiet;

root = sim_root();
tr = sim_v12_trim(P);
x0 = tr.x_star(:);
u_trim = tr.u_star(:);
scale = sim_state_scale(P);
h = P.sim.Ts_ctrl;
h_trim = -x0(12);

M = struct();
M.stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
M.params_revision = P.meta.revision;
M.trim_source = sprintf('trim_cruise at %.1f m/s, alpha %.4f deg, de %.4f deg, throttle %.4f', ...
   tr.V_mps, tr.alpha_deg, tr.de_deg, tr.throttle);
M.x0 = x0;
M.u_trim = u_trim;
M.state_names = {'u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'pn', 'pe', 'pd'};
M.state_scale = scale;
M.tol = opts.tol;
M.fixed_step = h;
M.plant_source = sim_plant_source();

t = (0:h:opts.t_end)';
[U_doublet, desc_doublet] = local_doublet(u_trim, t);
W_zero = zeros(numel(t), 3);
t_short = (0:h:5.0)';
U_short = repmat(local_const_input(u_trim), numel(t_short), 1);

% Density laws. 'const' is the frozen rev 9 behaviour and must stay bit for bit
% what it was; 'isa' turns on plant/atmo_isa.m.
atmo_const = struct('model', 'const', 'h_from_state', false, 'h_ref', 0);
atmo_frozen = struct('model', 'isa', 'h_from_state', false, 'h_ref', h_trim);
atmo_state = struct('model', 'isa', 'h_from_state', true, 'h_ref', h_trim);

cfg = local_one(P, atmo_const, 0, 0, 'const_density', x0, t, U_doublet, W_zero, scale, ...
   desc_doublet);
cfg(2) = local_one(P, atmo_const, 0, 0, 'const_cmd', x0, t_short, U_short, ...
   zeros(numel(t_short), 3), scale, 'fixed elevator and throttle');
cfg(3) = local_one(P, atmo_const, 0, 0, 'crosswind', x0, t_short, U_short, ...
   repmat([0, 5.0, 0], numel(t_short), 1), scale, ...
   'fixed elevator and throttle, 5 m/s east wind');
cfg(4) = local_one(P, atmo_frozen, h_trim, 0, 'isa_frozen', x0, t, U_doublet, W_zero, ...
   scale, sprintf('%s, both densities from ISA frozen at %.0f m', desc_doublet, h_trim));
cfg(5) = local_one(P, atmo_state, h_trim, 1, 'isa_state', x0, t, U_doublet, W_zero, ...
   scale, sprintf('%s, both densities from ISA at the moving altitude', desc_doublet));

M.cfgs = cfg;
M.criterion_cfg = 1;
M.max_rel = cfg(1).max_rel;
M.worst_state = cfg(1).worst_state;
M.max_rel_isa_frozen = cfg(4).max_rel;
M.max_rel_isa_state = cfg(5).max_rel;
M.max_rel_crosswind = cfg(3).max_rel;
M.rho_sl = P.env.rho_sl;
M.rho_isa_frozen = cfg(4).aux_ref(1);

% The wind frame term. Measured directly rather than inferred from a trajectory:
% the two models now share the same right-hand side, so the check is that
% sim_plant_rhs removes exactly omega x (R*W) and nothing else.
M.wind_frame = local_wind_frame(P, x0, u_trim);

save(fullfile(root, 'results', 'sim_v12_crosscheck.mat'), 'M');
if ~quiet
   fprintf('V12 cross check -> %s\n', fullfile(root, 'results', 'sim_v12_crosscheck.mat'));
   fprintf('V12 criterion %s: max rel %.6e at %s, limit %.3e\n', ...
      cfg(1).name, M.max_rel, M.state_names{M.worst_state}, M.tol);
   fprintf('V12 ISA frozen at %.0f m: max rel %.6e (rho %.9f against %.9f)\n', ...
      h_trim, M.max_rel_isa_frozen, M.rho_isa_frozen, M.rho_sl);
   fprintf('V12 ISA at the moving altitude: max rel %.6e\n', M.max_rel_isa_state);
   fprintf('V12 crosswind: max rel %.6e\n', M.max_rel_crosswind);
   for k = 1:numel(cfg)
      fprintf('V12 %-14s against the trajectory excursion %.4e, against the state scale %.4e\n', ...
         cfg(k).name, cfg(k).max_rel, cfg(k).max_rel_scale);
   end
   fprintf(['V12 wind frame: sim_plant_rhs dx - the ground frame equation = %s,\n' ...
      '    worst %.3e over all twelve; the term that used to be missing is %s\n'], ...
      mat2str(M.wind_frame.diff', 3), M.wind_frame.residual, ...
      mat2str(M.wind_frame.omega_cross_RW', 6));
end
end

%% ---------------- one configuration ----------------
function c = local_one(P, atmo, ref_h_isa, ref_h_from_state, name, x0, t, U, W, scale, desc)
% Every base workspace variable either model reads travels in one struct that
% sim_run_plant_cmd pushes immediately before the simulation. The shared base
% workspace can be cleared by another member while this job runs, and a plant
% that cannot find P looks like a modelling fault rather than a workspace one.
%
% The density law is set on our side through P.atmo and on the reference side
% through the ISA block's altitude input, so both models fly in the same air.
Pc = P;
Pc.atmo.model = atmo.model;
Pc.atmo.h_from_state = atmo.h_from_state;
Pc.atmo.h_ref = atmo.h_ref;
vars = struct('P', Pc, 'sim_ts_ctrl', t(2) - t(1), 'sim_t_end', t(end), 'sim_x0', x0);
vars = local_merge(vars, sim_push_ref_vars(Pc, x0, ref_h_isa, ref_h_from_state));
opts = struct('solver', 'ode4', 'vars', vars);

[T_our, m_our] = sim_run_plant_cmd('uacs_plant', t, U, W, opts);
[T_ref, m_ref] = sim_run_plant_cmd('uacs_plant_ref', t, U, W, opts);
X_our = T_our.by_name.x_state;
X_ref = T_ref.by_name.x_state;
assert(all(isfinite(X_our(:))) && all(isfinite(X_ref(:))), 'sim_v12_measure:Diverged', ...
   '%s produced a non finite state', name);

% Both models use the same fixed step and the same stop time, so the grids are
% expected to be identical. If they ever are not, the reference trajectory is
% interpolated onto ours with pchip, the same rule V10 uses, and that is recorded.
interp = 'none';
if numel(T_our.t) ~= numel(T_ref.t) || any(abs(T_our.t - T_ref.t) > 1e-12)
   X_ref = interp1(T_ref.t, X_ref, T_our.t, 'pchip');
   interp = 'pchip';
end
tt = T_our.t;
% Two guards that make the element by element comparison legitimate.
% The 6DOF block wraps its Euler angle output into (-pi, pi] while our model
% accumulates, measured in sim_ref_calibrate, so an angle that leaves that range
% would differ by a whole turn for an identical attitude. The command sequence is
% sized to stay well inside it, and this asserts that it did.
assert(max(abs(X_our(:, 7))) < pi / 2, 'sim_v12_measure:Singularity', ...
   '%s rolled past 90 deg, where the Euler angle kinematics of both models break down', name);
assert(max(abs(X_our(:, 8))) < pi / 2 && max(abs(X_our(:, 9))) < pi, ...
   'sim_v12_measure:Wrap', ...
   '%s left the Euler angle range where the block wraps and our model does not', name);

d = max(abs(X_ref - X_our), [], 1)';
ptp = (max(X_our, [], 1) - min(X_our, [], 1))';
denom = ptp;
small = denom < 1e-4 * scale;
denom(small) = scale(small);
rel = d ./ denom;
% The trajectory excursion is the strict denominator for a state that moves, but
% it is a degenerate one for a state that barely moves: the ISA configuration's
% altitude excursion is 0.53 m while the separation reaches 0.83 m, which reads
% as 158 percent. The same difference against the reference scale of the state is
% the number to quote for the reported configurations, and both are kept.
rel_scale = d ./ scale;

c = struct();
c.name = name;
c.desc = desc;
c.ref_h_isa = ref_h_isa;
c.t = tt;
c.u_act = U;
c.wind = W;
c.x_our = X_our;
c.x_ref = X_ref;
c.meta_our = m_our;
c.meta_ref = m_ref;
c.aux_ref = T_ref.by_name.aux(1, :)';
c.diff_max_abs = d;
c.state_ptp = ptp;
c.denom = denom;
c.denom_small = small;
c.rel = rel;
c.rel_scale = rel_scale;
c.max_rel = max(rel);
c.max_rel_scale = max(rel_scale);
c.worst_state = find(rel == c.max_rel, 1);
c.worst_state_scale = find(rel_scale == c.max_rel_scale, 1);
c.interp = interp;
end

%% ---------------- commands ----------------
function [U, desc] = local_doublet(u_trim, t)
% Pitch, throttle, roll and yaw inputs in turn, small enough that the aircraft
% stays inside the Euler angle region both models are valid in, rich enough that
% every one of the twelve states moves.
da = zeros(size(t));
de = zeros(size(t));
dr = zeros(size(t));
thr = zeros(size(t));
de(t >= 1.0 & t < 2.0) = deg2rad(0.4);
de(t >= 2.0 & t < 3.0) = deg2rad(-0.4);
thr(t >= 3.0 & t < 4.0) = 0.04;
da(t >= 4.0 & t < 5.0) = deg2rad(1.0);
da(t >= 5.0 & t < 6.0) = deg2rad(-1.0);
dr(t >= 6.0 & t < 7.0) = deg2rad(0.5);
dr(t >= 7.0 & t < 7.6) = deg2rad(-0.5);
U = [u_trim(1) + da, u_trim(2) + de, u_trim(3) + dr, ...
   min(u_trim(4) + thr, 1.0), min(u_trim(5) + thr, 1.0), zeros(size(t))];
desc = 'pitch, throttle, roll and yaw doublets';
end

function u = local_const_input(u_trim)
u = u_trim(:)';
u(2) = u(2) - deg2rad(1.0);
u(4) = min(u(4) + 0.05, 1.0);
u(5) = u(4);
u(6) = 0;
end

%% ---------------- wind frame check ----------------
function R = local_wind_frame(P, x, u)
% Two derivatives at the same state, with a steady wind: the one
% sim/sim_plant_rhs.m returns, and the one the ground-velocity state obeys,
% written out here from the state so the comparison does not go through the
% function under test. All twelve components are compared, not just the three
% the wind frame term lives in.
%
% The quantity the old version got wrong is carried as well, so the record shows
% how large it was: omega x (R*W), the term between d(v_air)/dt and d(v_g)/dt.
W = [0; 5.0; 0];
omega = [0.06; -0.04; 0.03];
x = x(:);
x(4:6) = omega;
u = u(:);

dx_our = sim_plant_rhs(0, x, u, W, P);

R_b = sim_dcm(x(7:9));
x_air = x;
x_air(1:3) = x(1:3) - R_b * W;
[~, F, M] = eom_6dof(0, x_air, u, P);
I_mat = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; ...
         0, P.mass.Iyy_cg, 0; ...
         -P.mass.Ixz_cg, 0, P.mass.Izz_cg];
phi = x(7);
theta = x(8);
dx_ground = zeros(12, 1);
dx_ground(1:3) = F / P.mass.m - cross(omega, x(1:3));
dx_ground(4:6) = I_mat \ (M - cross(omega, I_mat * omega));
dx_ground(7) = omega(1) + tan(theta) * (omega(2) * sin(phi) + omega(3) * cos(phi));
dx_ground(8) = omega(2) * cos(phi) - omega(3) * sin(phi);
dx_ground(9) = (omega(2) * sin(phi) + omega(3) * cos(phi)) / cos(theta);
dx_ground(10:12) = R_b' * x(1:3);

R = struct();
R.W = W;
R.omega = omega;
R.x = x;
R.dx_sim_plant_rhs = dx_our;
R.dx_ground_frame = dx_ground;
R.diff_1_3 = dx_our(1:3) - dx_ground(1:3);
R.omega_cross_RW = cross(omega, R_b * W);
R.diff = dx_our - dx_ground;
R.residual = max(abs(R.diff));
R.tol = 1e-12;
R.pass = R.residual < R.tol;
end

function S = local_merge(S, T)
f = fieldnames(T);
for k = 1:numel(f)
   S.(f{k}) = T.(f{k});
end
end
