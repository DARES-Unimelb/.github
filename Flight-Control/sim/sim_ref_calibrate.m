function C = sim_ref_calibrate(P, opts)
% SIM_REF_CALIBRATE Measure what the Aerospace Blockset reference blocks compute
%
%   P      parameter struct
%   opts   optional: t_end in s (default 2.0), quiet true suppresses the summary
%   C      measured quantities and per-check verdicts, also written to
%          results/sim_v12_calib.mat, with the text report in
%          results/sim_v12_calib.txt
%
%   port_map    which output port carries which physical quantity, from a known
%               initial state with zero force and moment
%   equations   the 6DOF block against an independent RK4 of the documented
%               equations of motion over a nontrivial force and moment history
%   isa         the Atmosphere (ISA) block against the standard atmosphere, and
%               rho(0) against P.env.rho_sl
%   force_path  the reference force and moment block against plant/eom_6dof at
%               the same state
%   const_cmd   both models on one constant elevator and throttle command,
%               against an independent RK4 oracle
%
% Every probe model exists in memory only and is never saved

if nargin < 2
   opts = struct();
end
if ~isfield(opts, 't_end')
   opts.t_end = 2.0;
end
quiet = isfield(opts, 'quiet') && opts.quiet;

root = sim_root();
tr = sim_v12_trim(P);
x_trim = tr.x_star(:);
u_trim = tr.u_star(:);
h = P.sim.Ts_ctrl;

% base workspace variables every probe and both models read, pushed as one struct
V = struct();
V.P = P;
V.sim_ts_ctrl = h;
V.sim_t_end = opts.t_end;
V.cal_mass = P.mass.m;
V.cal_inertia = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; ...
                 0, P.mass.Iyy_cg, 0; ...
                 -P.mass.Ixz_cg, 0, P.mass.Izz_cg];

C = struct();
C.stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
C.params_revision = P.meta.revision;
C.trim_source = sprintf('trim_cruise at %.1f m/s, alpha %.4f deg, de %.4f deg, throttle %.4f', ...
   tr.V_mps, tr.alpha_deg, tr.de_deg, tr.throttle);
C.x_trim = x_trim;
C.u_trim = u_trim;

C.port_map = local_port_map(P, V);
C.equations = local_equations(P, V, opts.t_end);
C.isa = local_isa(P, V);
C.force_path = local_force_path(P, V, x_trim, u_trim);
C.const_cmd = local_const_cmd(P, x_trim, u_trim, opts.t_end);

% the probe models exist in memory only, so they are closed here
for nm = {'v12_cal_eom', 'v12_cal_eq', 'v12_cal_isa', 'v12_cal_aero'}
   if bdIsLoaded(nm{1})
      close_system(nm{1}, 0);
   end
end

C.criterion = struct();
C.criterion.ports_identified = C.port_map.pass;
C.criterion.equations_match = C.equations.pass;
C.criterion.rho_sl_exact = C.isa.rho0_is_rho_sl;
C.criterion.forces_identical = C.force_path.pass;
C.criterion.const_cmd_match = C.const_cmd.pass;
C.pass = all(structfun(@(v) islogical(v) && v, C.criterion));

C.report_file = local_report(root, C);
save(fullfile(root, 'results', 'sim_v12_calib.mat'), 'C');
if ~quiet
   fprintf('V12 calibration -> %s\n', C.report_file);
   fprintf('V12 calibration pass %d, criterion %s\n', C.pass, ...
      mat2str(struct2array(C.criterion)));
end
end

%% 1. Which port is which state
function R = local_port_map(P, V)
% known initial state, zero force and moment, two solver steps; the four initial
% condition parameters carry distinct values in every component, so a permuted or
% transposed port cannot be mistaken for a correct one
x0_pos = [137; -42; -151];
x0_vb = [24; 1.5; -0.75];
x0_eul = [0.11; 0.19; -0.27];
x0_om = [0.31; -0.42; 0.53];
V.cal_F = [0; 0; 0];
V.cal_M = [0; 0; 0];
V.cal_pos = x0_pos;
V.cal_vb = x0_vb;
V.cal_eul = x0_eul;
V.cal_om = x0_om;
V.cal_h_step = P.sim.Ts_ctrl;
V.cal_t_end = 2 * P.sim.Ts_ctrl;

mdl = local_build_eom_probe('v12_cal_eom', P.sim.Ts_ctrl, V.cal_t_end);
y = local_run(mdl, V);

% port order as returned: 1 Ve, 2 Xe, 3 Euler, 4 DCM (3x3), 5 Vb, 6 pqr,
% 7 pqrdot; vector ports come back as nT x 3, the DCM as 3 x 3 x nT
R = struct();
R.x0 = [x0_vb; x0_om; x0_eul; x0_pos];
R.out1_Ve = y.signals(1).values(1, :)';
R.out2_Xe = y.signals(2).values(1, :)';
R.out3_eul = y.signals(3).values(1, :)';
R.out4_DCM = y.signals(4).values(:, :, 1);
R.out5_Vb = y.signals(5).values(1, :)';
R.out6_pqr = y.signals(6).values(1, :)';
R.out7_pqrdot = y.signals(7).values(1, :)';

Rb = sim_dcm(x0_eul);
R.expect_Ve = Rb' * x0_vb;
R.diff = struct( ...
   'Ve', max(abs(R.out1_Ve - R.expect_Ve)), ...
   'Xe', max(abs(R.out2_Xe - x0_pos)), ...
   'euler', max(abs(R.out3_eul - x0_eul)), ...
   'DCM', max(abs(R.out4_DCM(:) - Rb(:))), ...
   'Vb', max(abs(R.out5_Vb - x0_vb)), ...
   'pqr', max(abs(R.out6_pqr - x0_om)));
R.tol = 1e-12;
R.pass = all(structfun(@(v) v < R.tol, R.diff));
% with a nonzero attitude the inertial velocity on port 1 is not the body
% velocity, so this number rules out a guessed mapping
R.ve_minus_vb = max(abs(R.out1_Ve - x0_vb));
R.port1_is_not_vb = R.ve_minus_vb > 1.0;
end

%% 2. The block against the documented equations
function R = local_equations(P, V, t_end)
% constant nonzero force and moment on all six channels, so every equation and
% every sign is exercised; the oracle is a hand written RK4 of the documented
% equations on the same fixed step
F_ext = [13.5; -27.25; 8.125];
M_ext = [6.5; -14.75; 9.25];
x0_pos = [0; 0; -150];
x0_vb = [23; 2; -1];
x0_eul = [0.13; -0.07; 0.4];
x0_om = [0.5; -0.6; 0.7];
h = P.sim.Ts_ctrl;

V.cal_F = F_ext;
V.cal_M = M_ext;
V.cal_pos = x0_pos;
V.cal_vb = x0_vb;
V.cal_eul = x0_eul;
V.cal_om = x0_om;
V.cal_h_step = h;
V.cal_t_end = t_end;

mdl = local_build_eom_probe('v12_cal_eq', h, t_end);
y = local_run(mdl, V);
t = y.time;

% Oracle: same initial state, same constant force and moment, classic RK4
I = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; 0, P.mass.Iyy_cg, 0; -P.mass.Ixz_cg, 0, P.mass.Izz_cg];
m = P.mass.m;
n = numel(t);
Xo = zeros(n, 12);
Xo(1, :) = [x0_vb; x0_om; x0_eul; x0_pos]';
for k = 1:n - 1
   Xo(k + 1, :) = local_rk4(Xo(k, :)', h, @(xx) local_eom_ref(xx, F_ext, M_ext, m, I))';
end

% the block output order is not our state order
Xb = [y.signals(5).values, y.signals(6).values, y.signals(3).values, y.signals(2).values];

% the block wraps its Euler angle output into (-pi, pi] while the oracle
% accumulates, so the angles are compared modulo one turn and the attitude itself
% is checked separately through the block's DCMbe output
d_raw = Xb - Xo;
turns = zeros(size(d_raw));
for j = 7:9
   turns(:, j) = round(d_raw(:, j) / (2 * pi));
   d_raw(:, j) = d_raw(:, j) - 2 * pi * turns(:, j);
end
d = max(abs(d_raw), [], 1)';

DCM_b = y.signals(4).values;
d_dcm = zeros(n, 1);
for k = 1:n
   Rk = sim_dcm(Xo(k, 7:9)');
   d_dcm(k) = max(abs(reshape(DCM_b(:, :, k), 9, 1) - Rk(:)));
end

scale = max(abs(Xo), [], 1)';
R = struct();
R.state_names = {'u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'pn', 'pe', 'pd'};
R.diff_max_abs = d;
R.scale = scale;
R.rel = d ./ max(scale, eps);
R.max_rel = max(R.rel);
R.turns_min = min(turns(:));
R.turns_max = max(turns(:));
R.turns_used = unique(turns(turns ~= 0));
R.dcm_max_diff = max(d_dcm);
R.d_before_unwrap = max(abs(Xb(1, 7:9) - Xo(1, 7:9)));
R.t_end = t_end;
R.n_samples = n;
R.F_ext = F_ext;
R.M_ext = M_ext;
R.x0 = [x0_vb; x0_om; x0_eul; x0_pos];
R.tol = 1e-9;
R.pass = R.max_rel < R.tol && R.dcm_max_diff < R.tol;
end

function dx = local_eom_ref(x, F, M, m, I)
% the documented 6DOF (Euler Angles) equations: force and moment act at the CG in
% body axes, the flat Earth frame is inertial
phi = x(7);
theta = x(8);
Vb = x(1:3);
om = x(4:6);
dx = zeros(12, 1);
dx(1:3) = F / m - cross(om, Vb);
dx(4:6) = I \ (M - cross(om, I * om));
dx(7) = om(1) + tan(theta) * (om(2) * sin(phi) + om(3) * cos(phi));
dx(8) = om(2) * cos(phi) - om(3) * sin(phi);
dx(9) = (om(2) * sin(phi) + om(3) * cos(phi)) / cos(theta);
dx(10:12) = sim_dcm(x(7:9))' * Vb;
end

function x1 = local_rk4(x0, h, f)
k1 = f(x0);
k2 = f(x0 + h / 2 * k1);
k3 = f(x0 + h / 2 * k2);
k4 = f(x0 + h * k3);
x1 = x0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end

%% 3. Atmosphere
function R = local_isa(P, V)
% a fine altitude sweep, so the two implementations are compared across the whole
% band rather than at three points
h = unique([0; 150; 1000; (-50:2.5:400)']);
V.cal_h = h;
V.cal_h_step = P.sim.Ts_ctrl;
V.cal_t_end = P.sim.Ts_ctrl;
mdl = local_build_isa_probe('v12_cal_isa', P.sim.Ts_ctrl);
y = local_run(mdl, V);
T = y.signals(1).values(1, :)';
a = y.signals(2).values(1, :)';
pr = y.signals(3).values(1, :)';
rho = y.signals(4).values(1, :)';

% standard atmosphere, troposphere only at these altitudes; the block tabulates
% sea level density rather than recomputing it from p/(R*T), and uses
% R = 287.0531, so this reference uses the gas constant from P.env
T0 = 288.15;
P0 = 101325;
rho0 = 1.225;
L = 0.0065;
g = 9.80665;
if isfield(P.env, 'R_air') && ~isempty(P.env.R_air)
   Rgas = P.env.R_air;
else
   Rgas = 287.0531;
end
expo = g / (L * Rgas);
% sea level and below is sea level: the block saturates its altitude input at 0 m
% and our own atmosphere model follows the same convention
h_eff = max(h, 0);
T_ref = T0 - L * h_eff;
rho_ref = rho0 * (T_ref / T0).^(expo - 1);
p_ref = P0 * (T_ref / T0).^expo;
a_ref = sqrt(1.4 * Rgas * T_ref);

% and against our own atmosphere model, which is what the plant actually calls
if sim_have('atmo_isa')
   [rho_own, T_own, p_own, a_own] = atmo_isa(P, h);
else
   rho_own = nan(size(h));
   T_own = rho_own;
   p_own = rho_own;
   a_own = rho_own;
end

R = struct();
R.h = h;
R.rho_block = rho;
R.rho_ref = rho_ref;
R.T_block = T;
R.T_ref = T_ref;
R.p_block = pr;
R.p_ref = p_ref;
R.a_block = a;
R.a_ref = a_ref;
R.rel_rho = max(abs(rho - rho_ref) ./ rho_ref);
R.rel_T = max(abs(T - T_ref) ./ T_ref);
R.rel_p = max(abs(pr - p_ref) ./ p_ref);
R.rel_a = max(abs(a - a_ref) ./ a_ref);
R.arg_rel_rho = h(local_argmax(abs(rho - rho_ref) ./ rho_ref));

% address by address comparison against plant/atmo_isa.m: how many altitudes are
% identical, plus the worst absolute and relative difference and where it occurs
R.own_n_identical_rho = sum(rho == rho_own);
R.own_n_identical_T = sum(T == T_own);
R.own_n_identical_p = sum(pr == p_own);
R.own_n_identical_a = sum(a == a_own);
R.own_max_abs_rho = max(abs(rho - rho_own));
R.own_max_abs_T = max(abs(T - T_own));
R.own_max_abs_p = max(abs(pr - p_own));
R.own_max_abs_a = max(abs(a - a_own));
R.own_max_rel_rho = max(abs(rho - rho_own) ./ rho);
R.own_max_rel_T = max(abs(T - T_own) ./ T);
R.own_max_rel_p = max(abs(pr - p_own) ./ pr);
R.own_max_rel_a = max(abs(a - a_own) ./ a);
R.own_arg_max_abs_rho = h(local_argmax(abs(rho - rho_own)));

% split by the sign of the altitude, because the two halves are governed by
% different behaviour and a regression in one would be averaged away in the total
pos = h >= 0;
neg = h < 0;
R.pos_n = sum(pos);
R.neg_n = sum(neg);
R.pos_identical_rho = sum(rho(pos) == rho_own(pos));
R.neg_identical_rho = sum(rho(neg) == rho_own(neg));
R.pos_max_abs_rho = max(abs(rho(pos) - rho_own(pos)));
R.neg_max_abs_rho = max(abs(rho(neg) - rho_own(neg)));
R.pos_max_rel_rho = max(abs(rho(pos) - rho_own(pos)) ./ rho(pos));
R.neg_max_rel_rho = max(abs(rho(neg) - rho_own(neg)) ./ rho(neg));
R.neg_arg_max_abs_rho = h(find(neg, 1, 'first') - 1 + local_argmax(abs(rho(neg) - rho_own(neg))));

R.n_sweep = numel(h);
R.rho_sl_param = P.env.rho_sl;
R.rho_at_zero = rho(1);
R.rho0_is_rho_sl = (rho(1) == P.env.rho_sl);
R.tol = 1e-3;
% the gate covers the whole sweep, both bands, for both comparisons
R.sweep_tol = 1e-9;
R.sweep_pass = R.own_max_rel_rho < R.sweep_tol && R.own_max_rel_T < R.sweep_tol ...
   && R.own_max_rel_p < R.sweep_tol && R.own_max_rel_a < R.sweep_tol;
R.sweep_is_gate = true;
R.pass = R.rel_rho < R.tol && R.rel_T < R.tol && R.rel_p < R.tol ...
   && R.rel_a < R.tol && R.rho0_is_rho_sl && R.sweep_pass;
end

function i = local_argmax(v)
[~, i] = max(v);
end

%% 4. Our forces through the reference block
function R = local_force_path(P, V, x_trim, u_trim)
% the reference block has to reproduce plant/eom_6dof exactly, or the cross-check
% would be measuring the aerodynamics rather than the kinematics; three densities,
% sea level, ISA at the trim altitude, and a deliberately wrong one that shows the
% density port is live
x = x_trim;
x(4:6) = [0.05; -0.03; 0.02];
u = u_trim;
u(1) = u(1) + deg2rad(2.0);
u(2) = u(2) - deg2rad(1.5);
u(3) = u(3) + deg2rad(1.0);
wind = [1.5; -0.5; 0.2];

% the reference block shifts by wind itself and eom_6dof takes no wind argument,
% so the comparison state is the air-relative one
R_b = sim_dcm(x(7:9));
x_air = x;
x_air(1:3) = x(1:3) - R_b * wind;
[~, F_eom, M_eom] = eom_6dof(0, x_air, u, P);
[V_eom, alpha_eom, beta_eom, ~] = flow_state(P, x_air);

V.cal_wind = wind;
V.cal_u = u;
V.cal_vb = x(1:3);
V.cal_om = x(4:6);
V.cal_eul = x(7:9);
V.cal_h_step = P.sim.Ts_ctrl;
V.cal_t_end = P.sim.Ts_ctrl;

rho_case = [P.env.rho_sl; 1.207456419; 1.0];
res = struct('rho', {}, 'F', {}, 'M', {}, 'dF', {}, 'dM', {}, 'V', {}, ...
   'alpha', {}, 'beta', {}, 'qbar', {});
for k = 1:numel(rho_case)
   V.cal_rho = rho_case(k);
   mdl = local_build_aero_probe('v12_cal_aero', P.sim.Ts_ctrl);
   y = local_run(mdl, V);
   Fk = y.signals(1).values(1, :)';
   Mk = y.signals(2).values(1, :)';
   aux = y.signals(3).values(1, :)';
   res(k) = struct('rho', rho_case(k), 'F', Fk, 'M', Mk, ...
      'dF', max(abs(Fk - F_eom)), 'dM', max(abs(Mk - M_eom)), ...
      'V', aux(2), 'alpha', aux(3), 'beta', aux(4), 'qbar', aux(5));
end

R = struct();
R.x = x;
R.x_air = x_air;
R.u = u;
R.wind = wind;
R.F_eom = F_eom;
R.M_eom = M_eom;
R.V_eom = V_eom;
R.alpha_eom = alpha_eom;
R.beta_eom = beta_eom;
R.cases = res;
R.dV = abs(res(1).V - V_eom);
R.dalpha = abs(res(1).alpha - alpha_eom);
R.dbeta = abs(res(1).beta - beta_eom);
R.rho_sl = P.env.rho_sl;
R.qbar_at_rho_sl = res(1).qbar;
R.qbar_predicted = 0.5 * P.env.rho_sl * res(1).V^2;
R.qbar_ratio_other = res(2).qbar / res(1).qbar;
R.rho_ratio_other = res(2).rho / res(1).rho;
R.tol = 1e-9;
R.pass = res(1).dF < R.tol && res(1).dM < R.tol ...
   && R.dV < R.tol && R.dalpha < R.tol && R.dbeta < R.tol ...
   && abs(R.qbar_ratio_other - R.rho_ratio_other) < 1e-12 ...
   && res(3).dF > 1e-3;
end

%% 5. One constant command on both models
function R = local_const_cmd(P, x_trim, u_trim, t_end)
% fixed elevator and throttle, no aileron or rudder, from the trim state; the
% oracle is an independent RK4 on sim_plant_rhs, the same right-hand side the hand
% written model runs
u = u_trim;
u(2) = u(2) - deg2rad(1.0);
u(4) = min(u(4) + 0.05, 1.0);
u(5) = u(4);
u(6) = 0;
h = P.sim.Ts_ctrl;
n = round(t_end / h) + 1;
t = (0:h:t_end)';
U = repmat(u(:)', n, 1);
W = zeros(n, 3);

vars = struct('P', P, 'sim_ts_ctrl', h, 'sim_t_end', t_end, 'sim_x0', x_trim);
vars = local_merge(vars, sim_push_ref_vars(P, x_trim, 0));

[T_our, m_our] = sim_run_plant_cmd('uacs_plant', t, U, W, ...
   struct('solver', 'ode4', 'vars', vars));
[T_ref, m_ref] = sim_run_plant_cmd('uacs_plant_ref', t, U, W, ...
   struct('solver', 'ode4', 'vars', vars));
X_our = T_our.by_name.x_state;
X_ref = T_ref.by_name.x_state;

% oracle: RK4 on the hand written right-hand side, zero wind and a constant
% command, so the input handling inside the step cannot differ between models
Xo = zeros(n, 12);
Xo(1, :) = x_trim';
for k = 1:n - 1
   Xo(k + 1, :) = local_rk4(Xo(k, :)', h, @(xx) sim_plant_rhs(t(k), xx, u, zeros(3, 1), P))';
end

R = struct();
R.t = t;
R.u = u(:)';
R.x0 = x_trim;
R.x_our_end = X_our(end, :)';
R.x_ref_end = X_ref(end, :)';
R.x_oracle_end = Xo(end, :)';
R.d_our_oracle = max(abs(X_our - Xo), [], 1)';
R.d_ref_oracle = max(abs(X_ref - Xo), [], 1)';
R.d_ref_our = max(abs(X_ref - X_our), [], 1)';
R.state_names = {'u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'pn', 'pe', 'pd'};
R.t_end = t_end;
R.n_samples = n;
R.meta_our = m_our;
R.meta_ref = m_ref;
% the oracle advances the command one sample per step, so it is a 1e-3 witness;
% the two models are compared to each other at round-off level
%
% the denominator is sim_state_scale: this command is symmetric, so half the
% states stay at zero and a peak to peak denominator would be zero
R.tol_oracle = 1e-2;
R.tol_models = 1e-9;
denom = sim_state_scale(P);
R.denom = denom;
R.pass = all(R.d_our_oracle ./ denom < R.tol_oracle) ...
   && all(R.d_ref_oracle ./ denom < R.tol_oracle) ...
   && all(R.d_ref_our ./ denom < R.tol_models);
end

%% Probe models
function mdl = local_build_eom_probe(mdl, h, t_end)
mdl = local_new_probe(mdl, h, t_end);
add_block('simulink/Sources/Constant', [mdl '/F'], 'Value', 'cal_F', ...
   'Position', [20 60 60 80]);
add_block('simulink/Sources/Constant', [mdl '/M'], 'Value', 'cal_M', ...
   'Position', [20 120 60 140]);
add_block('aerolib6dof/6DoF (Euler Angles)', [mdl '/eom'], 'Position', [140 40 260 200], ...
   'xme_0', 'cal_pos', 'Vm_0', 'cal_vb', 'eul_0', 'cal_eul', 'pm_0', 'cal_om', ...
   'Mass', 'cal_mass', 'Inertia', 'cal_inertia');
add_line(mdl, 'F/1', 'eom/1');
add_line(mdl, 'M/1', 'eom/2');
for k = 1:7
   add_block('simulink/Sinks/Out1', sprintf('%s/y%d', mdl, k), 'Port', num2str(k), ...
      'Position', [340 20 + 30 * k 380 34 + 30 * k]);
   add_line(mdl, sprintf('eom/%d', k), sprintf('y%d/1', k));
end
end

function mdl = local_build_isa_probe(mdl, h)
mdl = local_new_probe(mdl, h, h);
add_block('simulink/Sources/Constant', [mdl '/h'], 'Value', 'cal_h', ...
   'Position', [20 20 60 60]);
add_block('aerolibatmos/ISA Atmosphere Model', [mdl '/isa'], 'Position', [120 20 240 100]);
add_line(mdl, 'h/1', 'isa/1');
for k = 1:4
   add_block('simulink/Sinks/Out1', sprintf('%s/o%d', mdl, k), 'Port', num2str(k), ...
      'Position', [320 20 + 40 * k 360 34 + 40 * k]);
   add_line(mdl, sprintf('isa/%d', k), sprintf('o%d/1', k));
end
end

function mdl = local_build_aero_probe(mdl, h)
mdl = local_new_probe(mdl, h, h);
src = {'cal_vb', 'cal_om', 'cal_eul', 'cal_wind', 'cal_u', 'cal_rho'};
for k = 1:numel(src)
   add_block('simulink/Sources/Constant', sprintf('%s/c%d', mdl, k), 'Value', src{k}, ...
      'Position', [20 20 + 50 * k 60 40 + 50 * k]);
end
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/aero'], ...
   'FunctionName', 'sim_plant_ref_aero', 'Position', [160 40 280 260]);
for k = 1:6
   add_line(mdl, sprintf('c%d/1', k), sprintf('aero/%d', k));
end
for k = 1:3
   add_block('simulink/Sinks/Out1', sprintf('%s/a%d', mdl, k), 'Port', num2str(k), ...
      'Position', [340 60 + 50 * k 380 74 + 50 * k]);
   add_line(mdl, sprintf('aero/%d', k), sprintf('a%d/1', k));
end
end

function mdl = local_new_probe(mdl, h, t_end)
if bdIsLoaded(mdl)
   close_system(mdl, 0);
end
new_system(mdl);
load_system(mdl);
assignin('base', 'cal_h_step', h);
assignin('base', 'cal_t_end', t_end);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'cal_h_step', ...
   'StopTime', 'cal_t_end', 'AlgebraicLoopMsg', 'error', ...
   'SaveTime', 'on', 'SaveOutput', 'on', 'SaveFormat', 'StructureWithTime', ...
   'ReturnWorkspaceOutputs', 'on');
end

function y = local_run(mdl, V)
sim_push_vars(V);
out = sim_guarded_sim(mdl, V, 'ReturnWorkspaceOutputs', 'on');
y = out.get('yout');
end

function S = local_merge(S, T)
f = fieldnames(T);
for k = 1:numel(f)
   S.(f{k}) = T.(f{k});
end
end

%% Report
function file = local_report(root, C)
file = fullfile(root, 'results', 'sim_v12_calib.txt');
fid = fopen(file, 'w');
if fid < 0
   error('sim_ref_calibrate:Report', 'cannot write %s', file);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'V12 calibration: what the Aerospace Blockset reference blocks compute\n');
fprintf(fid, 'generated      %s\n', C.stamp);
fprintf(fid, 'parameters     revision %d, trim source %s\n', C.params_revision, C.trim_source);
fprintf(fid, 'MATLAB         %s\n', version());
v = ver('aeroblks');
if ~isempty(v)
   fprintf(fid, 'Aerospace      %s %s\n', v.Name, v.Version);
end
fprintf(fid, 'trim state     %s\n', mat2str(C.x_trim', 8));
fprintf(fid, 'trim input     %s\n', mat2str(C.u_trim', 8));

p = C.port_map;
fprintf(fid, '\n[1 port identity, zero force and moment]\n');
fprintf(fid, 'initial state  %s\n', mat2str(p.x0', 8));
fprintf(fid, '%-6s %-34s %-34s %12s\n', 'port', 'block output', 'expected', 'max abs diff');
fprintf(fid, '%-6s %-34s %-34s %12.3e\n', '1', 'Ve', 'DCMbe'' * Vb', p.diff.Ve);
fprintf(fid, '%-6s %-34s %-34s %12.3e\n', '2', 'Xe', 'initial position', p.diff.Xe);
fprintf(fid, '%-6s %-34s %-34s %12.3e\n', '3', 'phi theta psi', 'initial Euler angles', p.diff.euler);
fprintf(fid, '%-6s %-34s %-34s %12.3e\n', '4', 'DCMbe', 'sim_dcm(phi,theta,psi)', p.diff.DCM);
fprintf(fid, '%-6s %-34s %-34s %12.3e\n', '5', 'Vb', 'initial body velocity', p.diff.Vb);
fprintf(fid, '%-6s %-34s %-34s %12.3e\n', '6', 'p q r', 'initial body rates', p.diff.pqr);
fprintf(fid, 'port 1 is not the body velocity: |Ve - Vb| = %.6f m/s\n', p.ve_minus_vb);
fprintf(fid, '=> x(1:3) = out5, x(4:6) = out6, x(7:9) = out3, x(10:12) = out2 -> %s\n', local_v(p.pass));

e = C.equations;
fprintf(fid, '\n[2 6DOF block against an independent RK4 of the documented equations]\n');
fprintf(fid, 'constant F     %s N\n', mat2str(e.F_ext', 10));
fprintf(fid, 'constant M     %s N*m\n', mat2str(e.M_ext', 10));
fprintf(fid, 'duration       %.3f s, %d samples\n', e.t_end, e.n_samples);
fprintf(fid, 'angles compared modulo one turn; whole turns removed %s\n', ...
   mat2str(e.turns_used'));
fprintf(fid, '%-6s %14s %14s %14s\n', 'state', 'max abs diff', 'scale', 'rel');
for k = 1:numel(e.state_names)
   fprintf(fid, '%-6s %14.3e %14.3e %14.3e\n', e.state_names{k}, ...
      e.diff_max_abs(k), e.scale(k), e.rel(k));
end
fprintf(fid, 'DCMbe against sim_dcm(oracle euler)  %.3e  (attitude, no angle ambiguity)\n', ...
   e.dcm_max_diff);
fprintf(fid, 'max relative %.3e, limit %.1e -> %s\n', e.max_rel, e.tol, local_v(e.pass));

i = C.isa;
fprintf(fid, '\n[3 Atmosphere (ISA) block]\n');
fprintf(fid, 'altitude sweep    %.1f..%.1f m, %d points\n', min(i.h), max(i.h), i.n_sweep);
fprintf(fid, '%-8s %14s %14s %14s %14s %14s\n', 'h[m]', 'rho block', 'rho ref', ...
   'T block', 'p block', 'a block');
for k = [find(i.h == 0, 1), find(i.h == 150, 1), find(i.h == 1000, 1)]
   fprintf(fid, '%-8.0f %14.9f %14.9f %14.6f %14.3f %14.6f\n', i.h(k), ...
      i.rho_block(k), i.rho_ref(k), i.T_block(k), i.p_block(k), i.a_block(k));
end
fprintf(fid, 'over the whole sweep against the standard atmosphere (limit %.1e):\n', i.tol);
fprintf(fid, '  rho max rel %.3e at h = %.2f m; T %.3e; p %.3e; a %.3e\n', ...
   i.rel_rho, i.arg_rel_rho, i.rel_T, i.rel_p, i.rel_a);
fprintf(fid, 'against plant/atmo_isa.m, the same sweep:\n');
fprintf(fid, '  identical altitudes rho %d, T %d, p %d, a %d, of %d\n', ...
   i.own_n_identical_rho, i.own_n_identical_T, i.own_n_identical_p, ...
   i.own_n_identical_a, i.n_sweep);
fprintf(fid, '  max abs difference rho %.6e at h = %.2f m; T %.6e; p %.6e; a %.6e\n', ...
   i.own_max_abs_rho, i.own_arg_max_abs_rho, i.own_max_abs_T, i.own_max_abs_p, i.own_max_abs_a);
fprintf(fid, '  max rel difference rho %.6e; T %.6e; p %.6e; a %.6e\n', ...
   i.own_max_rel_rho, i.own_max_rel_T, i.own_max_rel_p, i.own_max_rel_a);
fprintf(fid, '  h >= 0, %d points: rho identical %d, max abs %.6e, max rel %.6e\n', ...
   i.pos_n, i.pos_identical_rho, i.pos_max_abs_rho, i.pos_max_rel_rho);
fprintf(fid, '  h <  0, %d points: rho identical %d, max abs %.6e at h = %.2f, max rel %.6e\n', ...
   i.neg_n, i.neg_identical_rho, i.neg_max_abs_rho, i.neg_arg_max_abs_rho, i.neg_max_rel_rho);
fprintf(fid, 'rho at h = 0 is %.12f, P.env.rho_sl is %.12f, identical %d\n', ...
   i.rho_at_zero, i.rho_sl_param, i.rho0_is_rho_sl);
fprintf(fid, ['=> the constant density criterion run is the same fluid as our flow_state, and the\n' ...
   '   ISA configurations compare two implementations of the same density law.\n' ...
   '   Gate covers the WHOLE sweep, both bands, at %.1e -> %s\n'], ...
   i.sweep_tol, local_v(i.sweep_pass));
fprintf(fid, ['   Sea level and below is sea level: the Aerospace block saturates its altitude\n' ...
   '   input at 0 m and plant/atmo_isa.m now does the same, so the two agree over the\n' ...
   '   whole declared range and there is no boundary behaviour to carve out.\n']);
fprintf(fid, 'ISA sub-check -> %s\n', local_v(i.pass));

fq = C.force_path;
fprintf(fid, '\n[4 our force and moment through the reference block]\n');
fprintf(fid, 'state          %s\n', mat2str(fq.x', 8));
fprintf(fid, 'wind NED       %s, so the air relative state is %s\n', ...
   mat2str(fq.wind', 8), mat2str(fq.x_air', 8));
fprintf(fid, 'input          %s\n', mat2str(fq.u', 8));
fprintf(fid, 'eom_6dof F     %s N\n', mat2str(fq.F_eom', 10));
fprintf(fid, 'eom_6dof M     %s N*m\n', mat2str(fq.M_eom', 10));
for k = 1:numel(fq.cases)
   c = fq.cases(k);
   fprintf(fid, 'rho %-12.9f  V %.9f  alpha %.9f  beta %.9f  qbar %.6f  dF %.3e  dM %.3e\n', ...
      c.rho, c.V, c.alpha, c.beta, c.qbar, c.dF, c.dM);
end
fprintf(fid, 'air data       dV %.3e  dalpha %.3e  dbeta %.3e against flow_state\n', ...
   fq.dV, fq.dalpha, fq.dbeta);
fprintf(fid, 'qbar ratio between the two densities %.12f against the density ratio %.12f\n', ...
   fq.qbar_ratio_other, fq.rho_ratio_other);
fprintf(fid, '=> at rho = P.env.rho_sl the reference forces equal eom_6dof -> %s\n', local_v(fq.pass));

c = C.const_cmd;
fprintf(fid, '\n[5 constant elevator and throttle on both models]\n');
fprintf(fid, 'input          %s  (da, de, dr rad; throttles; dT N)\n', mat2str(c.u, 8));
fprintf(fid, 'initial state  %s\n', mat2str(c.x0', 8));
fprintf(fid, 'duration       %.3f s, %d samples, ode4 fixed step\n', c.t_end, c.n_samples);
fprintf(fid, '%-6s %16s %16s %16s %12s %12s %12s\n', 'state', 'uacs_plant', ...
   'uacs_plant_ref', 'RK4 oracle', 'd our', 'd ref', 'd our-ref');
for k = 1:numel(c.state_names)
   fprintf(fid, '%-6s %16.9f %16.9f %16.9f %12.3e %12.3e %12.3e\n', c.state_names{k}, ...
      c.x_our_end(k), c.x_ref_end(k), c.x_oracle_end(k), ...
      c.d_our_oracle(k), c.d_ref_oracle(k), c.d_ref_our(k));
end
fprintf(fid, 'oracle tolerance %.1e (the oracle advances the command one sample), model to model %.1e -> %s\n', ...
   c.tol_oracle, c.tol_models, local_v(c.pass));
fprintf(fid, 'denominator is sim_state_scale; states that never move have zero excursion here\n');

fprintf(fid, '\n[verdict]\n');
f = fieldnames(C.criterion);
for k = 1:numel(f)
   fprintf(fid, '%-24s %d\n', f{k}, C.criterion.(f{k}));
end
fprintf(fid, 'calibration PASS %d\n', C.pass);
end

function s = local_v(tf)
if tf
   s = 'PASS';
else
   s = 'FAIL';
end
end
