% UACS_JOB_ATMO_WIND Wind frame term and density altitude, plant level evidence.
%
% Job for run_job('atmo_wind'), see MATLAB/run/run_job.m. Seven stages, every
% number measured:
%   A  plant/atmo_isa.m against the Aerospace Blockset ISA Atmosphere Model block
%   B  the density path through plant/flow_state.m
%   C  the wind frame term in sim/sim_plant_rhs.m against Newton's law written in
%      the ground velocity, and the constant-wind translation identity
%   D  trim with the density model switched on
%   E  what the V12 ISA configuration reads for each way of wiring the reference
%      model's ISA block input
%   F  the single engine control speed and the speed envelope at 150 m, which are
%      safety numbers and are sea level numbers everywhere else in the project
%   G  plant/wind_trim.m: the crab trim under a steady wind, as an equilibrium of
%      the plant, against the analytic heading, and bit exact at zero wind
%
% Writes results/atmo_wind_report.txt and results/atmo_wind_check.mat.
% Manual run: run_job('atmo_wind','owner','ash-atmo') with MATLAB/run,
% MATLAB/plant, MATLAB/params and MATLAB/sim on the path.

here = fileparts(mfilename('fullpath'));
if isempty(here)
   here = fileparts(which('uacs_job_atmo_wind'));
end
if isempty(here)
   here = pwd;
end
root = fileparts(here);
addpath(fullfile(root, 'run'));
addpath(fullfile(root, 'plant'), fullfile(root, 'params'), fullfile(root, 'sim'), '-begin');
res_dir = fullfile(root, 'results');
stamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
pass_words = {'FAIL', 'PASS'};
pass_text = @(tf) pass_words{double(tf) + 1};

init_uacs_twin('quiet', true);
if ~exist('P', 'var')
   error('atmo_wind: init_uacs_twin did not push P into the base workspace');
end

fid = fopen(fullfile(res_dir, 'atmo_wind_report.txt'), 'w');
fprintf(fid, 'UACS twin: wind frame term and density altitude, plant level evidence\n');
fprintf(fid, 'generated %s, MATLAB %s\n', stamp, version());
fprintf(fid, 'params %s rev %d (%s)\n', P.meta.version, P.meta.revision, P.meta.date);
fprintf(fid, 'files sim/sim_plant_rhs.m, plant/flow_state.m, plant/atmo_isa.m, params/uacs_twin_params.m\n');
fprintf(fid, 'density model P.atmo.model = ''%s'', h_from_state %d, h_ref %.3f m\n', ...
   P.atmo.model, P.atmo.h_from_state, P.atmo.h_ref);
fprintf(fid, 'wind applied at the model boundary, dx(1:3) carries the omega x (R*W) term\n');

%% ---------------- A. ISA against the Aerospace Blockset block ----------------
fprintf('\n[atmo_wind] A: atmo_isa against the ISA Atmosphere Model block\n');
fprintf(fid, '\n[A  atmo_isa against the Aerospace Blockset ISA Atmosphere Model block]\n');
A = struct();
% The altitude list is the one the V12 calibration actually measured the block at,
% read back from its result file; hardcoding 0/150/1000 broke once when that list
% grew into a sweep.
calib_file = fullfile(res_dir, 'sim_v12_calib.mat');
A.block_source = calib_file;
if exist(calib_file, 'file') == 2
   Sc = load(calib_file);
   C = Sc.C;
   A.h_block = C.isa.h(:);
else
   A.h_block = [0; 150; 1000];
end
A.h = A.h_block;
[rho_m, T_m, p_m, a_m] = atmo_isa(P, A.h);
A.rho_model = rho_m;
A.T_model = T_m;
A.p_model = p_m;
A.a_model = a_m;
A.rho_sl_exact = (atmo_isa(P, 0) == P.env.rho_sl);
if exist(calib_file, 'file') == 2
   A.rho_block = C.isa.rho_block(:);
   A.T_block = C.isa.T_block(:);
   A.p_block = C.isa.p_block(:);
   A.a_block = C.isa.a_block(:);
   assert(numel(A.rho_block) == numel(A.h_block), 'atmo_wind:CalibSize', ...
      'the V12 calibration file has %d altitudes but %d densities', ...
      numel(A.h_block), numel(A.rho_block));
   rel = @(x, y) max(abs(x(:) - y(:)) ./ abs(y(:)));
   A.rel_rho = rel(rho_m, A.rho_block);
   A.rel_T = rel(T_m, A.T_block);
   A.rel_p = rel(p_m, A.p_block);
   A.rel_a = rel(a_m, A.a_block);
   A.max_rel = max([A.rel_rho, A.rel_T, A.rel_p, A.rel_a]);
   A.exact_all = isequal(rho_m, A.rho_block) && isequal(T_m, A.T_block) ...
      && isequal(p_m, A.p_block) && isequal(a_m, A.a_block);
   A.n_exact_rho = sum(rho_m == A.rho_block);
   % Split at sea level. The reference block saturates its altitude input at 0 m,
   % so below sea level it holds the sea level state while plant/atmo_isa.m keeps
   % extending the lapse rate. The flight envelope is at or above 0 m, so the part
   % the two models actually share is the upper segment, and the lower segment is
   % reported in full rather than averaged away.
   A.mask_ge0 = A.h_block >= 0;
   A.mask_lt0 = ~A.mask_ge0;
   A.n_ge0 = sum(A.mask_ge0);
   A.n_lt0 = sum(A.mask_lt0);
   A.rel_rho_ge0 = rel(rho_m(A.mask_ge0), A.rho_block(A.mask_ge0));
   A.rel_T_ge0 = rel(T_m(A.mask_ge0), A.T_block(A.mask_ge0));
   A.rel_p_ge0 = rel(p_m(A.mask_ge0), A.p_block(A.mask_ge0));
   A.rel_a_ge0 = rel(a_m(A.mask_ge0), A.a_block(A.mask_ge0));
   A.max_rel_ge0 = max([A.rel_rho_ge0, A.rel_T_ge0, A.rel_p_ge0, A.rel_a_ge0]);
   A.n_exact_rho_ge0 = sum(rho_m(A.mask_ge0) == A.rho_block(A.mask_ge0));
   A.clamp_test = true;
   if A.n_lt0 > 0
      A.rel_rho_lt0 = rel(rho_m(A.mask_lt0), A.rho_block(A.mask_lt0));
      A.n_exact_rho_lt0 = sum(rho_m(A.mask_lt0) == A.rho_block(A.mask_lt0));
      A.block_values_lt0 = unique(A.rho_block(A.mask_lt0));
      % Saturation is proved, not assumed: every negative altitude has to come back
      % as exactly the sea level density the block tabulates at h = 0.
      A.clamp_test = all(A.block_values_lt0 == P.env.rho_sl);
      A.h_lt0_range = [min(A.h_block(A.mask_lt0)), max(A.h_block(A.mask_lt0))];
   else
      A.rel_rho_lt0 = 0;
      A.n_exact_rho_lt0 = 0;
      A.block_values_lt0 = [];
      A.h_lt0_range = [0, 0];
   end
   A.pass = A.rel_rho_ge0 < 1e-12 && A.rho_sl_exact ...
      && (A.n_lt0 == 0 || A.clamp_test);
   fprintf(fid, '  %d altitudes from %s, %.0f to %.0f m\n', numel(A.h_block), ...
      A.block_source, min(A.h_block), max(A.h_block));
   if numel(A.h_block) <= 8
      fprintf(fid, '  h[m]      rho model          rho block          dT[K]     dp[Pa]     da[m/s]\n');
      for k = 1:numel(A.h_block)
         fprintf(fid, '  %-9.0f %.15f %.15f %10.2e %10.2e %10.2e\n', A.h_block(k), ...
            rho_m(k), A.rho_block(k), T_m(k) - A.T_block(k), ...
            p_m(k) - A.p_block(k), a_m(k) - A.a_block(k));
      end
   end
   fprintf(fid, '  all altitudes: max relative rho %.3e, T %.3e, p %.3e, a %.3e\n', ...
      A.rel_rho, A.rel_T, A.rel_p, A.rel_a);
   fprintf(fid, '  h >= 0, %d altitudes: max relative rho %.3e, T %.3e, p %.3e, a %.3e\n', ...
      A.n_ge0, A.rel_rho_ge0, A.rel_T_ge0, A.rel_p_ge0, A.rel_a_ge0);
   fprintf(fid, '  h >= 0: identical densities %d of %d, bit identical on all four %d\n', ...
      A.n_exact_rho_ge0, A.n_ge0, A.exact_all);
   if A.n_lt0 > 0
      fprintf(fid, '  h < 0, %d altitudes from %.1f to %.1f m: max relative rho %.3e, identical %d of %d\n', ...
         A.n_lt0, A.h_lt0_range(1), A.h_lt0_range(2), A.rel_rho_lt0, ...
         A.n_exact_rho_lt0, A.n_lt0);
      fprintf(fid, '  block densities below sea level: %s\n', mat2str(A.block_values_lt0', 16));
      fprintf(fid, '  that is exactly P.env.rho_sl, so the block saturates its altitude at 0 m %d\n', ...
         A.clamp_test);
   else
      fprintf(fid, '  the sweep has no altitude below sea level, so the saturation is not exercised\n');
   end
   fprintf(fid, '  rho at h = 0 is exactly P.env.rho_sl %d\n', A.rho_sl_exact);
   fprintf(fid, '  constants: rho_sl %.6f, T_sl %.2f K, p_sl %.0f Pa, L %.4f K/m, R %.4f, gamma %.1f\n', ...
      P.env.rho_sl, P.env.T_sl, P.env.p_sl, P.env.L_isa, P.env.R_air, P.env.gamma_air);
   fprintf(fid, '  the block and the plant use the same fluid over the flight envelope -> %s\n', ...
      pass_text(A.pass));
else
   A.max_rel = NaN;
   A.max_rel_ge0 = NaN;
   A.pass = false;
   fprintf(fid, '  SKIP: %s is missing, run run_job(''sim_v12'') first\n', calib_file);
end
% Monotone density and the dT_sl offset, independent of the block.
A.monotone = all(diff(atmo_isa(P, (0:100:2000)')) < 0);
A.dT_effect = atmo_isa(P, 150, 10) < atmo_isa(P, 150, 0);
A.trop_guard = false;
try
   atmo_isa(P, 12000);
catch
   A.trop_guard = true;
end
fprintf(fid, '  density falls with altitude %d, +10 K lowers it %d, 12 km is refused %d\n', ...
   A.monotone, A.dT_effect, A.trop_guard);

%% ---------------- B. density path through flow_state ----------------
fprintf('[atmo_wind] B: density path through flow_state\n');
fprintf(fid, '\n[B  density path through plant/flow_state.m]\n');
tr = trim_cruise(P, P.op.cruise);
x0 = tr.x_star(:);
[V_c, al_c, be_c, q_c, rho_c] = flow_state(P, x0);
P_isa = P;
P_isa.atmo.model = 'isa';
P_isa.atmo.h_from_state = true;
[V_i, al_i, be_i, q_i, rho_i] = flow_state(P_isa, x0);
P_fix = P_isa;
P_fix.atmo.h_from_state = false;
P_fix.atmo.h_ref = -x0(12);
[~, ~, ~, q_f, rho_f] = flow_state(P_fix, x0);
B = struct();
B.h_state = -x0(12);
B.rho_const = rho_c;
B.rho_isa_state = rho_i;
B.rho_isa_ref = rho_f;
B.rho_isa_expected = atmo_isa(P, -x0(12));
B.const_is_rho_sl = (rho_c == P.env.rho_sl);
B.isa_is_formula = (rho_i == B.rho_isa_expected);
B.ref_is_state = (rho_f == rho_i);
B.airdata_independent_of_density = isequal([V_c al_c be_c], [V_i al_i be_i]);
B.qbar_ratio = q_i / q_c;
B.density_ratio = rho_i / rho_c;
B.qbar_matches_density = abs(B.qbar_ratio - B.density_ratio) < 1e-15;
B.pass = B.const_is_rho_sl && B.isa_is_formula && B.ref_is_state ...
   && B.airdata_independent_of_density && B.qbar_matches_density;
fprintf(fid, '  trim altitude %.3f m; const rho %.15f (is rho_sl %d)\n', ...
   B.h_state, rho_c, B.const_is_rho_sl);
fprintf(fid, '  isa  rho %.15f, formula %.15f, identical %d\n', ...
   rho_i, B.rho_isa_expected, B.isa_is_formula);
fprintf(fid, '  h_from_state false with h_ref %.1f gives %.15f, identical to state driven %d\n', ...
   P_fix.atmo.h_ref, rho_f, B.ref_is_state);
fprintf(fid, '  V %.12f alpha %.12f beta %.12f do not depend on the density model %d\n', ...
   V_c, al_c, be_c, B.airdata_independent_of_density);
fprintf(fid, '  qbar ratio %.15f against density ratio %.15f -> %s\n', ...
   B.qbar_ratio, B.density_ratio, pass_text(B.pass));

%% ---------------- C. the wind frame term ----------------
fprintf('[atmo_wind] C: wind frame term in sim_plant_rhs\n');
fprintf(fid, '\n[C  wind frame term in sim/sim_plant_rhs.m]\n');
W5 = [0; 5.0; 0];
u_trim = tr.u_star(:);

% C1 one state with both a body rate and a wind: the identity, term by term.
xc = x0;
xc(4:6) = [0.06; -0.04; 0.03];
dx_sim = sim_plant_rhs(0, xc, u_trim, W5, P);
Rc = sim_dcm(xc(7:9));
Wb = Rc * W5;
xc_air = xc;
xc_air(1:3) = xc(1:3) - Wb;
[dx_air, F_air, M_air] = eom_6dof(0, xc_air, u_trim, P);
oracle = dx_air;
oracle(1:3) = F_air / P.mass.m - cross(xc(4:6), xc(1:3));   % Newton in v_ground
oracle(10:12) = Rc' * xc(1:3);
old_form = dx_air;                                          % the pre rev 10 right-hand side
old_form(10:12) = Rc' * xc(1:3);
C1 = struct();
C1.x = xc;
C1.wind_ned = W5;
C1.term = cross(xc(4:6), Wb);
C1.res_after = max(abs(dx_sim - oracle));
C1.res_before = max(abs(old_form - oracle));
C1.term_magnitude = max(abs(C1.term));
C1.position_ground = max(abs(dx_sim(10:12) - Rc' * xc(1:3)));
C1.position_air = max(abs(dx_sim(10:12) - Rc' * xc_air(1:3)));
C1.pass = C1.res_after < 1e-12 && C1.res_before > 1e-9 && C1.position_ground == 0;
fprintf(fid, '  state with omega [%.4f %.4f %.4f] rad/s and wind %s m/s NED\n', ...
   xc(4), xc(5), xc(6), mat2str(W5'));
fprintf(fid, '  omega x (R*W) body axes           %s\n', mat2str(C1.term', 8));
fprintf(fid, '  |dx(1:3) - Newton in v_ground| after rev 10   %.3e\n', C1.res_after);
fprintf(fid, '  same against the rev 9 formula                 %.3e  (the term that was missing)\n', C1.res_before);
fprintf(fid, '  dx(10:12) against C_bn*v_ground %.3e, against C_bn*v_air %.3e\n', ...
   C1.position_ground, C1.position_air);
fprintf(fid, '  -> %s\n', pass_text(C1.pass));

% C2 trajectory level: sim_plant_rhs against an independent integration of the
% ground frame equations, both driven by the same commands, in a crosswind.
t_c2 = (0:P.sim.Ts_ctrl:5.0)';
U_c2 = local_doublet(u_trim, t_c2);
X_fix = local_ode4(@(t, x, u) sim_plant_rhs(t, x, u, W5, P), t_c2, x0, U_c2);
X_old = local_ode4(@(t, x, u) local_rhs_old(t, x, u, W5, P), t_c2, x0, U_c2);
X_ora = local_ode4(@(t, x, u) local_rhs_oracle(t, x, u, W5, P), t_c2, x0, U_c2);
C2 = struct();
C2.t_end = t_c2(end);
C2.max_abs_fixed_vs_oracle = max(abs(X_fix(:) - X_ora(:)));
C2.max_abs_old_vs_oracle = max(abs(X_old(:) - X_ora(:)));
C2.max_rel_fixed_vs_oracle = max(local_rel(X_fix, X_ora, P));
C2.max_rel_old_vs_oracle = max(local_rel(X_old, X_ora, P));
C2.pass = C2.max_abs_fixed_vs_oracle < 1e-9 && C2.max_abs_old_vs_oracle > 1e-3;
fprintf(fid, '  %d s doublet in a 5 m/s east wind, ode4 at %.4f s, fixed vs oracle %.3e abs (%.3e rel)\n', ...
   C2.t_end, P.sim.Ts_ctrl, C2.max_abs_fixed_vs_oracle, C2.max_rel_fixed_vs_oracle);
fprintf(fid, '  the same run with the rev 9 formula against the oracle     %.3e abs (%.3e rel)\n', ...
   C2.max_abs_old_vs_oracle, C2.max_rel_old_vs_oracle);
fprintf(fid, '  -> %s\n', pass_text(C2.pass));
C2.x_fixed = X_fix;
C2.x_old = X_old;
C2.x_oracle = X_ora;

% C3 constant wind is a translation of the still-air trajectory: the ground
% velocity gains C_nb*W, the attitude and the rates are untouched, and the NED
% position gains exactly W*t. The body velocity states are ground velocity, so
% they are shifted and not compared directly.
t_c3 = (0:P.sim.Ts_ctrl:5.0)';
u_const = repmat(u_trim', numel(t_c3), 1);
R0 = sim_dcm(x0(7:9));
offset = (R0 * W5)';
x0_wind = x0;
x0_wind(1:3) = x0(1:3) + R0 * W5;
X_still = local_ode4(@(t, x, u) sim_plant_rhs(t, x, u, zeros(3, 1), P), t_c3, x0, u_const);
X_wind = local_ode4(@(t, x, u) sim_plant_rhs(t, x, u, W5, P), t_c3, x0_wind, u_const);
X_pred = X_still;
X_pred(:, 1:3) = X_still(:, 1:3) + repmat(offset, numel(t_c3), 1);
X_pred(:, 11) = X_still(:, 11) + W5(2) * t_c3;
C3 = struct();
C3.t = t_c3;
C3.velocity_offset = offset;
C3.max_abs_attitude = max(max(abs(X_wind(:, 4:9) - X_still(:, 4:9))));
C3.max_abs_velocity_offset = max(max(abs((X_wind(:, 1:3) - X_still(:, 1:3)) ...
   - repmat(offset, numel(t_c3), 1))));
C3.max_abs_pred = max(max(abs(X_wind - X_pred)));
C3.east_gain = X_wind(end, 11) - X_still(end, 11);
C3.east_expected = W5(2) * t_c3(end);
C3.pass = C3.max_abs_pred < 1e-9 && C3.max_abs_attitude < 1e-9 ...
   && C3.max_abs_velocity_offset < 1e-9 && abs(C3.east_gain - C3.east_expected) < 1e-9;
fprintf(fid, '  still air against 5 m/s wind over 5 s, same air data:\n');
fprintf(fid, '    attitude and rates differ by                    %.3e\n', C3.max_abs_attitude);
fprintf(fid, '    ground velocity against v_still + C_nb*W %s  %.3e\n', ...
   mat2str(offset, 6), C3.max_abs_velocity_offset);
fprintf(fid, '    full state against still air plus W*t           %.3e\n', C3.max_abs_pred);
fprintf(fid, '    east position gain %.9f m against %.9f m predicted\n', ...
   C3.east_gain, C3.east_expected);
fprintf(fid, '  -> %s\n', pass_text(C3.pass));

%% ---------------- D. trim with the density model on ----------------
fprintf('[atmo_wind] D: trim at 150 m with ISA density\n');
fprintf(fid, '\n[D  trim with the density model switched on]\n');
tr_isa = trim_cruise(P_isa, P.op.cruise);
tr_fix = trim_cruise(P_fix, P.op.cruise);
D = struct();
D.h_m = tr.h_m;
D.rho_const = rho_c;
D.rho_isa = rho_i;
D.density_drop_pct = 100 * (1 - rho_i / rho_c);
D.trim_const = tr;
D.trim_isa = tr_isa;
D.alpha_deg_const = tr.alpha_deg;
D.alpha_deg_isa = tr_isa.alpha_deg;
D.de_deg_const = tr.de_deg;
D.de_deg_isa = tr_isa.de_deg;
D.throttle_const = tr.throttle;
D.throttle_isa = tr_isa.throttle;
D.res_const = tr.residual_norm_max;
D.res_isa = tr_isa.residual_norm_max;
D.ref_equals_state = abs(tr_fix.alpha_rad - tr_isa.alpha_rad) < 1e-14 ...
   && abs(tr_fix.de_rad - tr_isa.de_rad) < 1e-14;
% The same state cannot balance in two fluids: balanced under the ISA density,
% out of balance by the density step under the frozen sea level one.
[~, Fi, Mi] = eom_6dof(0, tr_isa.x_star, tr_isa.u_star, P_isa);
[~, Fc, Mc] = eom_6dof(0, tr_isa.x_star, tr_isa.u_star, P);
D.res_isa_trim_with_isa_density = max(abs([Fi / tr_isa.norm_force_N; Mi / tr_isa.norm_moment_Nm]));
D.res_isa_trim_with_const_density = max(abs([Fc / tr_isa.norm_force_N; Mc / tr_isa.norm_moment_Nm]));
D.res_const_trim_with_const_density = tr.residual_norm_max;
% trim_cruise must report the density the plant used: the ISA value under the ISA
% model, and P.env.rho_sl still exactly under the constant model.
D.trim_rho_is_isa = (tr_isa.rho == rho_i);
D.const_trim_rho_is_rho_sl = (tr.rho == rho_c);
D.pass = D.res_isa < 1e-6 && D.res_isa_trim_with_const_density > 1e-3 ...
   && D.ref_equals_state && tr_isa.alpha_deg > tr.alpha_deg ...
   && D.trim_rho_is_isa && D.const_trim_rho_is_rho_sl;
fprintf(fid, '  density %.6f against %.6f kg/m^3, drop %.3f percent\n', ...
   rho_i, rho_c, D.density_drop_pct);
fprintf(fid, '  const: alpha %.6f deg, de %.6f deg, throttle %.6f, |r|max %.3e\n', ...
   tr.alpha_deg, tr.de_deg, tr.throttle, tr.residual_norm_max);
fprintf(fid, '  isa  : alpha %.6f deg, de %.6f deg, throttle %.6f, |r|max %.3e\n', ...
   tr_isa.alpha_deg, tr_isa.de_deg, tr_isa.throttle, tr_isa.residual_norm_max);
fprintf(fid, '  fixed reference altitude gives the same trim %d\n', D.ref_equals_state);
fprintf(fid, '  the ISA trim state balances at %.3e in ISA density and at %.3e in sea level density\n', ...
   D.res_isa_trim_with_isa_density, D.res_isa_trim_with_const_density);
fprintf(fid, '  trim.rho reports %.15f (isa) and %.15f (const), the densities actually used %d\n', ...
   tr_isa.rho, tr.rho, D.trim_rho_is_isa && D.const_trim_rho_is_rho_sl);
fprintf(fid, '  -> %s\n', pass_text(D.pass));

%% ---------------- E. what the V12 ISA configuration will read ----------------
fprintf('[atmo_wind] E: V12 ISA configuration projection\n');
fprintf(fid, '\n[E  V12 ISA configuration, projected from our own plant]\n');
t_e = (0:P.sim.Ts_ctrl:9.0)';
U_e = local_doublet(u_trim, t_e);
rhs = @(t, x, u, W, PP) sim_plant_rhs(t, x, u, W, PP);
X_e_const = local_ode4(@(t, x, u) rhs(t, x, u, zeros(3, 1), P), t_e, x0, U_e);
% Frozen density at the trim altitude: exactly what uacs_plant_ref.slx does today,
% its ISA block input being a constant 150 m. The override goes through the aero
% path only, because P.env.rho_sl still feeds the thrust map in both models.
P_z150 = P;
P_z150.atmo.model = 'isa';
P_z150.atmo.h_from_state = false;
P_z150.atmo.h_ref = -x0(12);
X_e_z150 = local_ode4(@(t, x, u) rhs(t, x, u, zeros(3, 1), P_z150), t_e, x0, U_e);
X_e_isa = local_ode4(@(t, x, u) rhs(t, x, u, zeros(3, 1), P_isa), t_e, x0, U_e);
rel_z150 = local_rel(X_e_const, X_e_z150, P);
rel_feedback = local_rel(X_e_const, X_e_isa, P);
rel_wiring = local_rel(X_e_z150, X_e_isa, P);
E = struct();
E.t_end = t_e(end);
E.metric = 'max over states of max|x_a-x_b| / ptp of the const density run';
E.rel_const_to_z150 = max(rel_z150);
E.rel_const_to_isa_feedback = max(rel_feedback);
E.rel_z150_to_isa_feedback = max(rel_wiring);
E.state_names = {'u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'pn', 'pe', 'pd'};
E.rel_const_to_z150_by_state = rel_z150;
E.rel_z150_to_isa_feedback_by_state = rel_wiring;
E.v12_isa_trim_measured = local_v12_number(res_dir, 'isa_trim');
E.v12_crosswind_measured = local_v12_number(res_dir, 'crosswind');
E.pass = true;
fprintf(fid, '  metric %s\n', E.metric);
fprintf(fid, '  const density against a frozen ISA density at 150 m   %.6e\n', E.rel_const_to_z150);
fprintf(fid, '  const density against density altitude from the state  %.6e\n', E.rel_const_to_isa_feedback);
fprintf(fid, '  frozen ISA(150) against density altitude from state    %.6e\n', ...
   E.rel_z150_to_isa_feedback);
if ~isnan(E.v12_isa_trim_measured)
   fprintf(fid, '  V12 measured isa_trim %.6e, this run reproduces the same law to a factor %.3f\n', ...
      E.v12_isa_trim_measured, E.rel_const_to_z150 / E.v12_isa_trim_measured);
end
fprintf(fid, [ '  reading: the residual between the two density laws is what the reference\n' ...
   '  model has to close. If its ISA block input stays a constant 150 m the ISA\n' ...
   '  configuration lands at %.3e; driven by -pd from the 6DOF position output it\n' ...
   '  lands at the integrator level, the same order as const_density.\n'], ...
   E.rel_z150_to_isa_feedback);

%% ---------------- F. V_mc and the envelope at the ISA density ----------------
fprintf('[atmo_wind] F: V_mc and the speed envelope at 150 m ISA\n');
fprintf(fid, '\n[F  single engine control speed and envelope at the ISA density]\n');
F = struct();
F.h_m = -x0(12);
F.rho_sl = rho_c;
F.rho_isa = rho_i;
Cn_dr_full = abs(P.aero.Cn_dr) * deg2rad(P.act.rudder.max_deg);
y_motor = P.prop.y_motor;
% Static thrust from the momentum relation T = (P_shaft*FM*sqrt(2*rho*A))^(2/3),
% which goes as rho^(1/3). The exponent is measured here, not assumed: the 2/3
% that belongs to shaft power is easy to mistake for the density exponent.
F.T_static_sl = (P.prop.P_shaft_peak * P.prop.power.figure_of_merit ...
   * sqrt(2 * rho_c * P.prop.disk_area))^(2/3);
F.T_static_isa = (P.prop.P_shaft_peak * P.prop.power.figure_of_merit ...
   * sqrt(2 * rho_i * P.prop.disk_area))^(2/3);
F.T_static_ratio = F.T_static_isa / F.T_static_sl;
F.rho_ratio = rho_i / rho_c;
F.exponent_measured = log(F.T_static_ratio) / log(F.rho_ratio);
F.exponent_static = 1/3;
F.exponent_shaft_power = 2/3;
% init_uacs_twin computes V_mc = sqrt(2*T_static*y/(rho*S*b*|Cn_dr|*dr_max)).
vmc = @(T_static, rho) sqrt(2 * T_static * y_motor / ...
   (rho * P.geom.S * P.geom.b * Cn_dr_full));
F.V_mc_sea_level = vmc(F.T_static_sl, rho_c);
F.V_mc_isa_as_built = vmc(F.T_static_sl, rho_i);   % thrust map still at rho_sl
F.V_mc_isa_physical = vmc(F.T_static_isa, rho_i);  % thrust map at the local density
F.cruise_mps = P.op.cruise.V_mps;
F.margin_sea_level_pct = 100 * (F.cruise_mps - F.V_mc_sea_level) / F.cruise_mps;
F.margin_isa_as_built_pct = 100 * (F.cruise_mps - F.V_mc_isa_as_built) / F.cruise_mps;
F.margin_isa_physical_pct = 100 * (F.cruise_mps - F.V_mc_isa_physical) / F.cruise_mps;
fprintf(fid, '  static thrust per motor rho^(1/3): %.6f N at sea level, %.6f N at 150 m, exponent %.6f\n', ...
   F.T_static_sl, F.T_static_isa, F.exponent_measured);
fprintf(fid, '  V_mc sea level (the declared number)              %.6f m/s, cruise margin %.3f percent\n', ...
   F.V_mc_sea_level, F.margin_sea_level_pct);
fprintf(fid, '  V_mc at 150 m, thrust map still at sea level      %.6f m/s, cruise margin %.3f percent\n', ...
   F.V_mc_isa_as_built, F.margin_isa_as_built_pct);
fprintf(fid, '  V_mc at 150 m, thrust map at the local density    %.6f m/s, cruise margin %.3f percent\n', ...
   F.V_mc_isa_physical, F.margin_isa_physical_pct);
fprintf(fid, '  physical correction against the declared number   %+.4f m/s (%+.3f percent)\n', ...
   F.V_mc_isa_physical - F.V_mc_sea_level, ...
   100 * (F.V_mc_isa_physical / F.V_mc_sea_level - 1));
% Envelope: stall, trimmable minimum, thrust limited ceiling, at the ISA density.
F.V_stall_sea_level = P.limits.V_stall_mps;
F.V_stall_isa = sqrt(2 * P.mass.m * P.env.g / (rho_i * P.geom.S * P.aero.CL_max));
F.V_min_stall_isa = P.limits.stall_margin_factor * F.V_stall_isa;
F.V_min_trim_isa = local_vmin_trim(P_isa, 0.5 * F.V_min_stall_isa);
F.V_min_trim_sea_level = P.limits.V_min_trim_mps;
F.V_min_trim_check = local_vmin_trim(P, 0.5 * P.limits.V_min_stall_mps);
F.de_trim_budget_deg = P.geom.elevator_trim_frac * P.act.elevator.max_deg;
P_thrust_isa = P;
P_thrust_isa.env.rho_sl = rho_i;
[F.V_top_isa_as_built, F.V_max_reachable_as_built] = local_top_speed(P, rho_i, P.prop.n_motors);
[F.V_top_isa_physical, F.V_max_reachable_physical] = local_top_speed(P_thrust_isa, rho_i, P.prop.n_motors);
F.V_top_sea_level = P.limits.V_top_thrust_limited_mps;
F.V_max_declared = P.limits.V_max_mps;
fprintf(fid, '  V_stall      sea level %.6f m/s, ISA 150 m %.6f m/s\n', ...
   F.V_stall_sea_level, F.V_stall_isa);
fprintf(fid, '  V_min_trim   sea level %.6f m/s, ISA 150 m %.6f m/s (elevator budget %.1f deg)\n', ...
   F.V_min_trim_sea_level, F.V_min_trim_isa, F.de_trim_budget_deg);
fprintf(fid, '  same bisection rerun here gives sea level %.6f m/s, against the stored %.6f, difference %.3e\n', ...
   F.V_min_trim_check, P.limits.V_min_trim_mps, abs(F.V_min_trim_check - P.limits.V_min_trim_mps));
fprintf(fid, '  V_top        sea level %.6f m/s, ISA 150 m as built %.6f m/s, physical %.6f m/s\n', ...
   F.V_top_sea_level, F.V_top_isa_as_built, F.V_top_isa_physical);
fprintf(fid, '  V_max declared %.1f m/s stays reachable at 150 m: as built %d, physical %d\n', ...
   F.V_max_declared, F.V_max_reachable_as_built, F.V_max_reachable_physical);
F.pass = F.exponent_measured > 0.32 && F.exponent_measured < 0.35 ...
   && F.V_mc_isa_physical > F.V_mc_sea_level ...
   && F.V_min_trim_isa > F.V_min_trim_sea_level ...
   && F.V_max_reachable_physical;
fprintf(fid, '  -> %s\n', pass_text(F.pass));

%% ---------------- G. wind trim ----------------
fprintf('[atmo_wind] G: wind trim\n');
fprintf(fid, '\n[G  plant/wind_trim.m: crab trim under a steady wind]\n');
G = struct();
G.V_mps = tr.V_mps;
G.gamma_deg = tr.gamma_deg;
I_mat = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; ...
         0, P.mass.Iyy_cg, 0; ...
         -P.mass.Ixz_cg, 0, P.mass.Izz_cg];
cases = {'zero', tr, [0; 0; 0], 0.0; ...
         'east5', tr, [0; 5; 0], 0.0; ...
         'east_track_north3west4', tr, [3; -4; 0], 90.0; ...
         'south_track_west6', tr, [0; -6; 0], 270.0; ...
         'climb6_east5', trim_cruise(P, P.op.climb), [0; 5; 0], 0.0};
G.n_cases = size(cases, 1);
G.max_res_f = 0;
G.max_res_m = 0;
G.max_psi_analytic = 0;
G.max_cross_track = 0;
G.max_air_data = 0;
G.zero_wind_bit_identical = false;
G.case = struct('name', {}, 'wind_ned', {}, 'track_deg', {}, 'psi_deg', {}, ...
   'crab_deg', {}, 'ground_speed_mps', {}, 'res_f', {}, 'res_m', {}, ...
   'cross_track_mps', {}, 'air_data_max', {}, 'psi_analytic_rad', {});
fprintf(fid, '  %-24s %-12s %6s %10s %10s %10s %10s %10s\n', ...
   'case', 'wind NED', 'track', 'psi deg', 'crab deg', 'Vg m/s', 'res F', 'res M');
for k = 1:G.n_cases
   nm = cases{k, 1};
   tr_k = cases{k, 2};
   W = cases{k, 3};
   trk = cases{k, 4};
   w = wind_trim(tr_k, W, trk);
   dx = sim_plant_rhs(0, w.x_star, w.u_star, W, P);
   % The plant's own right hand side must vanish on the nine dynamic states.
   res_f = max(abs(dx(1:3))) / P.env.g;
   R_w = sim_dcm(w.x_star(7:9));
   x_air = w.x_star;
   x_air(1:3) = w.x_star(1:3) - R_w * W;
   [V_w, al_w, be_w, q_w] = flow_state(P, x_air);
   res_m = max(abs(I_mat * dx(4:6))) / (q_w * P.geom.S * P.geom.cbar);
   air_data = max([abs(V_w - tr_k.V_mps), abs(al_w - tr_k.alpha_rad), abs(be_w)]);
   % Ground velocity from the position kinematics must lie along the track.
   v_g_ned = dx(10:12);
   t_hat = [cosd(trk); sind(trk)];
   cross_track = norm(v_g_ned(1:2) - (v_g_ned(1:2)' * t_hat) * t_hat);
   % Independent heading formula, valid on a north track only.
   psi_an = -asin(W(2) / (tr_k.V_mps * cosd(tr_k.gamma_deg)));
   G.max_res_f = max(G.max_res_f, res_f);
   G.max_res_m = max(G.max_res_m, res_m);
   G.max_cross_track = max(G.max_cross_track, cross_track);
   G.max_air_data = max(G.max_air_data, air_data);
   if trk == 0
      G.max_psi_analytic = max(G.max_psi_analytic, abs(w.psi_rad - psi_an));
   end
   if k == 1
      G.zero_wind_bit_identical = isequal(w.x_star, tr.x_star) && isequal(w.u_star, tr.u_star);
   end
   G.case(k) = struct('name', nm, 'wind_ned', W, 'track_deg', trk, ...
      'psi_deg', w.psi_deg, 'crab_deg', w.crab_deg, ...
      'ground_speed_mps', w.ground_speed_mps, 'res_f', res_f, 'res_m', res_m, ...
      'cross_track_mps', cross_track, 'air_data_max', air_data, ...
      'psi_analytic_rad', psi_an);
   fprintf(fid, '  %-24s %-12s %6.0f %10.5f %10.5f %10.5f %10.2e %10.2e\n', ...
      nm, mat2str(W'), trk, w.psi_deg, w.crab_deg, w.ground_speed_mps, res_f, res_m);
end
fprintf(fid, '  heading against the independent north track formula -asin(W_E/(V*cos gamma)): %.3e rad\n', ...
   G.max_psi_analytic);
fprintf(fid, '  cross track ground velocity %.3e m/s, air data against the still air trim %.3e\n', ...
   G.max_cross_track, G.max_air_data);
fprintf(fid, '  zero wind returns the input state bit for bit %d\n', G.zero_wind_bit_identical);
% The V9 use case: start on the crab trim and fly it. The dynamic states must not
% move and the ground track must be the commanded one.
W_g = [0; 5.0; 0];
w_g = wind_trim(tr, W_g, 0);
t_g = (0:P.sim.Ts_ctrl:5.0)';
U_g = repmat(w_g.u_star', numel(t_g), 1);
X_g = local_ode4(@(t, x, u) sim_plant_rhs(t, x, u, W_g, P), t_g, w_g.x_star, U_g);
G.traj_state_drift = max(max(abs(X_g(:, 1:9) - repmat(w_g.x_star(1:9)', numel(t_g), 1))));
G.traj_cross_track_m = max(abs(X_g(:, 11) - w_g.x_star(11)));
G.traj_north_m = X_g(end, 10) - w_g.x_star(10);
G.traj_north_expected = w_g.ground_speed_mps * t_g(end);
G.traj_drift_ok = G.traj_state_drift < 1e-9 && G.traj_cross_track_m < 1e-6 ...
   && abs(G.traj_north_m - G.traj_north_expected) < 1e-6;
fprintf(fid, '  5 s on the crab trim in a 5 m/s east wind: dynamic states drift %.3e,\n', G.traj_state_drift);
fprintf(fid, '    east position stays put to %.3e m, north distance %.6f m against %.6f m predicted %d\n', ...
   G.traj_cross_track_m, G.traj_north_m, G.traj_north_expected, G.traj_drift_ok);
G.pass = G.zero_wind_bit_identical && G.max_res_f < 1e-12 && G.max_res_m < 1e-12 ...
   && G.max_psi_analytic < 1e-12 && G.max_cross_track < 1e-9 && G.max_air_data < 1e-12 ...
   && G.traj_drift_ok;
fprintf(fid, '  -> %s\n', pass_text(G.pass));

%% ---------------- write ----------------
fprintf('[atmo_wind] A %s, B %s, C1 %s, C2 %s, C3 %s, D %s, F %s, G %s\n', ...
   pass_text(A.pass), pass_text(B.pass), pass_text(C1.pass), pass_text(C2.pass), ...
   pass_text(C3.pass), pass_text(D.pass), pass_text(F.pass), pass_text(G.pass));
verdict = A.pass && B.pass && C1.pass && C2.pass && C3.pass && D.pass && F.pass && G.pass;
fprintf(fid, '\n[verdict]\n');
fprintf(fid, '  A isa against the block        %s (max rel %.3e over h >= 0)\n', ...
   pass_text(A.pass), A.max_rel_ge0);
fprintf(fid, '  B density path                 %s\n', pass_text(B.pass));
fprintf(fid, '  C1 wind identity               %s (%.3e against %.3e)\n', ...
   pass_text(C1.pass), C1.res_after, C1.res_before);
fprintf(fid, '  C2 wind trajectory             %s (%.3e against %.3e)\n', ...
   pass_text(C2.pass), C2.max_abs_fixed_vs_oracle, C2.max_abs_old_vs_oracle);
fprintf(fid, '  C3 wind translation            %s (%.3e)\n', pass_text(C3.pass), C3.max_abs_pred);
fprintf(fid, '  D isa trim                     %s\n', pass_text(D.pass));
fprintf(fid, '  F envelope at 150 m ISA        %s (V_mc %.4f, V_min_trim %.4f, V_top %.4f)\n', ...
   pass_text(F.pass), F.V_mc_isa_physical, F.V_min_trim_isa, F.V_top_isa_physical);
fprintf(fid, '  G wind trim                    %s (res F %.3e, res M %.3e, zero wind exact %d)\n', ...
   pass_text(G.pass), G.max_res_f, G.max_res_m, G.zero_wind_bit_identical);
fprintf(fid, '  params rev %d, overall %s\n', P.meta.revision, pass_text(verdict));
fclose(fid);

save(fullfile(res_dir, 'atmo_wind_check.mat'), 'A', 'B', 'C1', 'C2', 'C3', 'D', 'E', 'F', 'G', 'P', 'stamp');
fprintf('[atmo_wind] wrote %s\n', fullfile(res_dir, 'atmo_wind_report.txt'));
fprintf('[atmo_wind] overall %s\n', pass_text(verdict));
if ~verdict
   error('atmo_wind:Failed', 'at least one stage failed, see results/atmo_wind_report.txt');
end

%% ================= local functions =================
function V_min = local_vmin_trim(P, V_lo)
% Lowest speed trimmable inside the elevator travel budget, by the same bisection
% init_uacs_twin runs: trim demand falls monotonically with speed across this band,
% so bisection is safe. Solved with the real trim solver, not a closed form.
V_hi = P.op.cruise.V_mps;
for k = 1:40
   V_mid = 0.5 * (V_lo + V_hi);
   if local_trim_de_frac(P, V_mid) > P.geom.elevator_trim_frac
      V_lo = V_mid;
   else
      V_hi = V_mid;
   end
end
V_min = 0.5 * (V_lo + V_hi);
end

function frac = local_trim_de_frac(P, V)
% Elevator travel fraction needed to trim at speed V in level flight. A speed the
% solver cannot trim counts as untrimmable rather than as trimable.
op = struct('V_mps', V, 'gamma_deg', 0.0, 'h_m', P.op.cruise.h_m);
try
   tr = trim_cruise(P, op);
catch
   frac = Inf;
   return;
end
if tr.exitflag <= 0 || tr.residual_norm_max > 1e-6
   frac = Inf;
   return;
end
frac = abs(tr.de_deg) / P.act.elevator.max_deg;
end

function [V_top, reachable] = local_top_speed(P, rho, n_motors)
% Thrust limited level flight ceiling at density rho: the scan init_uacs_twin
% runs, with the drag built at the same density. The thrust comes from the
% authoritative map, whose own density is P.env.rho_sl, so passing a P with that
% field changed is what asks for the available thrust at altitude.
V_scan = (P.limits.V_min_mps:0.5:P.limits.V_max_mps + 15)';
CL = P.mass.m * P.env.g ./ (0.5 * rho * V_scan.^2 * P.geom.S);
CD = P.aero.CD0 + P.aero.k_induced .* CL.^2;
D = 0.5 * rho * V_scan.^2 * P.geom.S .* CD;
T = n_motors * prop_thrust_from_throttle(P, 1.0, V_scan);
i_ceil = find(T(:) < D(:), 1, 'first');
if isempty(i_ceil)
   V_top = NaN;
   reachable = false;
   return;
end
V_top = V_scan(i_ceil);
reachable = P.limits.V_max_mps <= V_top;
end

function U = local_doublet(u_trim, t)
% The V12 command sequence: pitch, throttle, roll and yaw in turn, small enough
% that the aircraft stays inside the Euler angle region.
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
end

function X = local_ode4(rhs, t, x0, U)
% Fixed step RK4 over a command series sampled at the step, the same scheme the
% models run under ode4.
n = numel(t);
X = zeros(n, numel(x0));
X(1, :) = x0(:)';
h = t(2) - t(1);
for k = 1:n - 1
   xk = X(k, :)';
   uk = U(k, :)';
   k1 = rhs(t(k), xk, uk);
   k2 = rhs(t(k) + h / 2, xk + h / 2 * k1, uk);
   k3 = rhs(t(k) + h / 2, xk + h / 2 * k2, uk);
   k4 = rhs(t(k) + h, xk + h * k3, uk);
   X(k + 1, :) = (xk + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))';
end
end

function dx = local_rhs_oracle(~, x, u, W, P)
% Newton's law written directly in the ground velocity: the total body force comes
% from the plant at the air-relative state, and the momentum term uses v_ground.
% Nothing here knows about the correction term in sim_plant_rhs, so agreement with
% it is evidence rather than a restatement.
R = sim_dcm(x(7:9));
x_air = x;
x_air(1:3) = x(1:3) - R * W;
[dx_air, F_body] = eom_6dof(0, x_air, u, P);
dx = dx_air;
dx(1:3) = F_body / P.mass.m - cross(x(4:6), x(1:3));
dx(10:12) = R' * x(1:3);
end

function dx = local_rhs_old(~, x, u, W, P)
% The rev 9 right-hand side: the plant is handed the air-relative velocity and its
% derivative is used as if it were the ground velocity derivative.
R = sim_dcm(x(7:9));
x_air = x;
x_air(1:3) = x(1:3) - R * W;
dx = eom_6dof(0, x_air, u, P);
dx(10:12) = R' * x(1:3);
end

function rel = local_rel(Xa, Xb, P)
% Per state relative difference with the V12 denominator rule: the excursion of
% the run, floored by the design point scale for a state that barely moves.
d = max(abs(Xb - Xa), [], 1)';
ptp = (max(Xa, [], 1) - min(Xa, [], 1))';
scale = sim_state_scale(P);
denom = ptp;
small = denom < 1e-4 * scale;
denom(small) = scale(small);
rel = d ./ denom;
end

function v = local_v12_number(res_dir, cfg_name)
% Read a measured configuration difference back out of the V12 crosscheck file,
% so this report can be compared against the run it is meant to explain.
v = NaN;
f = fullfile(res_dir, 'sim_v12_crosscheck.mat');
if exist(f, 'file') ~= 2
   return;
end
S = load(f);
if ~isfield(S, 'M') || ~isfield(S.M, 'cfgs')
   return;
end
for k = 1:numel(S.M.cfgs)
   if strcmp(S.M.cfgs(k).name, cfg_name)
      v = S.M.cfgs(k).max_rel;
   end
end
end
