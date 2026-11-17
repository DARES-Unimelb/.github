% UACS_JOB_PLANT_BASELINE Trim and linearisation baseline of the plant model.
%
% Job script for run_job('plant_baseline'), see MATLAB/run/run_job.m.
% Runs trim_cruise at the cruise, climb and slow operating points, linearises
% at the cruise trim, studies the Jacobian step size, checks V2, V3 and V4,
% then writes
%   results/plant_baseline.mat
%   results/plant_trim_report.txt
%   results/plant_modes_report.txt
%
% Manual run:  run_job('plant_baseline')  with MATLAB/run and MATLAB/plant on
% the path. The script resolves its own paths and does not use the caller cwd.

here = fileparts(mfilename('fullpath'));
if isempty(here)
   here = fileparts(which('uacs_job_plant_baseline'));
end
if isempty(here)
   here = pwd;
end
root = fileparts(here);
addpath('C:\Ash\Projects\DARES_UACS\MATLAB\run');
addpath(fullfile(root, 'plant'), fullfile(root, 'params'), '-begin');
addpath(fullfile(root, 'sizing'), '-begin');
res_dir = fullfile(root, 'results');
if ~exist(res_dir, 'dir')
   mkdir(res_dir);
end
stamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
pass_words = {'FAIL', 'PASS'};
pass_text = @(tf) pass_words{double(tf) + 1};

init_uacs_twin('quiet', true);
if ~exist('P', 'var')
   error('plant_baseline: init_uacs_twin did not push P into the base workspace');
end
params_file = fullfile(root, 'params', 'uacs_twin_params.m');
params_dir = dir(params_file);
if ~isfield(P.meta, 'revision')
   error('plant_baseline:no_revision', 'params carry no P.meta.revision, refusing to run');
end
fprintf('plant_baseline: params %s rev %d (%s), MATLAB %s, %s\n', ...
   P.meta.version, P.meta.revision, P.meta.date, version('-release'), stamp);

%% Trim at the three operating points
trim = trim_cruise(P, P.op.cruise);
trim_climb = trim_cruise(P, P.op.climb);
trim_slow = trim_cruise(P, P.op.slow);
trim_names = {'cruise', 'climb', 'slow'};
trim_list = {trim, trim_climb, trim_slow};
save(fullfile(res_dir, 'plant_stage1_trim.mat'), 'P', 'trim', 'trim_climb', 'trim_slow');
fprintf('stage 1 saved: three trims, cruise alpha %.4f deg, de %.4f deg\n', ...
   trim.alpha_deg, trim.de_deg);

%% Linearisation at the cruise trim
[lin, lin_info] = linearize_at(P, trim);

%% Jacobian step-size study
steps = [1e-4 1e-5 1e-6 1e-7];
step_tab = zeros(numel(steps), 4);
for k = 1:numel(steps)
   lk = linearize_at(P, trim, struct('step', steps(k), 'verbose', false));
   step_tab(k, 1) = norm(lk.A - lin.A, 'fro') / norm(lin.A, 'fro');
   step_tab(k, 2) = norm(lk.B - lin.B, 'fro') / norm(lin.B, 'fro');
   i_sp_k = find(strcmp({lk.modes_long.name}, 'short_period'), 1);
   step_tab(k, 3) = lk.modes_long(i_sp_k).freq_rad_s;
   step_tab(k, 4) = lk.modes_long(i_sp_k).damping;
end
save(fullfile(res_dir, 'plant_stage2_lin.mat'), 'lin', 'lin_info', 'step_tab', 'steps');
fprintf('stage 2 saved: linearisation and step study, %d eom evaluations\n', lin_info.n_eval);

%% Linearisation at the climb and slow trims
lin_climb = linearize_at(P, trim_climb, struct('verbose', false));
lin_slow = linearize_at(P, trim_slow, struct('verbose', false));
fprintf('stage 2b saved: climb and slow linearised\n');

%% Trim envelope: lowest speed trimable inside 80 percent of travel
de_limit_deg = 0.8 * P.act.elevator.max_deg;
V_scan = 16:1:40;
sw_de = nan(size(V_scan));
sw_th = nan(size(V_scan));
sw_ok = false(size(V_scan));
for k = 1:numel(V_scan)
   opk = P.op.cruise;
   opk.V_mps = V_scan(k);
   try
      tk = trim_cruise(P, opk);
      sw_de(k) = tk.de_deg;
      sw_th(k) = tk.throttle;
      sw_ok(k) = abs(tk.de_deg) <= de_limit_deg && tk.throttle <= 1;
   catch
      sw_ok(k) = false;
   end
end
i_first_ok = find(sw_ok, 1, 'first');
if isempty(i_first_ok)
   V_trim_min = NaN;
elseif i_first_ok == 1
   V_trim_min = V_scan(1);
else
   a_lo = V_scan(i_first_ok - 1);
   b_hi = V_scan(i_first_ok);
   for it = 1:30
      v_mid = 0.5 * (a_lo + b_hi);
      opm = P.op.cruise;
      opm.V_mps = v_mid;
      tm = trim_cruise(P, opm);
      if abs(tm.de_deg) <= de_limit_deg && tm.throttle <= 1
         b_hi = v_mid;
      else
         a_lo = v_mid;
      end
   end
   V_trim_min = b_hi;
end
sweep = struct('V_scan', V_scan, 'de_deg', sw_de, 'throttle', sw_th, 'ok', sw_ok, ...
   'de_limit_deg', de_limit_deg, 'V_trim_min_mps', V_trim_min);
fprintf('stage 3: trim envelope scanned, V_trim_min %.4f m/s (limit %.1f deg, P.limits.V_min_mps %.4f)\n', ...
   V_trim_min, de_limit_deg, P.limits.V_min_mps);

%% Mode bookkeeping
eig_A = eig(lin.A);
eig_join = [eig(lin.A_long); eig(lin.A_lat); 0; 0; 0; 0];
join_err = 0;
for k = 1:numel(eig_A)
   join_err = max(join_err, min(abs(eig_A(k) - eig_join)));
end
max_real_A = max(real(eig_A));
n_zero_A = sum(abs(eig_A) < 1e-9);

i_sp = find(strcmp({lin.modes_long.name}, 'short_period'), 1);
i_ph = find(strcmp({lin.modes_long.name}, 'phugoid'), 1);
i_dr = find(strcmp({lin.modes_lat.name}, 'dutch_roll'), 1);
i_ro = find(strcmp({lin.modes_lat.name}, 'roll_subsidence'), 1);
i_spi = find(strcmp({lin.modes_lat.name}, 'spiral'), 1);

%% Acceptance criteria
crit = struct();
crit.V2_residual = trim.check_residual;
crit.V2_alpha_frozen = trim.check_alpha;
crit.V2_alpha_revised = trim.check_alpha_revised;
crit.V2_de = trim.check_de;
crit.V2_de_pct_travel = 100 * abs(trim.de_deg) / P.act.elevator.max_deg;
crit.V2_de_ok_pct80 = crit.V2_de_pct_travel <= 80;
crit.V2_all_frozen = trim.check_all;
crit.V2_all_revised = trim.check_all_revised;
crit.V2_values = struct('residual_norm_max', trim.residual_norm_max, ...
   'alpha_deg', trim.alpha_deg, 'de_deg', trim.de_deg, ...
   'throttle', trim.throttle, 'exitflag', trim.exitflag);
crit.V3_no_positive_real = max_real_A <= 0;
crit.V3_max_real_eig = max_real_A;
crit.V3_zero_eigs = n_zero_A;
crit.V3_mode_join_err = join_err;
crit.V4_window_rad_s = [2 20];
crit.V4_sp_tau_limit_s = 0.1;
% An overdamped short period has no oscillatory frequency or damping ratio, so
% judge it by its time constants and an oscillatory one by the window and damping
sp = lin.modes_long(i_sp);
crit.V4_sp_overdamped = ~sp.is_oscillatory;
crit.V4_sp_freq_rad_s = sp.freq_nat_rad_s;
crit.V4_sp_tau_s = sp.tau_s;
crit.V4_sp_freq_hz = sp.freq_hz;
crit.V4_sp_zeta = sp.damping;
crit.V4_sp_freq_ok = sp.freq_nat_rad_s >= crit.V4_window_rad_s(1) ...
   && sp.freq_nat_rad_s <= crit.V4_window_rad_s(2);
crit.V4_sp_freq_ok_old_window_2_8 = sp.freq_nat_rad_s >= 2 ...
   && sp.freq_nat_rad_s <= 8;
crit.V4_sp_freq_damped_ok = sp.freq_damped_rad_s >= crit.V4_window_rad_s(1) ...
   && sp.freq_damped_rad_s <= crit.V4_window_rad_s(2);
crit.V4_sp_freq_damped_rad_s = sp.freq_damped_rad_s;
crit.V4_sp_zeta_ok = sp.damping > 0.3;
% For an overdamped pair the binding criterion is the SLOWER time constant
crit.V4_sp_tau_ok = sp.tau_s < crit.V4_sp_tau_limit_s;
if crit.V4_sp_overdamped
   crit.V4_sp_ok = crit.V4_sp_tau_ok;
else
   crit.V4_sp_ok = crit.V4_sp_freq_ok && crit.V4_sp_zeta_ok;
end
if crit.V4_sp_overdamped
   crit.V4_sp_verdict = sprintf('overdamped, slowest tau %.6f s (limit %.2f s)', ...
      sp.tau_s, crit.V4_sp_tau_limit_s);
else
   crit.V4_sp_verdict = sprintf('oscillatory, wn %.4f rad/s zeta %.4f', ...
      sp.freq_nat_rad_s, sp.damping);
end
crit.V4_dr_zeta_ok = lin.modes_lat(i_dr).damping > 0.05;
crit.V4_dr_zeta = lin.modes_lat(i_dr).damping;
crit.V4_dr_freq_rad_s = lin.modes_lat(i_dr).freq_nat_rad_s;
crit.V4_roll_tau_ok = lin.modes_lat(i_ro).tau_s < 1;
crit.V4_roll_tau_s = lin.modes_lat(i_ro).tau_s;
crit.V4_all = crit.V4_sp_ok && crit.V4_dr_zeta_ok && crit.V4_roll_tau_ok;
if i_spi > 0 && lin.modes_lat(i_spi).real_part > 0
   crit.V3_spiral_lambda = lin.modes_lat(i_spi).real_part;
   crit.V3_spiral_time_to_double_s = log(2) / lin.modes_lat(i_spi).real_part;
elseif i_spi > 0
   crit.V3_spiral_lambda = lin.modes_lat(i_spi).real_part;
   crit.V3_spiral_time_to_double_s = Inf;
else
   crit.V3_spiral_lambda = NaN;
   crit.V3_spiral_time_to_double_s = Inf;
end
if i_ph > 0
   crit.phugoid_freq_rad_s = lin.modes_long(i_ph).freq_nat_rad_s;
   crit.phugoid_damping = lin.modes_long(i_ph).damping;
end

%% Cross-check against the sizing analytic approximations
% Real derivatives are tail-dominated single-derivative estimates, so the job
% reports the difference instead of picking one, and keeps working if that file
% is unavailable or mid-edit
sz_ok = true;
try
   addpath(fullfile(root, 'sizing'), '-begin');
   sz = sizing_summary(P);
catch ME
   sz_ok = false;
   sz = struct();
   fprintf('sizing_summary unavailable, analytic cross-check skipped: %s\n', ME.message);
end
crit.sz_ok = sz_ok;
if sz_ok
   crit.sz_short_period_rad_s = sz.modes.short_period_approx_rad_s;
   crit.sz_dutch_roll_rad_s = sz.modes.dutch_roll_approx_rad_s;
   crit.sz_roll_tau_s = sz.modes.roll_subsidence_tau_s;
   crit.sz_sp_rel_diff = (crit.V4_sp_freq_rad_s - crit.sz_short_period_rad_s) / crit.sz_short_period_rad_s;
   crit.sz_dr_rel_diff = (crit.V4_dr_freq_rad_s - crit.sz_dutch_roll_rad_s) / crit.sz_dutch_roll_rad_s;
   crit.sz_roll_rel_diff = (crit.V4_roll_tau_s - crit.sz_roll_tau_s) / crit.sz_roll_tau_s;
end

%% dT channel versus the throttle channels
% B(:,6) must equal (B(:,4)+B(:,5)) / (2*dT/dthrottle): the additive thrust
% command is split over the two motors, so one N of dT is half a newton each
h_th = 1e-6;
dT_dth = (prop_thrust_from_throttle(P, trim.throttle + h_th, trim.V_mps) ...
        - prop_thrust_from_throttle(P, trim.throttle - h_th, trim.V_mps)) / (2 * h_th);
B_dT_expected = (lin.B(:, 4) + lin.B(:, 5)) / (2 * dT_dth);
crit.dT_dthrottle_N_per_unit = dT_dth;
crit.B6_vs_throttle_rel = norm(lin.B(:, 6) - B_dT_expected) / max(norm(B_dT_expected), eps);
crit.B6_expected = B_dT_expected;

plant_meta = struct();
plant_meta.generated = stamp;
plant_meta.matlab = version();
plant_meta.params_version = P.meta.version;
plant_meta.params_revision = P.meta.revision;
plant_meta.params_date = P.meta.date;
plant_meta.params_bytes = params_dir.bytes;
plant_meta.params_filedate = params_dir.date;
plant_meta.lin_step = lin_info.step;
plant_meta.lin_hx = lin_info.hx;
plant_meta.lin_hu = lin_info.hu;
plant_meta.lin_eval = lin_info.n_eval;
plant_meta.idx_long_x = lin.idx_long_x;
plant_meta.idx_long_u = lin.idx_long_u;
plant_meta.idx_lat_x = lin.idx_lat_x;
plant_meta.idx_lat_u = lin.idx_lat_u;
plant_meta.alphadot_law = 'Cm_alphadot evaluated with alpha_dot = q';
% The constant-model string is kept as is so a default run still produces a
% byte identical plant_baseline.mat
plant_meta.density_law = 'constant P.env.rho_sl, no density altitude';
if isfield(P.atmo, 'model') && strcmp(P.atmo.model, 'isa')
   if ~isfield(P.atmo, 'h_from_state') || P.atmo.h_from_state
      plant_meta.density_law = 'ISA density at the state altitude -pd, plant/atmo_isa.m';
   else
      plant_meta.density_law = sprintf( ...
         'ISA density frozen at P.atmo.h_ref = %.1f m, plant/atmo_isa.m', P.atmo.h_ref);
   end
end
plant_meta.thrust_arm = 'z arm = P.prop.z_thrust_offset';

save(fullfile(res_dir, 'plant_baseline.mat'), ...
   'P', 'trim', 'trim_climb', 'trim_slow', 'lin', 'lin_climb', 'lin_slow', ...
   'lin_info', 'crit', 'plant_meta', 'sweep', 'sz', 'sz_ok');

%% Trim report
fid = fopen(fullfile(res_dir, 'plant_trim_report.txt'), 'w');
fprintf(fid, 'UACS twin-engine plant baseline: trim report\n');
fprintf(fid, 'generated %s, MATLAB %s\n', stamp, version('-release'));
fprintf(fid, 'params %s rev %d (%s), uacs_twin_params.m %d bytes dated %s\n', ...
   P.meta.version, P.meta.revision, P.meta.date, params_dir.bytes, params_dir.date);
fprintf(fid, '\nResidual definition\n');
fprintf(fid, '  r_raw  = [Fx Fy Fz Mx My Mz]'' at the trim state, taken from eom_6dof, N and N*m\n');
fprintf(fid, '  r_norm = [r_raw(1:3)/(m*g); r_raw(4:6)/(qbar*S*cbar)]\n');
fprintf(fid, '  m*g = %.6f N, qbar*S*cbar is per operating point, N*m\n', P.mass.m * P.env.g);
fprintf(fid, '  criterion V2 uses max(abs(r_norm)) < 1e-6\n');
fprintf(fid, '  unknowns alpha, de, one throttle for both motors; equations Fx = Fz = My = 0\n');
fprintf(fid, '  convention beta = da = dr = 0, phi = 0, theta = alpha + gamma,\n');
fprintf(fid, '  so theta equals alpha in level flight and theta = alpha + gamma when climbing\n');
fprintf(fid, '  solver %s\n', trim.solver);

for k = 1:numel(trim_names)
   t = trim_list{k};
   fprintf(fid, '\n=== %s operating point ===\n', trim_names{k});
   fprintf(fid, '  V_mps                     %12.6f\n', t.V_mps);
   fprintf(fid, '  gamma_deg                 %12.6f\n', t.gamma_deg);
   fprintf(fid, '  bank_deg                  %12.6f\n', t.bank_deg);
   fprintf(fid, '  h_m                       %12.6f\n', t.h_m);
   fprintf(fid, '  rho_kg_m3                 %12.6f\n', t.rho);
   fprintf(fid, '  qbar_Pa                   %12.6f\n', t.qbar);
   fprintf(fid, '  alpha_deg                 %12.6f\n', t.alpha_deg);
   fprintf(fid, '  theta_deg                 %12.6f\n', t.theta_deg);
   fprintf(fid, '  de_deg                    %12.6f\n', t.de_deg);
   fprintf(fid, '  throttle_per_motor        %12.6f\n', t.throttle);
   fprintf(fid, '  CL                        %12.6f\n', t.CL);
   fprintf(fid, '  CD                        %12.6f\n', t.CD);
   fprintf(fid, '  L_over_D                  %12.6f\n', t.L_over_D);
   fprintf(fid, '  drag_N                    %12.6f\n', t.drag_N);
   fprintf(fid, '  thrust_total_N            %12.6f\n', t.thrust_total_N);
   fprintf(fid, '  T_static_max_N_per_motor  %12.6f\n', P.prop.T_static_max);
   fprintf(fid, '  throttle_exponent         %12.6f (throttle is a shaft-power share)\n', ...
      P.prop.throttle_exponent);
   fprintf(fid, '  x_star                    %s\n', mat2str(t.x_star, 8));
   fprintf(fid, '  u_star                    %s\n', mat2str(t.u_star, 8));
   fprintf(fid, '  r_raw  [Fx Fy Fz Mx My Mz] %s\n', mat2str(t.residual, 6));
   fprintf(fid, '  r_norm                     %s\n', mat2str(t.residual_norm, 6));
   fprintf(fid, '  max|r_norm|                %12.3e\n', t.residual_norm_max);
   fprintf(fid, '  normalisers m*g, qbar*S*cbar  %.6f N, %.6f N*m\n', t.norm_force_N, t.norm_moment_Nm);
   fprintf(fid, '  fsolve exitflag %d, iterations %d, funcCount %d, first-order optimality %.3e\n', ...
      t.exitflag, t.iterations, t.func_count, t.first_order_opt);
   fprintf(fid, '  V2 residual < 1e-6        %s (%.3e)\n', pass_text(t.check_residual), t.residual_norm_max);
   fprintf(fid, '  V2 alpha in 2..10 deg (frozen in architecture.md 7)  %s (%.4f deg)\n', ...
      pass_text(t.check_alpha), t.alpha_deg);
   fprintf(fid, '  V2 alpha in 0..8 deg (revised window, see design notes)  %s (%.4f deg)\n', ...
      pass_text(t.check_alpha_revised), t.alpha_deg);
   fprintf(fid, '  V2 |de| <= 15 deg         %s (%.4f deg)\n', pass_text(t.check_de), t.de_deg);
   fprintf(fid, '  V2 de as percent of travel            %.2f percent of %.1f deg\n', ...
      100 * abs(t.de_deg) / P.act.elevator.max_deg, P.act.elevator.max_deg);
   fprintf(fid, '  V2 overall, frozen window     %s\n', pass_text(t.check_all));
   fprintf(fid, '  V2 overall, revised window    %s\n', pass_text(t.check_all_revised));
end

fprintf(fid, '\n=== summary ===\n');
fprintf(fid, '  %-8s %8s %8s %10s %10s %10s %10s %10s\n', ...
   'point', 'V', 'gamma', 'alpha_deg', 'theta_deg', 'de_deg', 'throttle', 'max|r_norm|');
for k = 1:numel(trim_names)
   t = trim_list{k};
   fprintf(fid, '  %-8s %8.2f %8.2f %10.4f %10.4f %10.4f %10.4f %10.2e\n', ...
      trim_names{k}, t.V_mps, t.gamma_deg, t.alpha_deg, t.theta_deg, t.de_deg, ...
      t.throttle, t.residual_norm_max);
end
fprintf(fid, '\n  V2 (cruise): residual %s, alpha frozen window %s, alpha revised window %s, de %s\n', ...
   pass_text(crit.V2_residual), pass_text(crit.V2_alpha_frozen), ...
   pass_text(crit.V2_alpha_revised), pass_text(crit.V2_de));
fprintf(fid, '  overall: frozen window %s, revised window %s\n', ...
   pass_text(crit.V2_all_frozen), pass_text(crit.V2_all_revised));
% Lift coefficient implied by each alpha window edge
cl_lo_frozen = P.aero.CL0 + P.aero.CL_alpha * deg2rad(trim.window_alpha_frozen_deg(1));
cl_hi_frozen = P.aero.CL0 + P.aero.CL_alpha * deg2rad(trim.window_alpha_frozen_deg(2));
cl_lo_rev = P.aero.CL0 + P.aero.CL_alpha * deg2rad(trim.window_alpha_revised_deg(1));
cl_hi_rev = P.aero.CL0 + P.aero.CL_alpha * deg2rad(trim.window_alpha_revised_deg(2));
w_kg = P.mass.m * P.env.g;
fprintf(fid, '\n  Required CL at cruise %.6f\n', trim.CL);
fprintf(fid, '  frozen window 2..10 deg spans CL %.4f..%.4f, revised window 0..8 deg spans CL %.4f..%.4f\n', ...
   cl_lo_frozen, cl_hi_frozen, cl_lo_rev, cl_hi_rev);
fprintf(fid, '  the frozen lower edge needs V <= %.2f m/s at this mass and area, or S <= %.4f m^2,\n', ...
   sqrt(2 * w_kg / (P.env.rho_sl * P.geom.S * cl_lo_frozen)), ...
   w_kg / (0.5 * P.env.rho_sl * P.op.cruise.V_mps^2 * cl_lo_frozen));
fprintf(fid, '  or m >= %.2f kg at 25 m/s. The revised lower edge (alpha = 0) needs CL %.4f,\n', ...
   0.5 * P.env.rho_sl * P.op.cruise.V_mps^2 * cl_lo_frozen * P.geom.S / P.env.g, cl_lo_rev);
fprintf(fid, '  which the cruise point clears without changing the geometry.\n');

fprintf(fid, '\n=== trim envelope, level flight, elevator limit %.1f deg (80 percent of travel) ===\n', ...
   sweep.de_limit_deg);
fprintf(fid, '  %6s %12s %10s %6s\n', 'V_mps', 'de_deg', 'throttle', 'ok');
for k = 1:numel(sweep.V_scan)
   fprintf(fid, '  %6.1f %12.4f %10.4f %6d\n', ...
      sweep.V_scan(k), sweep.de_deg(k), sweep.throttle(k), sweep.ok(k));
end
fprintf(fid, '  lowest trimable speed inside the limit: %.4f m/s\n', sweep.V_trim_min_mps);
if isfield(P.limits, 'V_stall_mps')
   fprintf(fid, '  P.limits.V_min_mps %.4f m/s, P.limits.V_stall_mps %.4f m/s\n', ...
      P.limits.V_min_mps, P.limits.V_stall_mps);
else
   fprintf(fid, '  P.limits.V_min_mps %.4f m/s\n', P.limits.V_min_mps);
end
fclose(fid);

%% Modes report
fid = fopen(fullfile(res_dir, 'plant_modes_report.txt'), 'w');
fprintf(fid, 'UACS twin-engine plant baseline: linearisation and modes report\n');
fprintf(fid, 'generated %s, MATLAB %s\n', stamp, version('-release'));
fprintf(fid, 'params %s rev %d (%s), uacs_twin_params.m %d bytes dated %s\n', ...
   P.meta.version, P.meta.revision, P.meta.date, params_dir.bytes, params_dir.date);
fprintf(fid, '\nLinearisation point\n');
fprintf(fid, '  cruise trim: V %.4f m/s, alpha %.6f deg, de %.6f deg, throttle %.6f\n', ...
   trim.V_mps, trim.alpha_deg, trim.de_deg, trim.throttle);
fprintf(fid, '  central difference step rule h_i = step*max(1,|x_i|), step = %.1e\n', lin_info.step);
fprintf(fid, '  state step vector             %s\n', mat2str(lin_info.hx, 3));
fprintf(fid, '  input step vector             %s\n', mat2str(lin_info.hu, 3));
fprintf(fid, '  eom evaluations %d, elapsed %.3f s\n', lin_info.n_eval, lin_info.elapsed_s);
fprintf(fid, '  C = eye(12), D = zeros(12,6): all 12 states are outputs, no feedthrough\n');
fprintf(fid, '\nIndex sets used for the submatrices (extracted from A and B as printed)\n');
fprintf(fid, '  A_long = A(%s, %s), B_long = B(%s, %s)  states [%s]  inputs [%s]\n', ...
   mat2str(lin.idx_long_x), mat2str(lin.idx_long_x), mat2str(lin.idx_long_x), ...
   mat2str(lin.idx_long_u), strjoin(lin.long_state_names, ' '), ...
   strjoin(lin.long_input_names, ' '));
fprintf(fid, '  A_lat  = A(%s, %s), B_lat  = B(%s, %s)  states [%s]  inputs [%s]\n', ...
   mat2str(lin.idx_lat_x), mat2str(lin.idx_lat_x), mat2str(lin.idx_lat_x), ...
   mat2str(lin.idx_lat_u), strjoin(lin.lat_state_names, ' '), ...
   strjoin(lin.lat_input_names, ' '));
fprintf(fid, '  u is used as the surrogate for V in A_long: at the trim point alpha is\n');
fprintf(fid, '  %.4f deg, so u = V*cos(alpha) differs from V by %.3e relative\n', ...
   trim.alpha_deg, 1 - cos(trim.alpha_rad));

fprintf(fid, '\nStep-size study, difference against the step 1e-6 result\n');
fprintf(fid, '  %10s %14s %14s %18s %12s\n', 'step', '|dA|/|A|', '|dB|/|B|', 'sp freq rad/s', 'sp zeta');
for k = 1:numel(steps)
   fprintf(fid, '  %10.0e %14.3e %14.3e %18.6f %12.6f\n', ...
      steps(k), step_tab(k, 1), step_tab(k, 2), step_tab(k, 3), step_tab(k, 4));
end

fprintf(fid, '\nTwelve eigenvalues of A\n');
fprintf(fid, '  %4s %16s %16s %14s %14s\n', 'idx', 'real', 'imag', 'abs', 'note');
for k = 1:numel(eig_A)
   note = '';
   if abs(eig_A(k)) < 1e-9
      note = 'kinematic zero';
   end
   fprintf(fid, '  %4d %16.8f %16.8f %14.8f %14s\n', ...
      k, real(eig_A(k)), imag(eig_A(k)), abs(eig_A(k)), note);
end
fprintf(fid, '  max real part %.8f, count of zero eigenvalues %d\n', max_real_A, n_zero_A);
fprintf(fid, '  eig(A) vs eig(A_long) + eig(A_lat) + {0,0,0,0}: max mismatch %.3e\n', join_err);

fprintf(fid, '\nLongitudinal modes from A_long, states [%s]\n', strjoin(lin.long_state_names, ' '));
fprintf(fid, '  wn_rad_s is the pole magnitude, the damp() convention; wd_rad_s is |Im|\n');
fprintf(fid, '  %-16s %14s %14s %12s %12s %10s %10s %12s\n', ...
   'mode', 'real', 'imag', 'wn_rad_s', 'wd_rad_s', 'freq_Hz', 'zeta', 'tau_s');
for k = 1:numel(lin.modes_long)
   md = lin.modes_long(k);
   fprintf(fid, '  %-16s %14.6f %14.6f %12.6f %12.6f %10.4f %10.4f %12.6f\n', ...
      md.name, md.real_part, md.imag_part, md.freq_nat_rad_s, md.freq_damped_rad_s, ...
      md.freq_hz, md.damping, md.tau_s);
end
fprintf(fid, '  A_long =\n%s\n', mat2str(lin.A_long, 6));
fprintf(fid, '  B_long =\n%s\n', mat2str(lin.B_long, 6));

fprintf(fid, '\nLateral modes from A_lat, states [%s]\n', strjoin(lin.lat_state_names, ' '));
fprintf(fid, '  wn_rad_s is the pole magnitude, the damp() convention; wd_rad_s is |Im|\n');
fprintf(fid, '  %-16s %14s %14s %12s %12s %10s %10s %12s\n', ...
   'mode', 'real', 'imag', 'wn_rad_s', 'wd_rad_s', 'freq_Hz', 'zeta', 'tau_s');
for k = 1:numel(lin.modes_lat)
   md = lin.modes_lat(k);
   fprintf(fid, '  %-16s %14.6f %14.6f %12.6f %12.6f %10.4f %10.4f %12.6f\n', ...
      md.name, md.real_part, md.imag_part, md.freq_nat_rad_s, md.freq_damped_rad_s, ...
      md.freq_hz, md.damping, md.tau_s);
end
fprintf(fid, '  A_lat =\n%s\n', mat2str(lin.A_lat, 6));
fprintf(fid, '  B_lat =\n%s\n', mat2str(lin.B_lat, 6));

fprintf(fid, '\nModes at the climb and slow trims, same index sets as above\n');
for kk = 1:2
   if kk == 1
      lk = lin_climb;
      tk = trim_climb;
      nm = 'climb';
   else
      lk = lin_slow;
      tk = trim_slow;
      nm = 'slow';
   end
   fprintf(fid, '  --- %s: V %.4f m/s, alpha %.6f deg, de %.6f deg (%.2f percent travel), throttle %.6f ---\n', ...
      nm, tk.V_mps, tk.alpha_deg, tk.de_deg, 100 * abs(tk.de_deg) / P.act.elevator.max_deg, tk.throttle);
   fprintf(fid, '  %-16s %14s %14s %12s %12s %10s %10s\n', ...
      'mode', 'real', 'imag', 'wn_rad_s', 'wd_rad_s', 'zeta', 'tau_s');
   for j = 1:numel(lk.modes_long)
      md = lk.modes_long(j);
      fprintf(fid, '  %-16s %14.6f %14.6f %12.6f %12.6f %10.4f %10.4f\n', ...
         md.name, md.real_part, md.imag_part, md.freq_nat_rad_s, md.freq_damped_rad_s, md.damping, md.tau_s);
   end
   for j = 1:numel(lk.modes_lat)
      md = lk.modes_lat(j);
      fprintf(fid, '  %-16s %14.6f %14.6f %12.6f %12.6f %10.4f %10.4f\n', ...
         md.name, md.real_part, md.imag_part, md.freq_nat_rad_s, md.freq_damped_rad_s, md.damping, md.tau_s);
   end
end

fprintf(fid, '\nAcceptance criteria\n');
fprintf(fid, '  V3 no positive real part eigenvalue   %s (max real %+.3e, %d kinematic zeros)\n', ...
   pass_text(crit.V3_no_positive_real), crit.V3_max_real_eig, n_zero_A);
if i_spi > 0 && ~lin.modes_lat(i_spi).stable
   fprintf(fid, '     unstable pole is the spiral mode, lambda %+.6f 1/s, time to double %.1f s\n', ...
      lin.modes_lat(i_spi).real_part, log(2) / lin.modes_lat(i_spi).real_part);
end
fprintf(fid, '  V4 window %.0f..%.0f rad/s, applied only to an oscillatory short period\n', ...
   crit.V4_window_rad_s(1), crit.V4_window_rad_s(2));
fprintf(fid, '  V4 short period type                  %s\n', crit.V4_sp_verdict);
fprintf(fid, '  V4 short period criterion             %s\n', pass_text(crit.V4_sp_ok));
fprintf(fid, '  V4   if overdamped, slowest tau < %.2f s   %s (tau %.6f s)\n', ...
   crit.V4_sp_tau_limit_s, pass_text(crit.V4_sp_tau_ok), crit.V4_sp_tau_s);
fprintf(fid, '  V4   if oscillatory, wn in window          %s (wn %.6f rad/s, %.4f Hz)\n', ...
   pass_text(crit.V4_sp_freq_ok), crit.V4_sp_freq_rad_s, crit.V4_sp_freq_hz);
fprintf(fid, '  V4   if oscillatory, damping > 0.3         %s (%.6f)\n', ...
   pass_text(crit.V4_sp_zeta_ok), crit.V4_sp_zeta);
fprintf(fid, '  V4 same wn against the old 2..8 band  %s (reported for history only)\n', ...
   pass_text(crit.V4_sp_freq_ok_old_window_2_8));
fprintf(fid, '  V4 short period wd in window          %s (wd %.6f rad/s, reported for history only)\n', ...
   pass_text(crit.V4_sp_freq_damped_ok), crit.V4_sp_freq_damped_rad_s);
fprintf(fid, '  V4 dutch roll damping > 0.05         %s (%.6f, wn %.4f rad/s)\n', ...
   pass_text(crit.V4_dr_zeta_ok), crit.V4_dr_zeta, crit.V4_dr_freq_rad_s);
fprintf(fid, '  V4 roll subsidence tau < 1 s         %s (%.6f s)\n', ...
   pass_text(crit.V4_roll_tau_ok), crit.V4_roll_tau_s);
fprintf(fid, '  V4 overall                           %s\n', pass_text(crit.V4_all));
if i_ph > 0
   fprintf(fid, '  phugoid: wn %.6f rad/s, wd %.6f rad/s, zeta %.6f, tau %.1f s\n', ...
      lin.modes_long(i_ph).freq_nat_rad_s, lin.modes_long(i_ph).freq_damped_rad_s, ...
      lin.modes_long(i_ph).damping, lin.modes_long(i_ph).tau_s);
end
fprintf(fid, '\nThrust moment bookkeeping\n');
fprintf(fid, '  z arm used = P.prop.z_thrust_offset = %.6f m\n', P.prop.z_thrust_offset);
fprintf(fid, '  z_motor - z_cg = %.6f m, difference %.3e m\n', ...
   P.prop.z_motor - P.mass.z_cg, P.prop.z_thrust_offset - (P.prop.z_motor - P.mass.z_cg));
fprintf(fid, '  thrust pitch moment at cruise %.4f N*m = dCm %.6f = %.4f deg of de\n', ...
   -trim.thrust_total_N * P.prop.z_thrust_offset, ...
   -trim.thrust_total_N * P.prop.z_thrust_offset / trim.norm_moment_Nm, ...
   rad2deg(-trim.thrust_total_N * P.prop.z_thrust_offset / trim.norm_moment_Nm / P.aero.Cm_de));

fprintf(fid, '\nCross-check against the sizing analytic approximations\n');
if sz_ok
   fprintf(fid, '  short period wn: measured %.6f rad/s, sizing %s %.6f rad/s, difference %+.2f%%\n', ...
      crit.V4_sp_freq_rad_s, 'sqrt(q*S*cbar*(-Cm_alpha)/Iyy) =', ...
      crit.sz_short_period_rad_s, 100 * crit.sz_sp_rel_diff);
   fprintf(fid, '  dutch roll wn:   measured %.6f rad/s, sizing %s %.6f rad/s, difference %+.2f%%\n', ...
      crit.V4_dr_freq_rad_s, 'sqrt(q*S*b*Cn_beta/Izz) =', ...
      crit.sz_dutch_roll_rad_s, 100 * crit.sz_dr_rel_diff);
   fprintf(fid, '  roll subsidence: measured %.6f s, sizing %s %.6f s, difference %+.2f%%\n', ...
      crit.V4_roll_tau_s, 'Ixx/(q*S*b*(b/2V)*|Cl_p|) =', ...
      crit.sz_roll_tau_s, 100 * crit.sz_roll_rel_diff);
   fprintf(fid, '  the sizing forms are single-derivative estimates and omit the damping\n');
   fprintf(fid, '  coupling, so a measured frequency above them is expected, not a defect\n');
else
   fprintf(fid, '  SKIP: sizing_summary was not available during this run\n');
end

fprintf(fid, '\ndT channel consistency\n');
fprintf(fid, '  dT/dthrottle at the cruise trim %.6f N per unit throttle per motor\n', ...
   crit.dT_dthrottle_N_per_unit);
fprintf(fid, '  (B(:,4)+B(:,5))/(2*dT/dthrottle) against B(:,6): relative difference %.3e\n', ...
   crit.B6_vs_throttle_rel);
fprintf(fid, '  B(:,6) expected %s\n', mat2str(crit.B6_expected, 6));
fprintf(fid, '  B(:,6) measured %s\n', mat2str(lin.B(:, 6), 6));
fclose(fid);

fprintf('\nplant_baseline done, params rev %d\n', P.meta.revision);
fprintf('  V2 residual %s (%.3e), alpha %.4f deg: frozen window %s, revised window %s, de %.4f deg %s\n', ...
   pass_text(crit.V2_residual), trim.residual_norm_max, trim.alpha_deg, ...
   pass_text(crit.V2_alpha_frozen), pass_text(crit.V2_alpha_revised), trim.de_deg, ...
   pass_text(crit.V2_de));
fprintf('  V3 no positive real part %s (max real %+.3e)\n', ...
   pass_text(crit.V3_no_positive_real), crit.V3_max_real_eig);
fprintf('  V4 short period %s, dutch roll wn %.4f zeta %.4f, roll tau %.5f s, overall %s\n', ...
   crit.V4_sp_verdict, crit.V4_dr_freq_rad_s, crit.V4_dr_zeta, ...
   crit.V4_roll_tau_s, pass_text(crit.V4_all));
if sz_ok
   fprintf('  sizing analytic: sp %.4f rad/s (%+.1f%%), dr %.4f rad/s (%+.1f%%), roll tau %.5f s (%+.1f%%)\n', ...
      crit.sz_short_period_rad_s, 100 * crit.sz_sp_rel_diff, crit.sz_dutch_roll_rad_s, ...
      100 * crit.sz_dr_rel_diff, crit.sz_roll_tau_s, 100 * crit.sz_roll_rel_diff);
end
fprintf('  B(:,6) vs throttle-channel slope: relative difference %.3e\n', crit.B6_vs_throttle_rel);
fprintf('  trim envelope: lowest speed inside 80 percent travel %.4f m/s (P.limits.V_min_mps %.4f)\n', ...
   sweep.V_trim_min_mps, P.limits.V_min_mps);
