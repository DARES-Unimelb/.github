% UACS_JOB_CONTROL_DESIGN Design LQR and PID, check V5..V8, write the reports.
%
% Run with:  addpath('C:\Ash\Projects\DARES_UACS\MATLAB\run'); run_job('control_design')
% Success is the marker === JOB DONE in results/job_control_design.log.

here = fileparts(which('uacs_job_control_design'));
if isempty(here)
   here = fileparts(mfilename('fullpath'));
end
root = fileparts(here);
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sizing'), fullfile(root, 'run'), fullfile(root, 'verify'));
resdir = fullfile(root, 'results');
if ~exist(resdir, 'dir')
   mkdir(resdir);
end

init_uacs_twin('quiet', true);
if isfield(P.meta, 'revision')
   p_rev = P.meta.revision;
else
   p_rev = NaN;
end
fprintf('control design job on P %s rev %g, MATLAB %s\n', P.meta.version, ...
   p_rev, version('-release'));

% ================= plant: trim, linearise, sizing budget =================
trim = trim_cruise(P, P.op.cruise);
assert(trim.exitflag > 0, 'trim failed');
[lin, lin_info] = linearize_at(P, trim, struct('step', 1e-6, 'verbose', true));
S = sizing_summary(P);

% ================= designs =================
gains_lqr = design_lqr(P, lin);
lqr_alt_theta = design_lqr(P, lin, struct('long_int_out', 'theta'));
lqr_alt_beta = design_lqr(P, lin, struct('lat_int_out', 'beta'));
gains_pid = design_pid(P, lin);
% lin_contract returns the applied transform map only when it is asked for two
% outputs; asking for one left mapm empty and the report stage failed on it.
[lc, mapm] = lin_contract(lin);

% Snapshot right after the designs: if anything below fails, the gains are still
% on disk and a second run does not have to redo trim, linearisation and design.
save(fullfile(resdir, 'control_design_snapshot.mat'), 'gains_lqr', 'gains_pid', ...
   'lqr_alt_theta', 'lqr_alt_beta', 'trim', 'S', 'lin', 'lc');
fprintf('snapshot written after the design stage\n');

% ================= V5: LQR =================
v5 = struct();
v5.poles_long_ok = gains_lqr.long.stable;
v5.poles_lat_ok = gains_lqr.lat.stable;
v5.poles_long_unaug_ok = gains_lqr.long.stable_unaug;
v5.poles_lat_unaug_ok = gains_lqr.lat.stable_unaug;
v5.care_diff_long = gains_lqr.long.dK_care;
v5.care_diff_lat = gains_lqr.lat.dK_care;
v5.care_tol = 1e-6;
v5.care_ok = max(v5.care_diff_long, v5.care_diff_lat) < v5.care_tol;
v5.riccati_residual_long = gains_lqr.long.riccati_residual;
v5.riccati_residual_lat = gains_lqr.lat.riccati_residual;
v5.ss_long_unaug = gains_lqr.long.ss_test.unaug.error;
v5.ss_long_aug = gains_lqr.long.ss_test.aug.error;
v5.ss_lat_unaug = gains_lqr.lat.ss_test.unaug.error;
v5.ss_lat_aug = gains_lqr.lat.ss_test.aug.error;
v5.ss_ok = abs(v5.ss_long_unaug) > 1e-3 && abs(v5.ss_long_aug) < 1e-9 && ...
   abs(v5.ss_lat_unaug) > 1e-3 && abs(v5.ss_lat_aug) < 1e-9;

% Reference self-consistency and the two feedforward sets. The residuals come from
% the second output of lqr_reference; they are not on the ref struct.
[ref0, info0] = lqr_reference(P, lin, gains_lqr, struct()); %#ok<ASGLU>
[refc, infoc] = lqr_reference(P, lin, gains_lqr, struct('dV', 2.0, 'dtheta', deg2rad(3), ...
   'dphi', deg2rad(20)));
v5.ref_residual = max([infoc.residual_long, infoc.residual_lat]);
v5.ref_ok = v5.ref_residual < 1e-9;

% Coordinate trap: absolute states fed to a gain designed on deviations.
Kt = gains_lqr.long.K_unaug;
At = gains_lqr.long.A;
Bt = gains_lqr.long.B;
x_trap_ref = gains_lqr.long.x_op;
x_trap_dev = -(At - Bt * Kt) \ (Bt * Kt * x_trap_ref);
x_trap_abs = gains_lqr.long.x_op + x_trap_dev;

v5.ok = v5.poles_long_ok && v5.poles_lat_ok && v5.care_ok && v5.ss_ok && v5.ref_ok;
fprintf('V5: poles long %d lat %d | care diff %.2e | ss err unaug %.4f aug %.2e | ref res %.2e\n', ...
   v5.poles_long_ok, v5.poles_lat_ok, max(v5.care_diff_long, v5.care_diff_lat), ...
   v5.ss_long_unaug, abs(v5.ss_long_aug), v5.ref_residual);

% ================= V6: PID =================
v6 = struct();
loop_names = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
v6.loops = struct();
for k = 1:numel(loop_names)
   nm = loop_names{k};
   v6.loops.(nm) = gains_pid.verify.(nm);
end
v6.separation = gains_pid.verify.separation;

% Closed-loop step cases on the linear model, controller at Ts_ctrl. The block is
% guarded because a single bad case must not cost the whole run.
v6.sim_ok = true;
try
sim_theta = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', deg2rad(5), 'V_step', 0, 'phi_step', 0, 't_end_s', 6));
sim_V = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', 0, 'V_step', 3, 'phi_step', 0, 't_end_s', 20));
sim_h = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'altitude', ...
   'h_step', 20, 'V_step', 0, 'phi_step', 0, 't_end_s', 30));
sim_phi = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', 0, 'V_step', 0, 'phi_step', deg2rad(20), 't_end_s', 8));

v6.theta_step = step_metrics(sim_theta.t, sim_theta.theta, 0.5, deg2rad(5));
v6.V_step = step_metrics(sim_V.t, sim_V.V, 0.5, P.op.cruise.V_mps + 3);
v6.h_step = step_metrics(sim_h.t, sim_h.h, 0.5, P.op.cruise.h_m + 20);
v6.phi_step = step_metrics(sim_phi.t, sim_phi.phi, 0.5, deg2rad(20));
v6.V_step_target_rise_s = 2.2 / P.ctrl.bw.speed_rad_s;
v6.h_step_target_rise_s = 2.2 / P.ctrl.bw.height_rad_s;
v6.theta_step_target_rise_s = 2.2 / P.ctrl.bw.att_theta_rad_s;
v6.phi_step_target_rise_s = 2.2 / P.ctrl.bw.att_phi_rad_s;

% Saturation and anti-windup, P.act and P.prop limits.
sat_on = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', deg2rad(30), 'V_step', 0, 'phi_step', 0, 't_end_s', 8, ...
   'anti_windup', true));
sat_off = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', deg2rad(30), 'V_step', 0, 'phi_step', 0, 't_end_s', 8, ...
   'anti_windup', false));
v6.sat_theta = struct();
v6.sat_theta.on = step_metrics(sat_on.t, sat_on.theta, 0.5, deg2rad(30));
v6.sat_theta.off = step_metrics(sat_off.t, sat_off.theta, 0.5, deg2rad(30));
v6.sat_theta.de_sat_frac_on = mean(sat_on.de_sat);
v6.sat_theta.de_sat_frac_off = mean(sat_off.de_sat);
v6.sat_theta.i_th_max_on = max(abs(sat_on.i_th));
v6.sat_theta.i_th_max_off = max(abs(sat_off.i_th));
% The two arms of the comparison must be able to differ. Before rev 11 this block
% ran both and printed them side by side without ever comparing them, and the two
% were identical to every printed digit: the integral-state clamp was pinning the
% state, so whatever the back-calculation did was absorbed before it reached the
% elevator. A comparison whose arms cannot differ is decorative.
%
% Two differences are checked, because one alone is not enough. The integral STATE
% is where the anti-windup acts and is the sensitive measure; the closed-loop
% RESPONSE is what a user sees but can be insensitive, because while the actuator
% is saturated the response barely moves whichever way the state goes. On the
% airspeed case the state differs by 38.4 out of a 50 bound while the response
% differs by 0.005 m/s of an 8 m/s step, so a response-only criterion would call a
% working anti-windup broken and a state-only criterion would miss a real
% difference in the flown response. Both floors are detectability floors at one
% part in ten thousand of the scale involved, so they can only fail on a genuinely
% degenerate comparison and never on ordinary integration noise.
v6.sat_theta.theta_abs_diff = max(abs(sat_on.theta - sat_off.theta));
v6.sat_theta.i_th_abs_diff = max(abs(sat_on.i_th - sat_off.i_th));
v6.sat_theta.on_pinned_frac = mean(abs(abs(sat_on.i_th) - ...
   P.ctrl.int_state_limit.theta) < 1e-9);
v6.sat_theta.aw_state_diff_min = 1e-4 * P.ctrl.int_state_limit.theta;
v6.sat_theta.aw_resp_diff_min = 1e-4 * deg2rad(30);
v6.sat_theta.pinned_frac_max = 0.10;
v6.sat_theta.aw_state_differs = v6.sat_theta.i_th_abs_diff > v6.sat_theta.aw_state_diff_min;
v6.sat_theta.aw_response_differs = v6.sat_theta.theta_abs_diff > v6.sat_theta.aw_resp_diff_min;
v6.sat_theta.aw_non_degenerate = v6.sat_theta.aw_state_differs && v6.sat_theta.aw_response_differs;
v6.sat_theta.clamp_not_binding = v6.sat_theta.on_pinned_frac < v6.sat_theta.pinned_frac_max;
v6.sat_theta.ok = v6.sat_theta.aw_non_degenerate && v6.sat_theta.clamp_not_binding;

satV_on = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', 0, 'V_step', -8, 'phi_step', 0, 't_end_s', 25, 'anti_windup', true));
satV_off = pid_cascade_sim(P, gains_pid, lc, struct('mode', 'attitude', ...
   'theta_step', 0, 'V_step', -8, 'phi_step', 0, 't_end_s', 25, 'anti_windup', false));
v6.sat_V = struct();
v6.sat_V.on = step_metrics(satV_on.t, satV_on.V, 0.5, P.op.cruise.V_mps - 8);
v6.sat_V.off = step_metrics(satV_off.t, satV_off.V, 0.5, P.op.cruise.V_mps - 8);
v6.sat_V.T_sat_frac_on = mean(satV_on.T_sat);
v6.sat_V.T_sat_frac_off = mean(satV_off.T_sat);
v6.sat_V.V_abs_diff = max(abs(satV_on.V - satV_off.V));
v6.sat_V.i_V_max_on = max(abs(satV_on.i_V));
v6.sat_V.i_V_abs_diff = max(abs(satV_on.i_V - satV_off.i_V));
v6.sat_V.on_pinned_frac = mean(abs(abs(satV_on.i_V) - ...
   P.ctrl.int_state_limit.V) < 1e-9);
v6.sat_V.aw_state_diff_min = 1e-4 * P.ctrl.int_state_limit.V;
v6.sat_V.aw_resp_diff_min = 1e-4 * 8.0;
v6.sat_V.pinned_frac_max = 0.10;
v6.sat_V.aw_state_differs = v6.sat_V.i_V_abs_diff > v6.sat_V.aw_state_diff_min;
v6.sat_V.aw_response_differs = v6.sat_V.V_abs_diff > v6.sat_V.aw_resp_diff_min;
v6.sat_V.aw_non_degenerate = v6.sat_V.aw_state_differs && v6.sat_V.aw_response_differs;
v6.sat_V.clamp_not_binding = v6.sat_V.on_pinned_frac < v6.sat_V.pinned_frac_max;
v6.sat_V.ok = v6.sat_V.aw_non_degenerate && v6.sat_V.clamp_not_binding;

% Coupling numbers for the report.
v6.coupling.beta_peak_on_phi_step_deg = rad2deg(max(abs(sim_phi.beta)));
v6.coupling.r_peak_on_phi_step_dps = rad2deg(max(abs(sim_phi.r)));
v6.coupling.alpha_peak_on_h_step_deg = rad2deg(max(abs(sim_h.alpha - sim_h.alpha(1))));
v6.coupling.theta_peak_on_h_step_deg = rad2deg(max(abs(sim_h.theta)));
v6.coupling.V_drop_on_h_step_mps = max(abs(sim_h.V - P.op.cruise.V_mps));
v6.coupling.h_peak_on_V_step_m = max(abs(sim_V.h - P.op.cruise.h_m));

catch ME6
   v6.sim_ok = false;
   v6.sim_error = sprintf('%s | %s', ME6.identifier, ME6.message);
   fprintf('V6 simulation block failed: %s\n', v6.sim_error);
   for kk = 1:numel(ME6.stack)
      fprintf('   at %s line %d\n', ME6.stack(kk).name, ME6.stack(kk).line);
   end
   v6.theta_step = struct('rise_time_s', NaN, 'overshoot_pct', NaN, 'error_ss', NaN);
   v6.V_step = v6.theta_step;
   v6.h_step = v6.theta_step;
   v6.phi_step = v6.theta_step;
   v6.V_step_target_rise_s = 2.2 / P.ctrl.bw.speed_rad_s;
   v6.h_step_target_rise_s = 2.2 / P.ctrl.bw.height_rad_s;
   v6.theta_step_target_rise_s = 2.2 / P.ctrl.bw.att_theta_rad_s;
   v6.phi_step_target_rise_s = 2.2 / P.ctrl.bw.att_phi_rad_s;
   v6.sat_theta = struct('on', v6.theta_step, 'off', v6.theta_step, ...
      'de_sat_frac_on', NaN, 'de_sat_frac_off', NaN, 'i_th_max_on', NaN, ...
      'i_th_max_off', NaN, 'theta_abs_diff', NaN, 'i_th_abs_diff', NaN, ...
      'on_pinned_frac', NaN, 'aw_state_diff_min', NaN, 'aw_resp_diff_min', NaN, ...
      'pinned_frac_max', NaN, 'aw_state_differs', false, ...
      'aw_response_differs', false, 'aw_non_degenerate', false, ...
      'clamp_not_binding', false, 'ok', false);
   v6.sat_V = struct('on', v6.theta_step, 'off', v6.theta_step, ...
      'T_sat_frac_on', NaN, 'T_sat_frac_off', NaN, 'V_abs_diff', NaN, ...
      'i_V_max_on', NaN, 'i_V_abs_diff', NaN, 'on_pinned_frac', NaN, ...
      'aw_state_diff_min', NaN, 'aw_resp_diff_min', NaN, 'pinned_frac_max', NaN, ...
      'aw_state_differs', false, 'aw_response_differs', false, ...
      'aw_non_degenerate', false, 'clamp_not_binding', false, 'ok', false);
   v6.coupling = struct('beta_peak_on_phi_step_deg', NaN, 'r_peak_on_phi_step_dps', NaN, ...
      'alpha_peak_on_h_step_deg', NaN, 'theta_peak_on_h_step_deg', NaN, ...
      'V_drop_on_h_step_mps', NaN, 'h_peak_on_V_step_m', NaN);
end

v6.ok = all(cellfun(@(n) v6.loops.(n).stable && ...
   (v6.loops.(n).overshoot_ok || isnan(v6.loops.(n).overshoot_pct)), loop_names)) && ...
   v6.separation.ok && v6.theta_step.overshoot_pct < 20 && v6.phi_step.overshoot_pct < 20 && ...
   v6.sat_theta.ok && v6.sat_V.ok;
fprintf('V6: theta rise %.3f s / target %.3f | V rise %.3f / %.3f | h rise %.3f / %.3f | phi rise %.3f / %.3f\n', ...
   v6.theta_step.rise_time_s, v6.theta_step_target_rise_s, v6.V_step.rise_time_s, ...
   v6.V_step_target_rise_s, v6.h_step.rise_time_s, v6.h_step_target_rise_s, ...
   v6.phi_step.rise_time_s, v6.phi_step_target_rise_s);

% ================= V7: allocation =================
v7 = struct();
da_lim = deg2rad(P.act.aileron.max_deg);
de_lim = deg2rad(P.act.elevator.max_deg);
dr_lim = deg2rad(P.act.rudder.max_deg);
T_trim = trim.thrust_total_N;
N_rud = S.authority.rudder.yaw_moment_Nm;
L_ail = S.authority.aileron.roll_moment_Nm;
M_ele = S.authority.elevator.pitch_moment_Nm;

rng(0);
M_test = [0.3 * L_ail, 0, 0; 0, 0.3 * M_ele, 0; 0, 0, 0.3 * N_rud]';
M_test = [M_test, [0.2 * L_ail; -0.25 * M_ele; 0.2 * N_rud], ...
   [-0.3 * L_ail; 0.1 * M_ele; -0.3 * N_rud]];
M_test = [M_test, (rand(3, 6) - 0.5) .* (0.4 * [L_ail; M_ele; N_rud])];
n_t = size(M_test, 2);
v7.recon_rel = zeros(1, n_t);
v7.recon_abs = zeros(1, n_t);
v7.sat_any = false(1, n_t);
v7.in_limits = false(1, n_t);
t_alloc = tic;
for j = 1:n_t
   [da, de, dr, dthl, dthr, ia] = allocate(P, M_test(:, j), T_trim);
   v7.recon_rel(j) = ia.M_err_rel;
   v7.recon_abs(j) = ia.M_err_norm;
   v7.sat_any(j) = ia.sat_any;
   v7.in_limits(j) = abs(da) <= da_lim + 1e-12 && abs(de) <= de_lim + 1e-12 && ...
      abs(dr) <= dr_lim + 1e-12 && dthl >= -1e-12 && dthl <= 1 + 1e-12 && ...
      dthr >= -1e-12 && dthr <= 1 + 1e-12;
end
v7.alloc_ms_mean = 1000 * toc(t_alloc) / n_t;
v7.recon_max_rel = max(v7.recon_rel);
v7.recon_tol = 1e-9;
v7.recon_ok = v7.recon_max_rel < v7.recon_tol;
v7.unsat_all_ok = all(v7.in_limits(~v7.sat_any));

% Saturation: five times the available authority on every axis.
M_huge = [4 * L_ail; 4 * M_ele; 4 * N_rud];
[da_s, de_s, dr_s, dthl_s, dthr_s, ia_s] = allocate(P, M_huge, T_trim);
d_eff_alloc = ia_s.limits.d_eff;
v7.sat = struct('da', da_s, 'de', de_s, 'dr', dr_s, 'dthl', dthl_s, 'dthr', dthr_s);
v7.sat_limits_ok = abs(da_s) <= da_lim + 1e-12 && abs(de_s) <= de_lim + 1e-12 && ...
   abs(dr_s) <= dr_lim + 1e-12 && all([dthl_s dthr_s] >= -1e-12) && ...
   all([dthl_s dthr_s] <= 1 + 1e-12) && ...
   abs(dthr_s - dthl_s) <= 2 * d_eff_alloc + 1e-12;
v7.sat_ok = ia_s.sat_any && v7.sat_limits_ok;
v7.sat_M_err = ia_s.M_err;
v7.sat_d_eff = d_eff_alloc;
v7.sat_throttle_sum_ok = abs((dthl_s + dthr_s) / 2 - ia_s.tc) < 1e-12;
v7.tc_cruise = ia_s.tc;
% Differential limit at the cruise throttle, against the sizing budget.
dT_lim_cruise = ia_s.dT_lim;
v7.dT_lim_cruise = dT_lim_cruise;
v7.yaw_diff_alloc_Nm = abs(dT_lim_cruise) * P.prop.y_motor;
v7.yaw_diff_sizing_Nm = S.authority.diff_thrust.yaw_moment_Nm;
v7.d_eff_sizing = S.authority.diff_thrust.d_effective;
v7.d_eff_diff = abs(ia_s.limits.d_eff - S.authority.diff_thrust.d_effective);
v7.ok = v7.recon_ok && v7.unsat_all_ok && v7.sat_ok && v7.sat_throttle_sum_ok;
fprintf('V7: recon rel max %.3e | sat inside limits %d | d_eff %.4f vs sizing %.4f\n', ...
   v7.recon_max_rel, v7.sat_limits_ok, ia_s.limits.d_eff, v7.d_eff_sizing);

% Allocator model against the linearised plant: implied moment derivatives.
% columns 1..3 are the aerodynamic effectors, column 4 is the differential
% thrust T_r - T_l, which acts on yaw alone; the plant's B(:,6) is the total
% thrust command, quoted separately because it acts on pitch alone.
Imat = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; 0, P.mass.Iyy_cg, 0; ...
   -P.mass.Ixz_cg, 0, P.mass.Izz_cg];
Bmom = Imat * lin.B([4 5 6], :);
G_alloc = ia_s.G;
y_arm = P.prop.y_motor - P.mass.y_cg;
G_plant = [Bmom(:, 1), Bmom(:, 2), Bmom(:, 3), [0; 0; y_arm]];
scale = max(abs(G_plant(:)));
v7.model_err_rel = abs(G_alloc - G_plant) / scale;
v7.model_err_max = max(v7.model_err_rel(:));
v7.M_per_dT_plant = Bmom(:, 6);
v7.M_per_dT_expect = [0; P.prop.z_thrust_offset; 0];
v7.M_per_dT_err = max(abs(v7.M_per_dT_plant - v7.M_per_dT_expect)) / scale;

% ================= V8: differential thrust =================
v8 = struct();
dT_test = S.authority.diff_thrust.thrust_left_N - S.authority.diff_thrust.thrust_right_N;
Tm = thrust_moment(P, S.authority.diff_thrust.thrust_left_N, ...
   S.authority.diff_thrust.thrust_right_N);
v8.analytic_Nm = y_arm * dT_test;
v8.plant_Nm = Tm(3);
v8.plant_vs_analytic_rel = abs(v8.plant_Nm - v8.analytic_Nm) / abs(v8.analytic_Nm);
% Sweep the identity thrust_moment against dT*y_motor over the whole differential
% range, including the counter-rotating sign, so V8 does not rest on one point.
T_c = T_trim / 2;
dT_sweep = linspace(-0.95 * T_c, 0.95 * T_c, 10);
v8.sweep_rel = zeros(size(dT_sweep));
for k = 1:numel(dT_sweep)
   Tl = T_c - dT_sweep(k) / 2;
   Tr = T_c + dT_sweep(k) / 2;
   Tmk = thrust_moment(P, Tl, Tr);
   den = max(abs(y_arm * (Tr - Tl)), eps);
   v8.sweep_rel(k) = abs(abs(Tmk(3)) - abs(y_arm * (Tr - Tl))) / den;
end
v8.sweep_max_rel = max(v8.sweep_rel);
v8.sizing_Nm = S.authority.diff_thrust.yaw_moment_Nm;
v8.alloc_Nm = v7.yaw_diff_alloc_Nm;
v8.tie_rel = abs(v8.alloc_Nm - v8.sizing_Nm) / v8.sizing_Nm;
v8.rudder_Nm = N_rud;
v8.ratio_to_rudder = v8.sizing_Nm / N_rud;
v8.ratio_to_rudder_alloc = v8.alloc_Nm / N_rud;
v8.ratio_at_cap = S.authority.diff_thrust.yaw_moment_at_cap_Nm / N_rud;
v8.limiting_factor = S.authority.diff_thrust.limiting_factor;
v8.tol = 0.01;
v8.ok = v8.sweep_max_rel < v8.tol;
% Allocator path: a yaw command beyond the rudder alone forces the supplement.
M_yaw_big = [0; 0; 1.35 * N_rud];
[da_y, de_y, dr_y, dthl_y, dthr_y, ia_y] = allocate(P, M_yaw_big, T_trim);
Tl_y = throttle_to_thrust(P, dthl_y, P.op.cruise.V_mps);
Tr_y = throttle_to_thrust(P, dthr_y, P.op.cruise.V_mps);
Tmy = thrust_moment(P, Tl_y, Tr_y);
v8.forced = struct('dr', dr_y, 'dthl', dthl_y, 'dthr', dthr_y, ...
   'M_ach', ia_y.M_ach, 'sat', ia_y.sat, 'ratio', ia_y.authority.ratio_rudder_over_diff);
v8.forced_N_diff = abs(Tmy(3));
v8.forced_N_expect = abs(y_arm * (Tr_y - Tl_y));
v8.forced_tie_rel = abs(v8.forced_N_diff - v8.forced_N_expect) / v8.forced_N_expect;
fprintf('V8: sweep vs analytic max %.3e rel | diff yaw alloc %.2f Nm vs sizing %.2f | ratio to rudder %.4f\n', ...
   v8.sweep_max_rel, v8.alloc_Nm, v8.sizing_Nm, v8.ratio_to_rudder);

% ================= reports =================
% The whole report stage is guarded: the numbers are already in checks, so a
% formatting slip here must not cost the run. The status file is cleared before
% the stage and written on BOTH outcomes, so it always describes the last run.
% Writing it only on failure left a stale failure note on disk for an hour after
% the fault was fixed, and the next reader took it for the current state.
status_file = fullfile(resdir, 'control_design_status.txt');
if exist(status_file, 'file')
   delete(status_file);
end
report_ok = true;
try
Rl = {};
Rl{end+1} = 'UACS twin: LQR design report (V5)';
Rl{end+1} = sprintf('generated %s by control/uacs_job_control_design.m', datestr(now));
Rl{end+1} = sprintf('MATLAB %s, P %s rev %g', version('-release'), P.meta.version, p_rev);
Rl{end+1} = '';
Rl{end+1} = sprintf('design point: V %.4f m/s, alpha %.4f deg, theta %.4f deg, de %.4f deg, throttle %.4f', ...
   trim.V_mps, trim.alpha_deg, trim.theta_deg, trim.de_deg, trim.throttle);
Rl{end+1} = sprintf('trim residual max %.3e, exitflag %d, Jacobian step %.1e', ...
   trim.residual_norm_max, trim.exitflag, lin_info.step);
Rl{end+1} = sprintf('contract mapping of the sub-models: %s (%s)', mapm.mode, mapm.reason);
if strcmp(mapm.mode, 'identity')
   Rl{end+1} = '  linearize_at already returned air-data states, no rotation applied';
else
   Rl{end+1} = sprintf('  T_long = [%s]', mat2str(mapm.T_long, 6));
   Rl{end+1} = sprintf('  V0 %.6f m/s, alpha0 %.6f rad: dV = cos(a0)du + sin(a0)dw, dalpha = (-sin(a0)du + cos(a0)dw)/V0, dbeta = dv/V0', ...
      mapm.V0, mapm.alpha0);
end
Rl{end+1} = '';
Rl{end+1} = '--- weights, straight from P.lqr (Bryson), no edit ---';
Rl{end+1} = sprintf('long q_diag = %s', mat2str(P.lqr.long.q_diag, 6));
Rl{end+1} = sprintf('long r_diag = %s', mat2str(P.lqr.long.r_diag, 6));
Rl{end+1} = sprintf('lat  q_diag = %s', mat2str(P.lqr.lat.q_diag, 6));
Rl{end+1} = sprintf('lat  r_diag = %s', mat2str(P.lqr.lat.r_diag, 6));
Rl{end+1} = sprintf('long integrator on %s, q_int = %.6f (rule: same Bryson tolerance as that state)', ...
   gains_lqr.long.int_output, gains_lqr.long.q_int);
Rl{end+1} = sprintf('lat  integrator on %s, q_int = %.6f', ...
   gains_lqr.lat.int_output, gains_lqr.lat.q_int);

Rl = [Rl, lqr_report_block('LONGITUDINAL [dV dalpha dq dtheta] x [dde dT]', ...
   gains_lqr.long, P, trim)];
Rl = [Rl, lqr_report_block('LATERAL-DIRECTIONAL [dbeta dp dr dphi] x [dda ddr]', ...
   gains_lqr.lat, P, trim)];

Rl{end+1} = '--- V5 checks ---';
Rl{end+1} = sprintf('closed-loop poles, augmented long: %s', mat2str(gains_lqr.long.poles, 6));
Rl{end+1} = sprintf('closed-loop poles, augmented lat : %s', mat2str(gains_lqr.lat.poles, 6));
Rl{end+1} = sprintf('all poles in the open left half plane: long %d, lat %d', ...
   v5.poles_long_ok, v5.poles_lat_ok);
Rl{end+1} = sprintf('max |K(lqr) - R\\B''X(care)| : long %.3e, lat %.3e (tolerance %.0e) -> %d', ...
   v5.care_diff_long, v5.care_diff_lat, v5.care_tol, v5.care_ok);
Rl{end+1} = sprintf('Riccati residual ||A''X + XA - XBR^-1B''X + Q|| : long %.3e, lat %.3e', ...
   v5.riccati_residual_long, v5.riccati_residual_lat);
Rl{end+1} = '';
Rl{end+1} = '--- V5 steady-state error, augmented versus not ---';
Rl{end+1} = sprintf('long: step %.3f m/s on dV. No integrator: settled %.6f m/s, error %+.6f m/s (%.2f%%)', ...
   gains_lqr.long.ss_test.step, gains_lqr.long.ss_test.unaug.out_ss, ...
   gains_lqr.long.ss_test.unaug.error, 100 * gains_lqr.long.ss_test.unaug.error_rel);
Rl{end+1} = sprintf('long: with integrator: settled %.9f m/s, error %+.3e m/s', ...
   gains_lqr.long.ss_test.aug.out_ss, gains_lqr.long.ss_test.aug.error);
Rl{end+1} = sprintf('long: unaugmented equilibrium state %s', ...
   mat2str(gains_lqr.long.ss_test.unaug.x_ss', 6));
Rl{end+1} = sprintf('long: augmented   equilibrium state %s, integrator state %.6f', ...
   mat2str(gains_lqr.long.ss_test.aug.x_ss', 6), gains_lqr.long.ss_test.aug.z_ss);
Rl{end+1} = sprintf('lat : step %.4f rad on dphi. No integrator: settled %.6f rad, error %+.6f rad', ...
   gains_lqr.lat.ss_test.step, gains_lqr.lat.ss_test.unaug.out_ss, ...
   gains_lqr.lat.ss_test.unaug.error);
Rl{end+1} = sprintf('lat : with integrator: settled %.9f rad, error %+.3e rad', ...
   gains_lqr.lat.ss_test.aug.out_ss, gains_lqr.lat.ss_test.aug.error);
Rl{end+1} = sprintf('V5 steady-state verdict: %d (needs a real error without the integrator and none with it)', v5.ss_ok);
Rl{end+1} = '';
Rl{end+1} = '--- the two feedforward values, never to be confused ---';
Rl{end+1} = 'set 1, design point (deviation coordinates): x_ref = 0, z_ref = 0, u_ff = [0;0]';
Rl{end+1} = sprintf('  the same point in absolute actuator units: u_star(de, dT) = [%.9f rad, %.6f N] with the throttles at %.6f', ...
   gains_lqr.long.u_star_full(2), gains_lqr.long.u_star_full(6), gains_lqr.long.u_star_full(4));
Rl{end+1} = sprintf('set 2, commanded point dV = %.3f m/s, dtheta = %.4f rad:', infoc.cmd.dV, infoc.cmd.dtheta);
Rl{end+1} = sprintf('  x_ref (deviation) = %s', mat2str(refc.long.x_ref', 6));
Rl{end+1} = sprintf('  u_ff  (deviation) = %s  [dde rad; dT N]', mat2str(refc.long.u_ff', 6));
Rl{end+1} = sprintf('  x_ref (absolute)  = [V %.6f, alpha %.6f, q %.6f, theta %.6f]', refc.long.x_ref_abs);
Rl{end+1} = sprintf('  u_abs (absolute)  = [de %.6f rad, dT %.6f N]', refc.long.u_abs(2), refc.long.u_abs(6));
Rl{end+1} = sprintf('  lateral, dphi = %.4f rad: x_ref = %s, u_ff = %s, residual %.2e', ...
   infoc.cmd.dphi, mat2str(refc.lat.x_ref', 6), mat2str(refc.lat.u_ff', 6), infoc.residual_lat);
Rl{end+1} = sprintf('equilibrium residual ||A*x_ref + B*u_ff|| = %.3e (long), %.3e (lat), threshold 1e-9', ...
   infoc.residual_long, infoc.residual_lat);
Rl{end+1} = '';
Rl{end+1} = '--- the coordinate trap, quantified ---';
Rl{end+1} = 'if the absolute trim state is used as the reference of a gain designed on deviations,';
Rl{end+1} = 'the unaugmented loop parks at:';
Rl{end+1} = sprintf('  x_dev = %s', mat2str(x_trap_dev', 6));
Rl{end+1} = sprintf('  absolute state = [V %.4f, alpha %.4f rad, q %.4f, theta %.4f rad]', x_trap_abs);
Rl{end+1} = sprintf('  intended      = [V %.4f, alpha %.4f rad, q %.4f, theta %.4f rad]', gains_lqr.long.x_op);
Rl{end+1} = sprintf('  error         = [%.4f m/s, %.4f rad, %.4f rad/s, %.4f rad]', x_trap_abs - gains_lqr.long.x_op);
Rl{end+1} = 'always feed x_dev = x_abs - x_op, and put the commanded point in x_ref.';
Rl{end+1} = '';
Rl{end+1} = '--- integrator output choice, alternative designs ---';
Rl = [Rl, lqr_report_block('long integrator on theta', lqr_alt_theta.long, P, trim, 'alt')];
Rl = [Rl, lqr_report_block('lat integrator on beta', lqr_alt_beta.lat, P, trim, 'alt')];
Rl{end+1} = sprintf('V5 verdict: %d', v5.ok);

Rp = {};
Rp{end+1} = 'UACS twin: PID cascade design report (V6)';
Rp{end+1} = sprintf('generated %s by control/uacs_job_control_design.m', datestr(now));
Rp{end+1} = sprintf('parameter set rev %g, P %s', p_rev, P.meta.version);
Rp{end+1} = sprintf('design rules: Kp from unit loop magnitude at the P.ctrl.bw crossover; integral zero at %.1f*wc for a plant with a pole at the origin and %.1f*wc otherwise', ...
   gains_pid.rules.zero_frac_type1, gains_pid.rules.zero_frac_type0);
Rp{end+1} = sprintf('D filter time constant %.4f s, anti-windup back-calculation gain %.2f', ...
   gains_pid.rate_filter_tau_s, gains_pid.anti_windup_Kb);
Rp{end+1} = sprintf('  aileron %.4f s, elevator %.4f s, rudder %.4f s, ESC %.4f s', ...
   P.act.aileron.tau_s, P.act.elevator.tau_s, P.act.rudder.tau_s, P.prop.esc.tau_s);
Rp{end+1} = '';
Rp{end+1} = '--- gains, u = Kp*e + Ki*int(e) + Kd*Df(e), signs already folded in ---';
names = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
for k = 1:numel(names)
   g = gains_pid.(names{k});
   extra = '';
   if isfield(g, 'int_limit')
      % g.int_limit is the ERROR-domain bound. The clamp the integrator actually
      % carries is the state-domain field, and printing only the error bound is how
      % the model and the report came to disagree by 2.5x without anyone noticing.
      if isfield(P.ctrl.int_state_limit, g.name)
         extra = sprintf(' err_limit %.5f state_clamp %.5f', g.int_limit, ...
            P.ctrl.int_state_limit.(g.name));
      else
         extra = sprintf(' surface_limit %.5f', g.int_limit);
      end
   end
   Rp{end+1} = sprintf('%-6s Kp %+12.6f  Ki %+12.6f  Kd %+12.6f  Tf %.4f  Kb %.2f  wc_target %.2f  sign %+d%s', ...
      g.name, g.Kp, g.Ki, g.Kd, g.Tf, g.Kb, g.wc_target_rad_s, g.sign, extra);
end
Rp{end+1} = '';
Rp{end+1} = '--- where the search had to give ground for stability ---';
Rp{end+1} = sprintf('%-6s %10s %10s %9s %9s %9s %6s %6s', 'loop', 'kp_nominal', ...
   'kp_used', 'scale', 'zf_nom', 'zf_used', 'gain_red', 'int_red');
for k = 1:numel(names)
   g = gains_pid.(names{k});
   if isfield(g, 'kp_nominal')
      Rp{end+1} = sprintf('%-6s %10.5f %10.5f %9.3f %9.4f %9.4f %6d %6d', ...
         g.name, g.kp_nominal, g.Kp_mag, g.kp_scale, g.zero_frac_nominal, ...
         g.zero_frac, g.gain_reduced, g.int_reduced);
   else
      Rp{end+1} = sprintf('%-6s %10.5f %10.5f %9s %9s %9s %6s %6s', g.name, ...
         g.Kp_mag, g.Kp_mag, 'n/a', 'n/a', 'n/a', 'n/a', 'n/a');
   end
end
Rp{end+1} = 'kp_nominal is the value the crossover rule asks for, kp_used what the loop was';
Rp{end+1} = 'designed with after the closed-loop stability search. A scale below one means the';
Rp{end+1} = 'roll attitude gain had to come down to keep the spiral pole inside the left half';
Rp{end+1} = 'plane, which is expected: that loop cancels the dihedral effect the spiral needs.';
Rp{end+1} = '';
Rp{end+1} = '--- per loop margins and step metrics, loop closed through the actuator lag ---';
Rp{end+1} = sprintf('%-6s %8s %8s %8s %9s %9s %9s %9s %9s %9s', 'loop', 'gm', 'gm_dB', ...
   'pm_deg', 'dm_s', 'rise_s', 'over_pct', 'bw_rad_s', 'wcp_rad_s', 'wc_tgt');
for k = 1:numel(names)
   v = gains_pid.verify.(names{k});
   % wcp_rad_s is the ACHIEVED crossover and wc_tgt the target the design aimed at.
   % The achieved one was previously printed under the header wc_rad_s, so the table
   % showed the pitch rate loop at 10.00 rad/s when its loop actually crosses unity at
   % 1.93, and reading that number as the achieved crossover inverts the sign of the
   % cascade separation verdict. Both are printed now.
   Rp{end+1} = sprintf('%-6s %8.3f %8.2f %8.1f %9.5f %9.4f %9.2f %9.3f %9.3f %9.2f', ...
      v.name, v.gm, v.gm_db, v.pm_deg, v.dm_s, v.rise_time_s, v.overshoot_pct, ...
      v.bandwidth_rad_s, v.wcp_rad_s, v.wc_target_rad_s);
end
Rp{end+1} = 'rise_s is the 10-90 percent rise time of the closed loop, over_pct its overshoot,';
Rp{end+1} = 'dm_s the delay margin in seconds, bw_rad_s the closed-loop -3 dB bandwidth,';
Rp{end+1} = 'wcp_rad_s the achieved crossover and wc_tgt the crossover the design aimed at.';
Rp{end+1} = 'The rate loops are near differentiators, so bw_rad_s is NaN for them by construction';
Rp{end+1} = 'and the separation check reads wcp_rad_s instead.';
Rp{end+1} = '';
Rp{end+1} = '--- cascade separation, outer bandwidth over the inner it commands ---';
s = gains_pid.verify.separation;
Rp{end+1} = sprintf('phi/p        %.3f  (target %.3f)', s.phi_over_p, s.target.phi_over_p);
Rp{end+1} = sprintf('theta/q      %.3f  (target %.3f)', s.theta_over_q, s.target.theta_over_q);
Rp{end+1} = sprintf('V/q          %.3f  (target %.3f)', s.V_over_q, s.target.V_over_q);
Rp{end+1} = sprintf('h/theta      %.3f  (target %.3f)', s.h_over_theta, s.target.h_over_theta);
Rp{end+1} = sprintf('h/V          %.3f  (target %.3f)', s.h_over_V, s.target.h_over_V);
Rp{end+1} = sprintf('separation verdict (every ratio below %.1f): %d', s.limit, s.ok);
% A ratio above one means the OUTER loop is faster than the INNER loop it commands,
% which breaks the cascade assumption the whole design rests on. It is printed with
% the offending axes named, because this verdict has been 0 since before rev 10 and
% was never written down anywhere: the acceptance record's V6 is a different check
% (v_sim_v6_pid, on the model trajectory) and does not look at separation at all.
sep_fields = {'phi_over_p', 'theta_over_q', 'V_over_q', 'h_over_theta', 'h_over_V'};
sep_names = {'phi/p', 'theta/q', 'V/q', 'h/theta', 'h/V'};
sep_bad = {};
for k = 1:numel(sep_fields)
   if s.(sep_fields{k}) >= s.limit
      sep_bad{end+1} = sprintf('%s %.3f', sep_names{k}, s.(sep_fields{k})); %#ok<AGROW>
   end
end
if isempty(sep_bad)
   Rp{end+1} = 'no ratio is at or above the limit';
else
   Rp{end+1} = sprintf('at or above the limit: %s', strjoin(sep_bad, ', '));
   Rp{end+1} = 'a ratio at or above one means the outer loop is faster than its inner loop, which is';
   Rp{end+1} = 'a design-margin defect and not an instability: the eigenvalue checks still pass.';
end
Rp{end+1} = '';
Rp{end+1} = '--- closed-loop step response on the linear model, controller at Ts_ctrl ---';
Rp{end+1} = sprintf('theta_c 5 deg : rise %.4f s (target %.4f), overshoot %.2f %%, settled error %.3e rad', ...
   v6.theta_step.rise_time_s, v6.theta_step_target_rise_s, v6.theta_step.overshoot_pct, v6.theta_step.error_ss);
Rp{end+1} = sprintf('V_c +3 m/s    : rise %.4f s (target %.4f), overshoot %.2f %%, settled error %.3e m/s', ...
   v6.V_step.rise_time_s, v6.V_step_target_rise_s, v6.V_step.overshoot_pct, v6.V_step.error_ss);
Rp{end+1} = sprintf('h_c +20 m     : rise %.4f s (target %.4f), overshoot %.2f %%, settled error %.3e m', ...
   v6.h_step.rise_time_s, v6.h_step_target_rise_s, v6.h_step.overshoot_pct, v6.h_step.error_ss);
Rp{end+1} = sprintf('phi_c 20 deg  : rise %.4f s (target %.4f), overshoot %.2f %%, settled error %.3e rad', ...
   v6.phi_step.rise_time_s, v6.phi_step_target_rise_s, v6.phi_step.overshoot_pct, v6.phi_step.error_ss);
Rp{end+1} = '';
Rp{end+1} = '--- saturation and anti-windup, limits from P.act and P.prop ---';
Rp{end+1} = sprintf('theta_c 30 deg step: elevator saturated for %.1f %% of the run with anti-windup, %.1f %% without', ...
   100 * v6.sat_theta.de_sat_frac_on, 100 * v6.sat_theta.de_sat_frac_off);
Rp{end+1} = sprintf('  with anti-windup: peak %.4f rad, overshoot %.2f %%, settling %.3f s, max|int| %.4f', ...
   v6.sat_theta.on.peak, v6.sat_theta.on.overshoot_pct, v6.sat_theta.on.settling_time_s, v6.sat_theta.i_th_max_on);
Rp{end+1} = sprintf('  without:         peak %.4f rad, overshoot %.2f %%, settling %.3f s, max|int| %.4f', ...
   v6.sat_theta.off.peak, v6.sat_theta.off.overshoot_pct, v6.sat_theta.off.settling_time_s, v6.sat_theta.i_th_max_off);
Rp{end+1} = sprintf('  the two arms differ by %.6f rad of pitch and %.6f of integral state; the state difference is the sensitive one and its detectability floor is %.6f', ...
   v6.sat_theta.theta_abs_diff, v6.sat_theta.i_th_abs_diff, v6.sat_theta.aw_state_diff_min);
Rp{end+1} = sprintf('  the integral-state clamp holds the state on %.2f %% of the run; above %.1f %% the clamp is the anti-windup and the back-calculation is not, so it is a criterion too', ...
   100 * v6.sat_theta.on_pinned_frac, 100 * v6.sat_theta.pinned_frac_max);
Rp{end+1} = sprintf('  state arms differ: %d, response arms differ: %d, clamp not binding: %d', ...
   v6.sat_theta.aw_state_differs, v6.sat_theta.aw_response_differs, ...
   v6.sat_theta.clamp_not_binding);
Rp{end+1} = sprintf('V_c -8 m/s step: throttle saturated for %.1f %% of the run with anti-windup, %.1f %% without', ...
   100 * v6.sat_V.T_sat_frac_on, 100 * v6.sat_V.T_sat_frac_off);
Rp{end+1} = sprintf('  with anti-windup: undershoot %.2f %%, settling %.3f s, max|int| %.4f, clamp holds %.2f %% of the run', ...
   v6.sat_V.on.overshoot_pct, v6.sat_V.on.settling_time_s, v6.sat_V.i_V_max_on, ...
   100 * v6.sat_V.on_pinned_frac);
Rp{end+1} = sprintf('  without:         undershoot %.2f %%, settling %.3f s', ...
   v6.sat_V.off.overshoot_pct, v6.sat_V.off.settling_time_s);
Rp{end+1} = sprintf('  the two arms differ by %.6f m/s of airspeed and %.6f of integral state; the state difference is the sensitive one', ...
   v6.sat_V.V_abs_diff, v6.sat_V.i_V_abs_diff);
Rp{end+1} = sprintf('  state arms differ: %d, response arms differ: %d, clamp not binding: %d', ...
   v6.sat_V.aw_state_differs, v6.sat_V.aw_response_differs, v6.sat_V.clamp_not_binding);
Rp{end+1} = '';
Rp{end+1} = '--- coupling, measured on the linear closed loop ---';
Rp{end+1} = sprintf('pitch/yaw: a 20 deg bank command produces peak beta %.3f deg and peak r %.3f deg/s', ...
   v6.coupling.beta_peak_on_phi_step_deg, v6.coupling.r_peak_on_phi_step_dps);
Rp{end+1} = sprintf('speed/height: a +20 m height command produces peak alpha excursion %.3f deg and %.3f m/s speed loss', ...
   v6.coupling.alpha_peak_on_h_step_deg, v6.coupling.V_drop_on_h_step_mps);
Rp{end+1} = sprintf('             a +3 m/s speed command produces %.3f m height excursion', ...
   v6.coupling.h_peak_on_V_step_m);
Rp{end+1} = 'handling: h drives theta_c and V drives thrust, the pairing that keeps the phugoid out of the';
Rp{end+1} = 'altitude loop; the yaw loop is left on the pilot stick, coordination is not implemented in v1.';
Rp{end+1} = sprintf('V6 verdict: %d', v6.ok);

Ra = {};
Ra{end+1} = 'UACS twin: control allocation report (V7, V8)';
Ra{end+1} = sprintf('generated %s by control/uacs_job_control_design.m', datestr(now));
Ra{end+1} = sprintf('parameter set rev %g, P %s', p_rev, P.meta.version);
Ra{end+1} = sprintf('allocation linearisation point: V %.2f m/s, qbar %.2f Pa', ...
   P.op.cruise.V_mps, 0.5 * P.env.rho_sl * P.op.cruise.V_mps^2);
Ra{end+1} = sprintf('effectiveness G, rows [L M N], columns [da de dr dT_rl], dT_rl = T_r - T_l:');
for r = 1:3
   Ra{end+1} = sprintf('  %s', mat2str(G_alloc(r, :), 6));
end
Ra{end+1} = sprintf('against the linearised plant (I*B(4:6,:)), max difference %.3e of the largest derivative', ...
   v7.model_err_max);
Ra{end+1} = sprintf('total thrust column I*B(4:6,6) = %s, expected [0; z_arm; 0] = %s, difference %.3e', ...
   mat2str(v7.M_per_dT_plant', 6), mat2str(v7.M_per_dT_expect', 6), v7.M_per_dT_err);
Ra{end+1} = 'priority: elevator first and alone for pitch; then aileron and rudder; differential thrust last,';
Ra{end+1} = sprintf('weighted by its own authority ratio, %.4f rudder-equivalents at the commanded throttle', ...
   ia_s.authority.ratio_rudder_over_diff);
Ra{end+1} = sprintf('weights [da dr dT] = %s (normalised by each limit, larger means more expensive)', ...
   mat2str(ia_s.priority', 6));
Ra{end+1} = '';
Ra{end+1} = '--- V7 unsaturated reconstruction ---';
Ra{end+1} = sprintf('%d moment commands at up to 30 percent of each axis authority, max relative error %.3e (tolerance %.0e)', ...
   n_t, v7.recon_max_rel, v7.recon_tol);
Ra{end+1} = sprintf('max absolute error %.3e N*m, all outputs inside limits: %d', ...
   max(v7.recon_abs), v7.unsat_all_ok);
Ra{end+1} = '--- V7 saturation ---';
Ra{end+1} = sprintf('command [L M N] = %s N*m, four times each authority', mat2str(M_huge', 6));
Ra{end+1} = sprintf('outputs da %.6f, de %.6f, dr %.6f rad; dth_l %.6f, dth_r %.6f', ...
   da_s, de_s, dr_s, dthl_s, dthr_s);
Ra{end+1} = sprintf('limits da/de/dr %.6f rad, differential %.6f throttle (headroom %.6f, structural cap %.6f)', ...
   da_lim, d_eff_alloc, ia_s.limits.d_headroom, ia_s.limits.d_cap);
Ra{end+1} = sprintf('inside every limit: %d; throttle sum preserved: %d', v7.sat_limits_ok, v7.sat_throttle_sum_ok);
Ra{end+1} = sprintf('unmet moment demand [dL dM dN] = %s N*m', mat2str(v7.sat_M_err', 6));
Ra{end+1} = '';
Ra{end+1} = '--- V8 differential thrust, authoritative numbers from sizing/sizing_summary.m ---';
Ra{end+1} = sprintf('cruise throttle %.4f, headroom %.4f, structural cap %.4f, effective %.4f, limiting factor: %s', ...
   S.prop.throttle_cruise, S.authority.diff_thrust.d_headroom, ...
   S.authority.diff_thrust.d_cap, S.authority.diff_thrust.d_effective, v8.limiting_factor);
Ra{end+1} = sprintf('symmetric modulation with dT = T_L - T_R = %.4f N gives yaw %.4f N*m', ...
   dT_test, v8.sizing_Nm);
Ra{end+1} = sprintf('analytic y_motor*dT = %.4f * %.4f = %.4f N*m', P.prop.y_motor, dT_test, v8.analytic_Nm);
Ra{end+1} = sprintf('plant/thrust_moment returns %.4f N*m, relative difference %.3e (tolerance %.0e)', ...
   v8.plant_Nm, v8.plant_vs_analytic_rel, v8.tol);
Ra{end+1} = sprintf('sweep over nine differentials up to +-95 percent of the cruise thrust per motor: max error %.3e', ...
   v8.sweep_max_rel);
Ra{end+1} = sprintf('allocate.m differential authority at cruise: %.4f N*m, relative difference from the budget %.3f', ...
   v8.alloc_Nm, v8.tie_rel);
Ra{end+1} = '  the difference is real and explained: sizing_summary evaluates T = T_static*throttle and';
Ra{end+1} = '  therefore counts the low motor at 0 N, while plant/throttle_to_thrust subtracts the ram';
Ra{end+1} = '  drag k_ram*V and clips at zero, so the high motor loses that term and the low one cannot';
Ra{end+1} = '  go negative. The allocator uses the plant law, which is the conservative one.';
Ra{end+1} = sprintf('rudder authority at cruise %.4f N*m', N_rud);
Ra{end+1} = sprintf('ratio differential thrust / rudder = %.4f (with the structural cap %.4f)', ...
   v8.ratio_to_rudder, v8.ratio_at_cap);
Ra{end+1} = sprintf('the same ratio as the allocator computes it: %.4f', v8.ratio_to_rudder_alloc);
Ra{end+1} = sprintf('allocator yaw path check, M_cmd = %.2f N*m: dr %.6f rad, dth_l %.6f, dth_r %.6f, achieved N %.4f N*m', ...
   M_yaw_big(3), dr_y, dthl_y, dthr_y, ia_y.M_ach(3));
Ra{end+1} = sprintf('achieved N against y_motor*dT_ach: relative difference %.3e', v8.forced_tie_rel);
Ra{end+1} = sprintf('V8 verdict: %d', v8.ok);

fid = fopen(fullfile(resdir, 'control_lqr_report.txt'), 'w');
fprintf(fid, '%s\n', Rl{:});
fclose(fid);
fid = fopen(fullfile(resdir, 'control_pid_report.txt'), 'w');
fprintf(fid, '%s\n', Rp{:});
fclose(fid);
fid = fopen(fullfile(resdir, 'control_allocation_report.txt'), 'w');
fprintf(fid, '%s\n', Ra{:});
fclose(fid);
fprintf('%s\n', Rl{:});
fprintf('%s\n', Rp{:});
fprintf('%s\n', Ra{:});
fids = fopen(status_file, 'w');
fprintf(fids, 'report stage OK %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fids, 'parameter set rev %g, P %s\n', p_rev, P.meta.version);
fprintf(fids, 'control_lqr_report.txt, control_pid_report.txt, control_allocation_report.txt rewritten\n');
fclose(fids);
catch MErep
   report_ok = false;
   fidr = fopen(fullfile(resdir, 'control_design_status.txt'), 'w');
   fprintf(fidr, 'report stage failed: %s | %s\n', MErep.identifier, MErep.message);
   for kk = 1:numel(MErep.stack)
      fprintf(fidr, '  at %s line %d\n', MErep.stack(kk).name, MErep.stack(kk).line);
   end
   fprintf(fidr, 'checks struct was still saved to gains_baseline.mat\n');
   fclose(fidr);
   fprintf('REPORT STAGE FAILED: %s | %s\n', MErep.identifier, MErep.message);
   for kk = 1:numel(MErep.stack)
      fprintf('   at %s line %d\n', MErep.stack(kk).name, MErep.stack(kk).line);
   end
end

checks = struct('v5', v5, 'v6', v6, 'v7', v7, 'v8', v8);
created = datestr(now, 'yyyy-mm-dd HH:MM:SS');
save(fullfile(resdir, 'gains_baseline.mat'), 'gains_lqr', 'gains_pid', 'checks', ...
   'trim', 'S', 'created', 'lqr_alt_theta', 'lqr_alt_beta');
fprintf('gains_baseline.mat written\n');
fprintf('CONTROL DESIGN SUMMARY: V5 %d | V6 %d | V7 %d | V8 %d\n', v5.ok, v6.ok, v7.ok, v8.ok);
