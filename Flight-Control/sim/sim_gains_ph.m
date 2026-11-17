function gains = sim_gains_ph(P, lin, kind, trim)
% SIM_GAINS_PH Fallback PID or LQR gains, used only when control/ is absent
%
%   gains = sim_gains_ph(P, lin, kind, trim)   kind is 'pid' or 'lqr'

switch lower(kind)
   case 'pid'
      gains = local_pid(P, lin);
   case 'lqr'
      gains = local_lqr(P, lin);
   otherwise
      error('sim_gains_ph:Kind', 'unknown kind %s', kind);
end
gains.placeholder = true;
gains.trim_theta_deg = trim.theta_deg;
end

%% Local helpers
function gains = local_pid(P, lin)
lc = lin_contract(lin);
V0 = lc.x_op_long(1);
h = @(A, B, i, j) (abs(B(i, j)) > eps) * 0 + 0; %#ok<NASGU>
% static gain of each rate loop: rate per radian of surface, at the design point
Kp_to_p = lc.B_lat(2, 1);
Kr_to_r = lc.B_lat(3, 2);
Kq_to_q = lc.B_long(3, 1);
gains.p = local_rate(P.ctrl.bw.rate_p_rad_s, Kp_to_p, P);
gains.q = local_rate(P.ctrl.bw.rate_q_rad_s, Kq_to_q, P);
gains.r = local_rate(P.ctrl.bw.rate_r_rad_s, Kr_to_r, P);
gains.phi = local_att(P.ctrl.bw.att_phi_rad_s, P, P.ctrl.int_limit.phi);
gains.theta = local_att(P.ctrl.bw.att_theta_rad_s, P, P.ctrl.int_limit.theta);
gains.V = local_att(P.ctrl.bw.speed_rad_s, P, P.ctrl.int_limit.V);
gains.h = local_att(P.ctrl.bw.height_rad_s, P, P.ctrl.int_limit.h);
gains.x_op = lc.u_op_full;
gains.int_limit = P.ctrl.int_limit;
gains.rate_filter_tau_s = P.ctrl.rate_filter_tau_s;
gains.anti_windup_Kb = P.ctrl.anti_windup_Kb;
gains.law = struct('p', 'placeholder', 'q', 'placeholder', 'r', 'placeholder', ...
   'phi', 'placeholder', 'theta', 'placeholder', 'V', 'placeholder', 'h', 'placeholder');
gains.placeholder_reason = ['control/design_pid.m was not on the path, so the ' ...
   'rate loops use Kp = wc/K_plant with no D term'];
end

function g = local_rate(wc, K, P)
g = struct('name', 'rate', 'Kp', wc / max(K, eps), 'Ki', 0, 'Kd', 0, ...
   'Tf', P.ctrl.rate_filter_tau_s, 'Kb', P.ctrl.anti_windup_Kb, 'sign', 1, ...
   'wc_target_rad_s', wc);
end

function g = local_att(wc, P, ilim)
g = struct('name', 'att', 'Kp', wc, 'Ki', 0.2 * wc^2, 'Kd', 0, ...
   'Tf', P.ctrl.rate_filter_tau_s, 'Kb', P.ctrl.anti_windup_Kb, 'sign', 1, ...
   'wc_target_rad_s', wc, 'int_limit', ilim);
end

function gains = local_lqr(P, lin)
lc = lin_contract(lin);
Ql = blkdiag(diag(P.lqr.long.q_diag), P.lqr.long.q_diag(1));
Qa = blkdiag(diag(P.lqr.lat.q_diag), P.lqr.lat.q_diag(4));
Kl = lqr([lc.A_long, zeros(4, 1); [1 0 0 0], 0], [lc.B_long; 0 0], Ql, diag(P.lqr.long.r_diag));
Ka = lqr([lc.A_lat, zeros(4, 1); [0 0 0 1], 0], [lc.B_lat; 0 0], Qa, diag(P.lqr.lat.r_diag));
gains = struct();
gains.long = struct('K', Kl, 'A', lc.A_long, 'B', lc.B_long, 'C_int', [1 0 0 0], ...
   'x_op', lc.x_op_long, 'u_star_full', lc.u_op_full, 'u_idx_full', [2 6]);
gains.lat = struct('K', Ka, 'A', lc.A_lat, 'B', lc.B_lat, 'C_int', [0 0 0 1], ...
   'x_op', lc.x_op_lat, 'u_star_full', lc.u_op_full, 'u_idx_full', [1 3]);
gains.placeholder_reason = 'control/design_lqr.m was not on the path, gains solved here with lqr()';
end
