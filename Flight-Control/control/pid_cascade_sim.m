function log = pid_cascade_sim(P, gains, lc, opts)
% PID_CASCADE_SIM Time-domain check of the PID cascade on the linear model.
%
%   log = pid_cascade_sim(P, gains, lc, opts)
%
%   lc    contract-order linear model from lin_contract.m
%   opts  t_end_s, t_step_s, V_step, h_step, theta_step, phi_step, r_step,
%         mode ('attitude' or 'altitude'), anti_windup (logical),
%         sat_enable (logical)
%
% Plant: the four longitudinal and four lateral states plus one first-order
% actuator lag per effector, so the loop simulated here is the loop the margins
% were measured on. The controller runs at P.sim.Ts_ctrl with a zero-order hold
% and the plant integrates at Ts_ctrl/4.
%
% Saturation is applied in absolute actuator units: elevator and aileron and
% rudder against P.act.*.max_deg, total thrust against the two ESC windows
% (0..2*T_static_max). Anti-windup is back-calculation: the actuator saturation
% error is mapped back through the downstream loop gain into each integrator,
% with gain P.ctrl.anti_windup_Kb, and each integral state is capped at
% int_limit/loop bandwidth, one loop time constant of persistent error at the
% limit value.

if nargin < 4
   opts = struct();
end
d = struct('t_end_s', 8.0, 't_step_s', 0.5, 'V_step', 3.0, 'h_step', 20.0, ...
   'theta_step', 0, 'phi_step', deg2rad(20), 'r_step', 0, 'mode', 'attitude', ...
   'anti_windup', true, 'sat_enable', true, 'int_limit_scale', 1.0);
f = fieldnames(d);
for k = 1:numel(f)
   if ~isfield(opts, f{k}) || isempty(opts.(f{k}))
      opts.(f{k}) = d.(f{k});
   end
end

Ts = P.sim.Ts_ctrl;
n_sub = 4;
dt = Ts / n_sub;
n = round(opts.t_end_s / Ts);
t = (0:n)' * Ts;

%% Trim and limits
x_op = lc.x_op_long;
V0 = x_op(1);
u_star = lc.u_op_full;
de_trim = u_star(2);
de_lim = deg2rad(P.act.elevator.max_deg);
da_lim = deg2rad(P.act.aileron.max_deg);
dr_lim = deg2rad(P.act.rudder.max_deg);
T_trim = 2 * prop_thrust_from_throttle(P, u_star(4), V0);
T_min = 0;
T_max = 2 * prop_thrust_from_throttle(P, 1.0, V0);

% Rate-command guards on the attitude outer loops, read from the same field the
% built model is pushed from, so this simulation checks the loop the model
% actually flies; V13 asserts the two agree
q_lim = P.ctrl.rate_cmd_limit.q_rad_s;
p_lim = P.ctrl.rate_cmd_limit.p_rad_s;
r_lim = P.ctrl.rate_cmd_limit.r_rad_s;
theta_lim = deg2rad(P.limits.pitch_max_deg);

%% Augmented linear plants
% Actuator states are deviations from trim, one first-order lag per effector
tau_de = P.act.elevator.tau_s;
tau_da = P.act.aileron.tau_s;
tau_dr = P.act.rudder.tau_s;
tau_T = P.prop.esc.tau_s;
Al = [lc.A_long, lc.B_long; zeros(2, 4), diag([-1 / tau_de, -1 / tau_T])];
Bl = [zeros(4, 2); diag([1 / tau_de, 1 / tau_T])];
At = [lc.A_lat, lc.B_lat; zeros(2, 4), diag([-1 / tau_da, -1 / tau_dr])];
Bt = [zeros(4, 2); diag([1 / tau_da, 1 / tau_dr])];

%% State
xl = zeros(6, 1);
xt = zeros(6, 1);
h = P.op.cruise.h_m;
i_th = 0; i_V = 0; i_h = 0; i_phi = 0;
xf_q = 0; xf_p = 0; xf_r = 0;
% The integral-state clamps, read from the one authoritative field so this
% simulation and the built model cannot invent different ones; int_limit_scale
% exists only so a diagnostic can move the clamp, and 1.0 is the default
lim_ith = opts.int_limit_scale * P.ctrl.int_state_limit.theta;
lim_iV = opts.int_limit_scale * P.ctrl.int_state_limit.V;
lim_ih = opts.int_limit_scale * P.ctrl.int_state_limit.h;
lim_iphi = opts.int_limit_scale * P.ctrl.int_state_limit.phi;
Kb = gains.anti_windup_Kb * double(opts.anti_windup);
Kaw_th = 1 / (gains.q.Kp * gains.theta.Kp);
Kaw_h = 1 / (gains.q.Kp * gains.h.Kp);
Kaw_phi = 1 / (gains.p.Kp * gains.phi.Kp);
Kaw_V = 1 / gains.V.Kp;

L = struct();
L.t = t;
L.V = zeros(n + 1, 1); L.alpha = zeros(n + 1, 1); L.q = zeros(n + 1, 1);
L.theta = zeros(n + 1, 1); L.h = zeros(n + 1, 1);
L.de = zeros(n + 1, 1); L.de_cmd = zeros(n + 1, 1);
L.T = zeros(n + 1, 1); L.T_cmd = zeros(n + 1, 1);
L.phi = zeros(n + 1, 1); L.p = zeros(n + 1, 1); L.r = zeros(n + 1, 1);
L.beta = zeros(n + 1, 1); L.da = zeros(n + 1, 1); L.dr = zeros(n + 1, 1);
L.i_h = zeros(n + 1, 1); L.i_th = zeros(n + 1, 1); L.i_V = zeros(n + 1, 1);
L.i_phi = zeros(n + 1, 1);
L.de_sat = false(n + 1, 1); L.T_sat = false(n + 1, 1);
L.theta_c = zeros(n + 1, 1); L.V_c = zeros(n + 1, 1); L.phi_c = zeros(n + 1, 1);
L.p_c = zeros(n + 1, 1); L.q_c = zeros(n + 1, 1); L.r_c = zeros(n + 1, 1);

for k = 1:n + 1
   tk = t(k);
   step_on = tk >= opts.t_step_s;
   if strcmp(opts.mode, 'altitude')
      theta_c_direct = 0;
      h_c = P.op.cruise.h_m + step_on * opts.h_step;
   else
      % Clamp the commanded pitch attitude to the envelope, exactly as the
      % altitude branch below does, so a caller that passes theta_step in the
      % wrong units cannot ask for an impossible attitude
      theta_c_direct = min(max(step_on * opts.theta_step, -theta_lim), theta_lim);
      h_c = P.op.cruise.h_m;
   end
   V_c = V0 + step_on * opts.V_step;
   phi_c = step_on * opts.phi_step;
   % Clamped to the same yaw rate-command field the model uses; the yaw channel
   % has no outer loop, so its command is clamped directly
   r_c = min(max(step_on * opts.r_step, -r_lim), r_lim);

   %% Longitudinal cascade
   if strcmp(opts.mode, 'altitude')
      theta_c_raw = gains.h.Kp * (h_c - h) + gains.h.Ki * i_h;
      theta_c = min(max(theta_c_raw, -theta_lim), theta_lim);
   else
      theta_c_raw = theta_c_direct; theta_c = theta_c_direct;
   end
   q_c_raw = gains.theta.Kp * (theta_c - xl(4)) + gains.theta.Ki * i_th;
   q_c = min(max(q_c_raw, -q_lim), q_lim);
   exf_q = q_c - xl(3);
   de_cmd = de_trim + gains.q.Kp * exf_q + gains.q.Kd * (exf_q - xf_q) / gains.q.Tf;
   de_sat = min(max(de_cmd, -de_lim), de_lim);
   T_cmd_raw = T_trim + gains.V.Kp * (V_c - (V0 + xl(1))) + gains.V.Ki * i_V;
   T_cmd = min(max(T_cmd_raw, T_min), T_max);
   if ~opts.sat_enable
      de_sat = de_cmd; T_cmd = T_cmd_raw;
   end

   %% Lateral cascade
   p_c_raw = gains.phi.Kp * (phi_c - xt(4)) + gains.phi.Ki * i_phi;
   p_c = min(max(p_c_raw, -p_lim), p_lim);
   exf_p = p_c - xt(2);
   da_cmd = gains.p.Kp * exf_p + gains.p.Kd * (exf_p - xf_p) / gains.p.Tf;
   da_sat = min(max(da_cmd, -da_lim), da_lim);
   exf_r = r_c - xt(3);
   dr_cmd = gains.r.Kp * exf_r + gains.r.Kd * (exf_r - xf_r) / gains.r.Tf;
   dr_sat = min(max(dr_cmd, -dr_lim), dr_lim);
   if ~opts.sat_enable
      da_sat = da_cmd; dr_sat = dr_cmd;
   end

   %% Log
   L.V(k) = V0 + xl(1); L.alpha(k) = x_op(2) + xl(2); L.q(k) = xl(3);
   L.theta(k) = x_op(4) + xl(4); L.h(k) = h;
   L.de(k) = de_trim + xl(5); L.de_cmd(k) = de_cmd;
   L.T(k) = T_trim + xl(6); L.T_cmd(k) = T_cmd_raw;
   L.phi(k) = xt(4); L.p(k) = xt(2); L.r(k) = xt(3); L.beta(k) = xt(1);
   L.da(k) = xt(5); L.dr(k) = xt(6);
   L.i_h(k) = i_h; L.i_th(k) = i_th; L.i_V(k) = i_V; L.i_phi(k) = i_phi;
   L.de_sat(k) = abs(de_cmd) > de_lim; L.T_sat(k) = (T_cmd_raw < T_min) || (T_cmd_raw > T_max);
   L.theta_c(k) = theta_c; L.V_c(k) = V_c; L.phi_c(k) = phi_c;
   L.p_c(k) = p_c; L.q_c(k) = q_c; L.r_c(k) = r_c;

   if k > n
      break;
   end

   %% Derivative filters
   xf_q = xf_q + (Ts / gains.q.Tf) * (exf_q - xf_q);
   xf_p = xf_p + (Ts / gains.p.Tf) * (exf_p - xf_p);
   xf_r = xf_r + (Ts / gains.r.Tf) * (exf_r - xf_r);

   %% Integrators, back-calculation, limits
   % Each integrator sees its own error plus Kb times the equivalent of the
   % actuator saturation error referred to that loop's output; the downstream
   % factors are signed loop gains, so the correction opposes the error that
   % caused the saturation
   e_th = theta_c - xl(4);
   i_th = i_th + Ts * (e_th + Kb * Kaw_th * (de_sat - de_cmd));
   i_th = min(max(i_th, -lim_ith), lim_ith);
   e_V = V_c - (V0 + xl(1));
   i_V = i_V + Ts * (e_V + Kb * Kaw_V * (T_cmd - T_cmd_raw));
   i_V = min(max(i_V, -lim_iV), lim_iV);
   e_h = h_c - h;
   if strcmp(opts.mode, 'altitude')
      i_h = i_h + Ts * (e_h + Kb * Kaw_h * (theta_c - theta_c_raw));
      i_h = min(max(i_h, -lim_ih), lim_ih);
   end
   e_phi = phi_c - xt(4);
   i_phi = i_phi + Ts * (e_phi + Kb * Kaw_phi * (da_sat - da_cmd));
   i_phi = min(max(i_phi, -lim_iphi), lim_iphi);

   %% Plant, zero-order hold over the control period
   ul = [de_sat - de_trim; T_cmd - T_trim];   % deviations fed to the linear model
   ut = [da_sat; dr_sat];
   for s = 1:n_sub
      xl = xl + dt * (Al * xl + Bl * ul);
      xt = xt + dt * (At * xt + Bt * ut);
   end
   h = h + Ts * (V0 * (xl(4) - xl(2)) + xl(1) * (x_op(4) - x_op(2)));
end
log = L;
% The limits this run actually applied are recorded, so a consistency check can
% compare what this simulation used against what the model was pushed
log.meta = struct('opts', opts, 'Ts', Ts, 'V0', V0, 'de_trim', de_trim, ...
   'T_trim', T_trim, 'q_lim', q_lim, 'p_lim', p_lim, 'r_lim', r_lim, ...
   'lim_ith', lim_ith, 'lim_iV', lim_iV, 'lim_ih', lim_ih, 'lim_iphi', lim_iphi);
end

%% Local helpers
function T = local_thrust(P, V, dth)
% Superseded by the authoritative map; kept only so an accidental call fails
% loudly instead of silently using the retired ram-drag law
error('pid_cascade_sim:RetiredLaw', 'use prop_thrust_from_throttle(P, dth, V)');
end
