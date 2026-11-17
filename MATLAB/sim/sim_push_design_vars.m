function push = sim_push_design_vars(P, D)
% SIM_PUSH_DESIGN_VARS Put every design number the models read into the base workspace.
%
% The models hold no literal numbers: each block parameter is the name of a base
% workspace variable. Gains keep their contract names where a block can take an
% expression (gains.q.Kp), and everything that has to be pre-computed, such as
% the Tustin numerator of a filtered derivative or a 3x3 effectiveness matrix,
% is pushed here under a sim_ prefixed name. The mapping is
%
%   sim_pid_<loop>_Kp      gains.<loop>.Kp        proportional gain, signed
%   sim_pid_<loop>_Kp_abs  abs of the same         what the Gain block carries
%   sim_pid_<loop>_Ki      gains.<loop>.Ki        integral gain, signed
%   sim_pid_<loop>_Dnum    Tustin numerator of Kd*s/(Tf*s+1)
%   sim_pid_<loop>_Dnum_abs abs of the same
%   sim_pid_<loop>_Dden    Tustin denominator of the same
%   sim_pid_<loop>_pol     gains.<loop>.sign      -1 or +1, applied once after the sum
%   sim_pid_<loop>_Ts      P.sim.Ts_ctrl for p,q,r and P.sim.Ts_outer for phi,theta,V,h
%   sim_pid_<loop>_Isat    integrator saturation limit
%   sim_pid_h_lim          deg2rad(P.limits.pitch_max_deg)
%   sim_u_trim             trim.u_star, 6x1
%   sim_u_trim_surf        trim.u_star(1:3), rad
%   sim_T_trim             total trim thrust, N
%   sim_G_surf             qbar*S*diag([b cbar b]) times the surface derivatives, 3x3
%   sim_lqr_*              gains_lqr contract quantities and the linear
%                          reference map built from lqr_reference
%
% Pushing is idempotent; the return value lists what was written and is recorded
% in the build report.

loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
ts_of = struct('p', P.sim.Ts_ctrl, 'q', P.sim.Ts_ctrl, 'r', P.sim.Ts_ctrl, ...
   'phi', P.sim.Ts_outer, 'theta', P.sim.Ts_outer, 'V', P.sim.Ts_outer, ...
   'h', P.sim.Ts_outer);
surf_lim = struct('p', deg2rad(P.act.aileron.max_deg), ...
   'q', deg2rad(P.act.elevator.max_deg), 'r', deg2rad(P.act.rudder.max_deg));
gp = D.gains_pid;

written = {};
written = local_push(written, 'sim_ts_ctrl', P.sim.Ts_ctrl);
written = local_push(written, 'sim_ts_outer', P.sim.Ts_outer);
written = local_push(written, 'sim_ts_alloc', P.sim.Ts_alloc);
written = local_push(written, 'sim_ts_sensor', P.sim.Ts_sensor);
written = local_push(written, 'sim_t_end', P.sim.t_end_s);
written = local_push(written, 'sim_c_zero', 0);
written = local_push(written, 'sim_c_one', 1);
written = local_push(written, 'sim_c_neg1', -1);
written = local_push(written, 'sim_gains_pid', gp);
written = local_push(written, 'sim_gains_lqr', D.gains_lqr);
written = local_push(written, 'gains', gp);
% The trim and linear model also go into the base workspace under their plain
% names: sim_push_scenario needs trim.x_star for a default initial state, and
% the scenario scripts read trim.u_star for the feedforward quantities.
written = local_push(written, 'trim', D.trim);
written = local_push(written, 'lin', D.lin);

for k = 1:numel(loops)
   L = loops{k};
   g = gp.(L);
   c = sim_pid_coeffs(g, ts_of.(L));
   % Loop polarity is a designed quantity, not a constant: a plant with a right
   % half plane pole wants negative feedback through the whole controller, and
   % the pitch attitude loop is one of those. The model applies it once, with a
   % Gain block after the sum, so the three gain paths cannot disagree. The
   % magnitude of every gain is published beside it; sim_add_pid reads the _abs
   % names, so a sign error can only ever come from sim_pid_<loop>_pol.
   pol = sign(g.Kp);
   if pol == 0
      pol = 1;
   end
   written = local_push(written, sprintf('sim_pid_%s_pol', L), pol);
   written = local_push(written, sprintf('sim_pid_%s_Kp', L), c.Kp);
   written = local_push(written, sprintf('sim_pid_%s_Kp_abs', L), abs(c.Kp));
   % The Discrete-Time Integrator requires gainval STRICTLY POSITIVE and a positive
   % upper saturation limit, so zero is not acceptable even though Ki = 0 is a
   % legitimate design outcome (a pure PD loop, which the roll loop uses). The
   % integral gain is therefore floored at a negligible positive number so the block
   % accepts it while contributing nothing, and the same floor is applied to Isat.
   ki_floor = 1e-12;
   written = local_push(written, sprintf('sim_pid_%s_Ki_abs', L), max(abs(c.Ki), ki_floor));
   written = local_push(written, sprintf('sim_pid_%s_Ki_sign', L), sign(c.Ki) + (c.Ki == 0));
   written = local_push(written, sprintf('sim_pid_%s_Ki', L), c.Ki);
   written = local_push(written, sprintf('sim_pid_%s_Dnum', L), c.Dnum);
   written = local_push(written, sprintf('sim_pid_%s_Dnum_abs', L), abs(c.Dnum));
   written = local_push(written, sprintf('sim_pid_%s_Dden', L), c.Dden);
   written = local_push(written, sprintf('sim_pid_%s_Ts', L), c.Ts);
   % The integral-state clamp is read from the one authoritative field, the same one
   % control/pid_cascade_sim.m reads. Before rev 11 this pushed gains.<loop>.int_limit,
   % which is an ERROR-domain bound and not a state bound, so the model flew with a
   % clamp 2.5x the design report's on theta and 0.8x on V. The three rate loops have
   % no integral action and fall through to their surface travel.
   if isfield(P.ctrl.int_state_limit, L)
      ilim = P.ctrl.int_state_limit.(L);
   elseif isfield(surf_lim, L)
      ilim = surf_lim.(L);
   else
      ilim = P.ctrl.int_state_limit.V;
   end
   ilim = max(abs(ilim), ki_floor);
   written = local_push(written, sprintf('sim_pid_%s_Isat', L), ilim);
   written = local_push(written, sprintf('sim_pid_%s_Isat_neg', L), -ilim);
end
written = local_push(written, 'sim_pid_h_lim', deg2rad(P.limits.pitch_max_deg));
written = local_push(written, 'sim_pid_h_lim_neg', -deg2rad(P.limits.pitch_max_deg));

% Rate command clamps for the attitude loops. Published from P so the model limit and
% the parameter that documents it cannot drift apart; the values are derived in
% uacs_twin_params from the full-surface steady rate at cruise, not tuned by hand.
written = local_push(written, 'sim_rate_cmd_lim_p', P.ctrl.rate_cmd_limit.p_rad_s);
written = local_push(written, 'sim_rate_cmd_lim_q', P.ctrl.rate_cmd_limit.q_rad_s);
written = local_push(written, 'sim_rate_cmd_lim_r', P.ctrl.rate_cmd_limit.r_rad_s);

% ---------------- trim, effectiveness, allocation feedforward ----------------
u_star = D.trim.u_star(:);
V_a = P.op.cruise.V_mps;
qbar_a = 0.5 * P.env.rho_sl * V_a^2;
Sb = qbar_a * P.geom.S * P.geom.b;
Sc = qbar_a * P.geom.S * P.geom.cbar;
G_surf = [Sb * P.aero.Cl_da, 0, Sb * P.aero.Cl_dr; ...
          0, Sc * P.aero.Cm_de, 0; ...
          Sb * P.aero.Cn_da, 0, Sb * P.aero.Cn_dr];
T_trim = sim_thrust(P, u_star(4), V_a) + sim_thrust(P, u_star(5), V_a) + u_star(6);
written = local_push(written, 'sim_u_trim', u_star);
written = local_push(written, 'sim_u_trim_surf', u_star(1:3));
written = local_push(written, 'sim_T_trim', T_trim);
written = local_push(written, 'sim_G_surf', G_surf);
written = local_push(written, 'sim_qbar_trim', qbar_a);

% ---------------- LQR quantities ----------------
gl = D.gains_lqr;
written = local_push(written, 'sim_lqr_K_long', gl.long.K);
written = local_push(written, 'sim_lqr_K_lat', gl.lat.K);
written = local_push(written, 'sim_lqr_C_long', gl.long.C_int);
written = local_push(written, 'sim_lqr_C_lat', gl.lat.C_int);
written = local_push(written, 'sim_lqr_xop_long', gl.long.x_op);
written = local_push(written, 'sim_lqr_xop_lat', gl.lat.x_op);
written = local_push(written, 'sim_lqr_ustar', gl.long.u_star_full(:));
written = local_push(written, 'sim_lqr_zlim_long', P.ctrl.int_limit.V);
written = local_push(written, 'sim_lqr_zlim_long_neg', -P.ctrl.int_limit.V);
written = local_push(written, 'sim_lqr_zlim_lat', P.ctrl.int_limit.phi);
written = local_push(written, 'sim_lqr_zlim_lat_neg', -P.ctrl.int_limit.phi);
written = local_push(written, 'sim_lqr_z0', 0);
refmap = sim_lqr_refmap(P, D.lin, gl);
written = local_push(written, 'sim_lqr_Tref_long', refmap.long.Tref);
written = local_push(written, 'sim_lqr_Uff_long', refmap.long.Uff);
written = local_push(written, 'sim_lqr_Tref_lat', refmap.lat.Tref);
written = local_push(written, 'sim_lqr_Uff_lat', refmap.lat.Uff);

push = struct();
push.written = written;
push.count = numel(written);
push.refmap = refmap;
push.gains_source = 'deliverable';
if isfield(gp, 'placeholder') && gp.placeholder
   push.gains_source = 'placeholder';
end
end

%% ---------------- local ----------------
function w = local_push(w, name, value)
assignin('base', name, value);
w{end + 1} = name;
end
