function S = sizing_summary(P)
% SIZING_SUMMARY Preliminary sizing and authority budget for the twin UAV.
%
%   S = sizing_summary(P);
%
% Pure computation from P, returning the derived quantities as a struct so the report
% and the control scripts quote the same numbers instead of re-deriving them.

rho = P.env.rho_sl;
g   = P.env.g;
m   = P.mass.m;
W   = m * g;
b   = P.geom.b;
Sref= P.geom.S;
cbar= P.geom.cbar;
AR  = P.geom.AR;

%% Wing loading and stall
S.wing_loading_Npm2 = W / Sref;
S.wing_loading_kgpm2 = m / Sref;
S.V_stall = sqrt(2 * W / (rho * Sref * P.aero.CL_max));
S.q_stall = 0.5 * rho * S.V_stall^2;

%% The operating points that matter
names = {'cruise', 'climb', 'slow', 'bank'};
for k = 1:numel(names)
    op = P.op.(names{k});
    V  = op.V_mps;
    h  = op.h_m;
    gam = deg2rad(op.gamma_deg);
    phi = deg2rad(op.bank_deg);

    q = 0.5 * rho * V^2;
    % Load factor from the bank angle
    n = 1 / cos(phi);
    CL = n * W / (q * Sref);
    CD = P.aero.CD0 + P.aero.k_induced * CL^2;
    D  = q * Sref * CD;
    L  = q * Sref * CL;

    % Thrust along the flight path must cover drag and the climb component
    T_req = D + W * sin(gam);
    P_shaft = T_req * V;
    P_elec  = P_shaft / P.prop.power.eta_total_cruise;

    S.op(k).name = names{k};
    S.op(k).V = V;
    S.op(k).h = h;
    S.op(k).q = q;
    S.op(k).n = n;
    S.op(k).CL = CL;
    S.op(k).CD = CD;
    S.op(k).L_over_D = CL / CD;
    S.op(k).drag_N = D;
    S.op(k).lift_N = L;
    S.op(k).thrust_req_N = T_req;
    S.op(k).P_shaft_W = P_shaft;
    S.op(k).P_elec_W = P_elec;
    S.op(k).P_per_motor_W = P_elec / P.prop.n_motors;
    S.op(k).V_stall_margin = V / S.V_stall;

    % CL must stay under CL_max with margin
    S.op(k).CL_utilisation = CL / P.aero.CL_max;
    % Reynolds number on the mean chord, for airfoil selection
    S.op(k).Re = rho * V * cbar / P.env.mu_sl;
end

%% Endurance and energy at cruise
c = S.op(1);
S.endurance.target_s = P.prop.endurance_target_s;
E_Wh = c.P_elec_W * S.endurance.target_s / 3600;
S.endurance.cruise_P_elec_W = c.P_elec_W;
S.endurance.energy_Wh = E_Wh;
% Pack sized at 80 percent usable depth of discharge
S.endurance.pack_Wh = E_Wh / 0.80;
% At a pack-level specific energy of 200 Wh/kg this is the mass penalty
S.endurance.pack_mass_kg_at_200Whpkg = S.endurance.pack_Wh / 200.0;
S.endurance.pack_mass_fraction = S.endurance.pack_mass_kg_at_200Whpkg / m;

%% Propulsion checks
% Static thrust is derived in the parameter file from peak shaft power and the figure of
% merit, so here it is re-expressed only, never re-chosen
S.prop.T_static_max = P.prop.T_static_max;
S.prop.n_static_rps = P.prop.rpm_static / 60;
S.prop.n_static_rpm = P.prop.rpm_static;
S.prop.tip_mach_static = (pi * P.prop.D * S.prop.n_static_rps) / 340.0;
S.prop.thrust_to_weight_static = P.prop.n_motors * P.prop.T_static_max / W;
% Thrust actually needed at cruise, per motor
T_cruise_per_motor = c.thrust_req_N / P.prop.n_motors;
S.prop.thrust_cruise_per_motor_N = T_cruise_per_motor;
% The cruise throttle is the inverse of the same momentum-theory map the plant uses,
% solved by bisection so there is exactly one propulsion model
S.prop.throttle_cruise = local_throttle_for_thrust(P, T_cruise_per_motor, c.V);
S.prop.throttle_margin_to_full = 1 - S.prop.throttle_cruise;
% Available thrust at this speed, full throttle, both motors
S.prop.thrust_available_at_cruise_N = P.prop.n_motors * ...
    prop_thrust_from_throttle(P, 1.0, c.V);
S.prop.throttle_reserve_ratio = S.prop.thrust_available_at_cruise_N / c.thrust_req_N;
S.prop.power_loading_W_per_kg = (P.prop.power.peak_per_motor_W * P.prop.n_motors) / m;
% Electrical power actually drawn at the cruise setting
S.prop.P_shaft_cruise_per_motor_W = S.prop.throttle_cruise^P.prop.throttle_exponent * ...
    P.prop.P_shaft_peak;
S.prop.P_elec_cruise_W = 2 * S.prop.P_shaft_cruise_per_motor_W / ...
    (P.prop.power.eta_motor * P.prop.power.eta_esc);

%% Yaw and roll authority budget
q_c = c.q;
% Rudder at full deflection
S.authority.rudder.yaw_moment_Nm = q_c * Sref * b * abs(P.aero.Cn_dr) * ...
    deg2rad(P.act.rudder.max_deg);
% Aileron at full deflection
S.authority.aileron.roll_moment_Nm = q_c * Sref * b * P.aero.Cl_da * ...
    deg2rad(P.act.aileron.max_deg);
% Elevator at full deflection
S.authority.elevator.pitch_moment_Nm = q_c * Sref * cbar * abs(P.aero.Cm_de) * ...
    deg2rad(P.act.elevator.max_deg);

% Differential thrust: symmetric modulation of the total throttle by delta, holding
% total thrust roughly constant. The yaw moment is (T_L - T_R) * y_motor.
% The usable delta is NOT the structural cap alone: it is also limited by the throttle
% headroom at this operating point, because the low-throttle motor cannot go below idle
% and the total thrust must not change
d_cap = P.prop.differential_limit;
d_headroom = min(S.prop.throttle_cruise, 1 - S.prop.throttle_cruise);
d_eff = min(d_cap, d_headroom);
S.authority.diff_thrust.d_cap = d_cap;
S.authority.diff_thrust.d_headroom = d_headroom;
S.authority.diff_thrust.d_effective = d_eff;
if d_eff == d_cap
    S.authority.diff_thrust.limiting_factor = 'structural cap';
else
    S.authority.diff_thrust.limiting_factor = 'throttle headroom';
end

T_L = P.prop.T_static_max * (S.prop.throttle_cruise + d_eff);
T_R = P.prop.T_static_max * (S.prop.throttle_cruise - d_eff);
S.authority.diff_thrust.yaw_moment_Nm = abs(T_L - T_R) * P.prop.y_motor;
S.authority.diff_thrust.thrust_left_N = T_L;
S.authority.diff_thrust.thrust_right_N = T_R;
S.authority.diff_thrust.total_thrust_N = T_L + T_R;
S.authority.diff_thrust.ratio_to_rudder = ...
    S.authority.diff_thrust.yaw_moment_Nm / S.authority.rudder.yaw_moment_Nm;
% The same modulation if the full structural cap were reachable
S.authority.diff_thrust.yaw_moment_at_cap_Nm = ...
    2 * d_cap * P.prop.T_static_max * P.prop.y_motor;
S.authority.diff_thrust.ratio_to_rudder_at_cap = ...
    S.authority.diff_thrust.yaw_moment_at_cap_Nm / S.authority.rudder.yaw_moment_Nm;

% Engine-out asymmetric thrust at peak power, the case that sizes the fin
T_peak_static = P.prop.T_static_max;                 % both engines at full power
T_asym = T_peak_static;                              % one engine lost
S.authority.engine_out.yaw_moment_Nm = T_asym * P.prop.y_motor;
S.authority.engine_out.ratio_to_rudder = ...
    S.authority.engine_out.yaw_moment_Nm / S.authority.rudder.yaw_moment_Nm;
% Thrust line offset above the CG produces a nose-down moment
S.authority.engine_out.pitch_offset_m = P.prop.z_thrust_offset;
S.authority.engine_out.pitch_moment_Nm = T_asym * abs(P.prop.z_thrust_offset);
S.authority.engine_out.pitch_ratio_to_elevator = ...
    S.authority.engine_out.pitch_moment_Nm / S.authority.elevator.pitch_moment_Nm;

% Minimum control speed: the speed at which full rudder just balances the engine-out yaw
% moment. The aircraft is not controllable on one engine below it, which the failsafe
% must respect
Cn_dr_max = abs(P.aero.Cn_dr) * deg2rad(P.act.rudder.max_deg);
S.authority.engine_out.V_mc = sqrt(2 * S.authority.engine_out.yaw_moment_Nm / ...
    (rho * Sref * b * Cn_dr_max));
S.authority.engine_out.V_mc_vs_cruise = ...
    S.authority.engine_out.V_mc / P.op.cruise.V_mps;
S.authority.engine_out.V_mc_vs_stall = ...
    S.authority.engine_out.V_mc / S.V_stall;
% Fraction of the usable speed range at which the rudder can just hold the aircraft on
% one engine at full power. Below it the only safe action is to reduce the failed-engine
% side throttle, not to fight the yaw with rudder
S.authority.engine_out.controllable_above_fraction = ...
    S.authority.engine_out.V_mc / P.limits.V_max_mps;

%% Longitudinal static margin
% Stick-fixed neutral point from the tail volume, both derivatives per radian:
%   x_np - x_ref = -Cm_alpha / CL_alpha * cbar
S.stability.x_np_minus_ref_m = -P.aero.Cm_alpha / P.aero.CL_alpha * cbar;
S.stability.static_margin_frac_cbar = ...
    (S.stability.x_np_minus_ref_m - (P.mass.x_cg - P.geom.x_ac)) / cbar;
S.stability.Cm_alpha = P.aero.Cm_alpha;
S.stability.CL_alpha = P.aero.CL_alpha;

%% Natural frequencies implied by the derivatives
% Short-period frequency, tail-dominated approximation
Iyy = P.mass.Iyy_cg;
S.modes.short_period_approx_rad_s = sqrt(q_c * Sref * cbar * ...
    (-P.aero.Cm_alpha) / Iyy);
% Roll subsidence time constant from roll damping
S.modes.roll_subsidence_tau_s = P.mass.Ixx_cg / ...
    (q_c * Sref * b * (b / (2 * c.V)) * abs(P.aero.Cl_p));
% Dutch roll frequency, weathercock stiffness against yaw inertia
S.modes.dutch_roll_approx_rad_s = sqrt(q_c * Sref * b * P.aero.Cn_beta / P.mass.Izz_cg);
end

%% Local helpers
function th = local_throttle_for_thrust(P, T_target, V)
% LOCAL_THROTTLE_FOR_THRUST Invert the propulsion map for the throttle setting
% Thrust is monotone increasing in throttle, so bisection is safe and needs no starting
% guess; inverting the same function the plant uses keeps one model
if T_target <= 0
    th = 0;
    return;
end
lo = 0;
hi = 1;
if prop_thrust_from_throttle(P, hi, V) < T_target
    % The target exceeds full throttle at this speed; report full throttle rather than a
    % number above one, and let the caller notice the shortfall
    th = 1;
    return;
end
for iter = 1:200
    mid = 0.5 * (lo + hi);
    if prop_thrust_from_throttle(P, mid, V) < T_target
        lo = mid;
    else
        hi = mid;
    end
end
th = 0.5 * (lo + hi);
end
