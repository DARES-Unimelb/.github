function P = uacs_twin_params()
% UACS_TWIN_PARAMS Baseline parameter set for the twin-engine 20 kg UAV.
%
% Returns a single struct P. Every script in this project derives its numbers
% from this struct, so there is exactly one authoritative definition of mass,
% geometry, aerodynamics, propulsion, actuators and the design operating point.
%
% Usage:
%   P = uacs_twin_params();          % local struct
%   init_uacs_twin;                  % push P (and short aliases) into base
%
% Conventions
%   Axes      body frame: x forward, y right, z down (standard aviation NED).
%   Angles    rad.
%   Position  m.  Attitude rad.  Rates rad/s.  Velocity m/s.
%   Origin    taken at the wing aerodynamic centre, on the aircraft centreline.
%             x_cg is negative because the centre of gravity sits ahead of it.
%
% Source tagging used in the tables below
%   MEASURED  taken from a datasheet or a published model.
%   SIZED     computed in this file from the density / geometry above, so it is
%             self-consistent by construction; change an input and it moves.
%   ESTIMATE  engineering estimate for this class of airframe, to be replaced by
%             wind-tunnel, CFD or flight-test identification.
%
% Each entry that is not MEASURED is also flagged in P.meta.estimated_fields.

%% Environment
env.rho_sl = 1.225;      % kg/m^3, ISA sea level
env.g = 9.80665;         % m/s^2
env.mu_sl = 1.789e-5;    % Pa*s, for Reynolds number checks
% ISA constants, used by plant/atmo_isa.m; R = 287.0531 is the value the
% Aerospace Blockset ISA block uses, so the two agree bit for bit
env.T_sl = 288.15;       % K, ISA sea level temperature
env.p_sl = 101325.0;     % Pa, ISA sea level pressure
env.L_isa = 0.0065;      % K/m, troposphere lapse rate, temperature falls with h
env.R_air = 287.0531;    % J/(kg*K), specific gas constant of dry air
env.gamma_air = 1.4;     % ratio of specific heats, for the speed of sound
env.h_trop_m = 11000.0;  % m, top of the modelled troposphere; above it atmo_isa errors

%% Mass and inertia
% xyz positions are relative to the reference origin defined above.
mass.m = 17.0;           % kg, take-off mass. Team requirement.
mass.payload_kg = 12.0;  % kg, fixed payload. Team requirement.
mass.airframe_budget_kg = 5.0;   % kg, all-up minus payload. Team requirement.
% The budget is tight: the payload is 71 percent of take-off mass and the battery
% alone takes 2.10 kg for the endurance target, so the installed peak power is
% modest and the self-check asserts the table sums to mass.m and that the
% non-payload part equals the budget
mass.items = {
%   name                     m[kg]   x[m]    y[m]    z[m]
    'wing'                    1.10   -0.05   0.00    0.00
    'fuselage_boom'           0.55    0.00   0.00    0.00
    'payload'                12.00   -0.22   0.00    0.02
    'battery'                 2.10   -0.15   0.00    0.03
    'tail'                    0.20    1.95   0.00    0.08
    'motor_esc_left'          0.17   -0.05   0.90   -0.10
    'motor_esc_right'         0.17   -0.05  -0.90   -0.10
    'prop_left'               0.03   -0.05   0.90   -0.10
    'prop_right'              0.03   -0.05  -0.90   -0.10
    'landing_gear'            0.20   -0.10   0.00    0.18
    'servos_links'            0.20   -0.10   0.00    0.05
    'margin'                  0.25    0.00   0.00    0.00
};

% Point-mass inertia built from mass.items, then scaled by a distribution factor
% that accounts for mass spread away from each point. SIZED.
mass = uacs_apply_override('mass', mass);
[mass.Ixx, mass.Iyy, mass.Izz, mass.Ixz, mass.dist_scale] = local_inertia(mass.items);

% Inertia about the actual centre of gravity (parallel axis from the origin)
[mass.x_cg, mass.y_cg, mass.z_cg] = local_cg(mass.items);
mass.Ixx_cg = mass.Ixx - mass.m * (mass.y_cg^2 + mass.z_cg^2);
mass.Iyy_cg = mass.Iyy - mass.m * (mass.x_cg^2 + mass.z_cg^2);
mass.Izz_cg = mass.Izz - mass.m * (mass.x_cg^2 + mass.y_cg^2);
mass.Ixz_cg = mass.Ixz + mass.m * mass.x_cg * mass.z_cg;

%% Design operating point
% This block must stay above the control-surface, geometry and aerodynamics
% sections, because the tail sizing and the zero-lift pitching moment are derived
% from the cruise trim condition; moving it breaks the file with an undefined op
% Cruise is the primary linearisation point; climb, slow and bank are used for
% gain-schedule sanity checks, robustness sweeps and trim-envelope limits.
op.cruise.h_m = 150.0;            % m AGL
op.cruise.V_mps = 25.0;           % m/s TAS
op.cruise.gamma_deg = 0.0;        % deg flight path angle
op.cruise.bank_deg = 0.0;         % deg
op.cruise.phi_deg = 0.0;
op.cruise.theta_deg = 0.0;        % solved by trim, stored back by the trim script

op.climb.h_m = 150.0;
op.climb.V_mps = 23.0;
op.climb.gamma_deg = 6.0;
op.climb.bank_deg = 0.0;

op.slow.h_m = 150.0;
op.slow.V_mps = 18.0;
op.slow.gamma_deg = 0.0;
op.slow.bank_deg = 0.0;

op.bank.h_m = 150.0;
op.bank.V_mps = 23.0;
op.bank.gamma_deg = 0.0;
op.bank.bank_deg = 30.0;
op.bank.phi_deg = 30.0;
op = uacs_apply_override('op', op);

%% Control surface limits
% Declared before geometry because the horizontal tail is sized by the elevator
% travel available at the low-speed corner.
act.aileron.max_deg = 25.0;    % ESTIMATE.
act.aileron.rate_deg_s = 120.0;
act.aileron.tau_s = 0.045;
act.aileron.trim_deg = 0.0;

act.elevator.max_deg = 25.0;   % ESTIMATE.
act.elevator.rate_deg_s = 100.0;
act.elevator.tau_s = 0.045;
act.elevator.trim_deg = 0.0;   % set by trim

act.rudder.max_deg = 25.0;     % ESTIMATE.
act.rudder.rate_deg_s = 100.0;
act.rudder.tau_s = 0.045;
act.rudder.trim_deg = 0.0;

% Servo resolution and backlash are not modelled in v1
act = uacs_apply_override('act', act);

%% Geometry
% DESIGN CONSTRAINT: total span must not exceed 3.5 m; the span is asserted
% against the limit at init
geom.span_limit_m = 3.50;      % m, hard limit. Team requirement.
geom.b = 3.40;                 % m, wing span, 0.10 m inside the limit. ESTIMATE.
geom.AR = 5.00;                % aspect ratio, the free shape parameter. ESTIMATE.
geom = uacs_apply_override('geom', geom);
geom.S = geom.b^2 / geom.AR;   % m^2, wing reference area. SIZED from b and AR.
geom.cbar = geom.S / geom.b;   % m, mean aerodynamic chord. SIZED.
% Wing lift slope, needed by the tail sizing below; repeated as aero.CL_alpha with
% the same value, and the self-check asserts the two agree
aero_CL_alpha_design = 4.65;   % 1/rad, finite-wing estimate. ESTIMATE.
geom.taper = 0.75;             % tip/root chord ratio. ESTIMATE.
geom.dihedral_deg = 3.0;       % deg, wing dihedral. ESTIMATE.
geom.sweep_deg = 0.0;          % deg, quarter-chord sweep. ESTIMATE.
geom.x_ac = 0.0;               % m, wing aerodynamic centre, by definition the origin.
geom.iwing_deg = 2.0;          % deg, wing rigging incidence. ESTIMATE.

% The horizontal tail is sized by the LOW-SPEED TRIM requirement rather than by
% static margin: at this wing loading the elevator runs out of travel well before
% the wing stalls, and Sh/S = 0.16 buys usable low-speed envelope while keeping
% the static margin inside the plausibility band that init_uacs_twin asserts
geom.Sh_over_S = 0.16;         % horizontal tail area / wing area. ESTIMATE, trim-sized.
geom.lh = 1.95;                % m, wing AC to tail AC arm. ESTIMATE.
geom.ih_deg = -1.0;            % deg, horizontal tail incidence. ESTIMATE.
geom = uacs_apply_override('geom', geom);
geom.Sh = geom.Sh_over_S * geom.S;      % m^2. SIZED.
geom.Vh = geom.Sh * geom.lh / (geom.S * geom.cbar);    % tail volume. SIZED.
geom.elevator_trim_frac = 0.80;   % max fraction of elevator travel, design intent.

% The fin is SIZED by the single-engine control requirement. What sets V_mc is the
% RATIO of rudder authority to weathercock stiffness, Cn_dr/Cn_beta, not the fin
% area alone:
%   Cn_beta =  -Vv * CY_beta_tail                  (stiffness, hurts V_mc)
%   Cn_dr   =  -Vv * CY_dr * (S_rudder/S_fin)      (authority, helps V_mc)
%   V_mc    = sqrt( 2 * T_peak * y_motor / (rho * S * b * |Cn_dr| * dr_max) )
geom.Sv = 0.600;               % m^2, vertical tail area. ESTIMATE, control-sized.
geom.lv = 1.95;                % m, wing AC to vertical tail AC arm. ESTIMATE.
geom = uacs_apply_override('geom', geom);
geom.Vv = geom.Sv * geom.lv / (geom.S * geom.b);      % tail volume. SIZED.

geom.aileron.span_frac = 0.45;    % fraction of semispan per aileron. ESTIMATE.
geom.aileron.chord_frac = 0.25;   % fraction of local chord. ESTIMATE.
geom.aileron.y_inboard = 0.35;    % fraction of semispan, inboard edge. ESTIMATE.
geom.elevator.chord_frac = 0.40;  % fraction of tail chord. ESTIMATE, control-sized.
geom.rudder.chord_frac = 0.50;    % fraction of fin chord. ESTIMATE, control-sized.

%% Aerodynamics
% All derivatives are per radian unless the name ends in _deg.
% Cm is about the body y axis, Cn about the body z axis, Cl about the body x
% axis, all in the standard body convention.
%
% Sign conventions for the control derivatives, chosen so that a positive
% command produces the named aircraft motion:
%   elevator  positive = trailing edge down = nose-down pitching moment
%   aileron   positive = right aileron down = right-wing-down rolling moment
%   rudder    positive = trailing edge left = nose-left yawing moment
% The signs above are enforced by assertions in init_uacs_twin, so the trim and
% control scripts may rely on them without re-deriving them.

aero.CL0 = 0.22;          % wing-fuselage lift at alpha = 0. ESTIMATE.
aero.CL_alpha = 4.65;     % 1/rad, AR 5.8 finite-wing estimate. ESTIMATE.

% Control-surface effectiveness comes from flap theory, so it moves with the
% surface geometry instead of staying behind:
%   lift increment = tau * CL_alpha * (S_surface/S_ref) * deflection
% tau is roughly 0.65 for the 40 percent chord elevator, 0.50 for the rudder.
aero.tau_elevator = 0.65;    % ESTIMATE, 40 percent chord elevator.
aero.tau_rudder = 0.50;      % ESTIMATE, 50 percent chord rudder.
aero.CL_q = 3.90;         % 1/rad, pitch-rate lift, 2*CL_alpha*lh/cbar. ESTIMATE.
aero.CL_de = aero.tau_elevator * aero.CL_alpha * geom.Sh / geom.S;   % SIZED.
aero.CL_flap = 0.0;       % 1/rad, flaps not modelled in v1.
aero.CL_max = 1.33;       % clean stall. ESTIMATE.
aero.alpha_stall_deg = 14.0;   % deg. ESTIMATE.

aero.CD0 = 0.0300;        % zero-lift drag. ESTIMATE.
aero.e_oswald = 0.78;     % Oswald efficiency, AR 5.8. ESTIMATE.
aero.k_induced = 1.0 / (pi * geom.AR * aero.e_oswald);   % SIZED.
aero.CD_alpha = 0.10;     % 1/rad^2, drag rise away from CL_min. ESTIMATE.
aero.CD_de = 0.010;       % 1/rad. ESTIMATE.
aero.CD_beta = 0.0;       % 1/rad. ESTIMATE.

% Zero-lift pitching moment is DERIVED from the tail setting, and the tail setting
% from the trim target of zero elevator at the design cruise point:
%   Cm(trim) = Cm0_wing - Vh*CL_alpha*a_t + Cm_alpha*alpha_trim = 0
%   =>  a_t = (Cm0_wing + Cm_alpha*alpha_trim) / (Vh*CL_alpha)
%   =>  eps_downwash = i_tail_rel_wing - a_t
% The tail angle is measured from the WING chord line, which makes the wing's own
% zero-lift angle drop out.
aero.CL0 = 0.22;          % wing-fuselage lift at alpha = 0. ESTIMATE.
aero.alpha0_wing = -aero.CL0 / aero.CL_alpha;   % rad, wing zero-lift angle. SIZED.
aero.Cm0_wing = -0.012;   % ESTIMATE, camber moment about the wing AC.
aero.i_tail_rel_wing_deg = 1.5;   % deg, tail setting relative to the wing chord. ESTIMATE.
% Static stability, referenced to the wing aerodynamic centre:
%   Cm_alpha = Cm_alpha_wb - Vh*CL_alpha_tail
aero.Cm_alpha_wing_body = 0.020;   % ESTIMATE, small positive at the wing AC.
aero.Cm_alpha = aero.Cm_alpha_wing_body - geom.Vh * aero.CL_alpha;   % SIZED.
aero.Cm_q = -2 * aero.CL_alpha * geom.Vh * (geom.lh / geom.cbar);    % SIZED.
aero.Cm_de = -aero.CL_de * geom.Vh * (geom.lh / geom.cbar);          % SIZED.
aero.Cm_alphadot = -2.5;  % 1/rad, downwash lag. ESTIMATE.

% Trim target: zero elevator at the design cruise point
q_design = 0.5 * env.rho_sl * op.cruise.V_mps^2;
CL_design = mass.m * env.g / (q_design * geom.S);
aero.alpha_trim_design = (CL_design - aero.CL0) / aero.CL_alpha;   % rad. SIZED.
aero.alpha_tail_zerolift = (aero.Cm0_wing + aero.Cm_alpha * aero.alpha_trim_design) / ...
    (geom.Vh * aero.CL_alpha);                                     % rad. SIZED.
aero.eps_downwash_deg = aero.i_tail_rel_wing_deg - rad2deg(aero.alpha_tail_zerolift);  % SIZED.
aero.Cm0 = aero.Cm0_wing - geom.Vh * aero.CL_alpha * aero.alpha_tail_zerolift;        % SIZED.

aero.CY_beta = -0.750;    % side force due to sideslip. ESTIMATE.
% Rudder side force, from the same flap relation as the elevator but on the fin
% planform, then derated because a fin sits in the fuselage and wing wake:
%   CY_dr = k_body * tau_rudder * CL_alpha * Sv / S,   k_body = 0.85
aero.k_body_rudder = 0.85;   % ESTIMATE, fin in fuselage and wing wake.
aero.CY_dr = aero.k_body_rudder * aero.tau_rudder * aero.CL_alpha * geom.Sv / geom.S;

aero.Cl_beta = -0.090;    % dihedral effect, high wing plus 3 deg dihedral. ESTIMATE.
aero.Cl_p = -aero.CL_alpha / 12.0;   % roll damping, AR 5.8 estimate. SIZED.
aero.Cl_r = 0.050;        % 1/rad. ESTIMATE.
aero.Cl_da = 0.0412;      % aileron control power, from aileron geometry. SIZED.
aero.Cl_dr = 0.0100;      % roll from rudder. ESTIMATE.
aero.Cl_betadot = 0.0;    % neglected in v1.

% Weathercock stability and yaw damping, both tail-volume driven.
% Sign chain, in the body convention used throughout this file: a positive
% sideslip puts the fin at a positive local incidence, and the resulting side
% force acts in the negative y direction behind the CG, giving a nose-left
% restoring yaw:
%   Cn_beta = -Vv * CY_beta_tail                    (positive, restoring)
%   Cn_r    =  2 * Vv * CY_beta_tail                (negative, damping)
%   Cn_dr   = -Vv * CY_dr * (S_rudder / S_fin)      (negative)
% The rudder term carries the rudder-to-fin area ratio, equal to the rudder chord
% fraction for a full-span rudder; without that factor the chord fraction is dead
% data
aero.CY_beta_tail = -0.750;                      % ESTIMATE, side force per rad of sideslip.
aero.Cn_beta = -geom.Vv * aero.CY_beta_tail;     % SIZED, must come out positive.
aero.Cn_p = -0.030;       % 1/rad. ESTIMATE.
aero.Cn_r = 2 * geom.Vv * aero.CY_beta_tail;     % SIZED, must come out negative.
aero.Cn_da = -0.0060;     % adverse yaw from aileron. ESTIMATE.
aero.Cn_dr = -geom.Vv * aero.CY_dr * geom.rudder.chord_frac;   % SIZED.
aero.Cn_betadot = 0.0;    % neglected in v1.

%% Propulsion
% Twin wing-mounted motors with counter-rotating propellers, which cancels the
% torque and P-factor asymmetry of a conventional twin
prop.kind = 'counter_rotating';
prop.D = 0.4064;          % m, 16 inch propeller. ESTIMATE.
prop.pitch_in = 8.0;      % inch, nominal pitch. ESTIMATE.
prop.rpm_static = 6000;   % rpm at full throttle, static. ESTIMATE.
prop.n_motors = 2;
prop.endurance_target_s = 1200;   % s, 20 min. Team requirement.

prop.power.peak_per_motor_W = 1500;    % W electrical, short burst. ESTIMATE.
% Sized by the airframe mass budget, which allows only a modest 1.5 kW class motor
prop.power.eta_prop_cruise = 0.72;     % ESTIMATE.
prop.power.eta_motor = 0.88;           % ESTIMATE.
prop.power.eta_esc = 0.95;             % ESTIMATE.
prop.power.eta_total_cruise = prop.power.eta_prop_cruise * ...
                              prop.power.eta_motor * prop.power.eta_esc;   % SIZED.
prop.power.figure_of_merit = 0.55;     % static thrust efficiency. ESTIMATE.

% Static thrust is DERIVED from the peak shaft power by momentum theory rather
% than entered independently, so the two cannot contradict each other:
%   T_static = (P_shaft * FM * sqrt(2*rho*A))^(2/3),  A = pi*D^2/4
% FM is the ideal-to-actual power ratio, P_ideal = T*v_i and P_actual = P_ideal/FM
prop = uacs_apply_override('prop', prop);
prop.disk_area = pi * prop.D^2 / 4;                             % SIZED.
prop.P_shaft_peak = prop.power.peak_per_motor_W * ...
                    prop.power.eta_motor * prop.power.eta_esc;  % SIZED.
prop.T_static_max = (prop.P_shaft_peak * prop.power.figure_of_merit * ...
                     sqrt(2 * env.rho_sl * prop.disk_area))^(2/3);   % SIZED, N per motor.
% Throttle is the fraction of PEAK SHAFT POWER, not of thrust, and the resulting
% thrust follows power^(2/3), so thrust goes as throttle^(4/3)
prop.throttle_exponent = 2/3;  % ESTIMATE, throttle to shaft power.

% Available thrust versus advance speed, tabulated at init from the same momentum
% theory; do not replace it with a fitted ram-drag line, whose slope feeds the
% thrust-speed coupling in the A matrix
prop.speed_table_V = [0 5 10 15 20 25 30 35 40 45];    % m/s. SIZED
prop.speed_table_T = local_thrust_table(prop, env.rho_sl);   % N per motor. SIZED

prop.y_motor = 0.90;      % m, motor station outboard of the centreline. ESTIMATE.
prop.x_motor = -0.05;     % m. ESTIMATE.
% Thrust line offset taken from the LAYOUT, not from the trim budget below: the
% budget is a limit to check against, not a value to consume
prop.z_motor = -0.10;     % m, motor axis above the reference origin. ESTIMATE.
prop.z_thrust_offset = prop.z_motor - mass.z_cg;   % m. SIZED, negative is above the CG.
% Layout limit: the engine-out nose-down moment must stay inside the elevator
% budget, so the thrust line offset is bounded by
%   T_static_max * |z_offset| <= engine_out_margin * q_cruise * S * cbar
%                                * |Cm_de| * de_max
prop.engine_out_margin = 0.80;   % fraction of full elevator authority reserved. ESTIMATE.
prop.z_thrust_offset_max = prop.engine_out_margin * ...
    (0.5 * env.rho_sl * op.cruise.V_mps^2) * geom.S * geom.cbar * ...
    abs(aero.Cm_de) * deg2rad(act.elevator.max_deg) / prop.T_static_max;   % SIZED, m.

% In-flight thrust comes from prop.speed_table_T against prop.speed_table_V,
% interpolated by thrust_from_speed
prop.differential_limit = 0.60;% structural cap on |differential| throttle. ESTIMATE.
prop.esc.tau_s = 0.060;        % ESC plus motor electrical time constant. ESTIMATE.
prop.esc.rate_per_s = 4.0;     % throttle units per second. ESTIMATE.
prop.esc.min_throttle = 0.0;
prop.esc.max_throttle = 1.0;

op.bank.bank_deg = 30.0;
op.bank.phi_deg = 30.0;

%% Atmosphere and wind
% Density model: 'const' holds P.env.rho_sl at every altitude, 'isa' evaluates
% plant/atmo_isa.m at the density altitude, which is -pd from the state when
% h_from_state is true, else the fixed h_ref. At 150 m ISA is 1.43 percent below
% sea level, so switching the model moves every trim, mode and envelope number.
atmo.model = 'const';
atmo.h_from_state = true;      % true -> density altitude = -pd, the state altitude
atmo.h_ref = 0.0;              % m, density altitude used when h_from_state is false
atmo.dT_sl = 0.0;              % K offset from ISA
% Wind is applied at the model boundary, not read from here: the plant model takes
% the NED wind from its wind inport and shifts the velocity there; wind_ned below
% is the steady value a plant-level study carries
atmo.wind_ned = [0.0; 0.0; 0.0];      % m/s steady wind, NED
atmo.gust_rms = [0.0; 0.0; 0.0];      % m/s, Dryden-lite rms, used by the gust case
atmo.gust_tau = [1.0; 1.0; 1.0];      % s, first-order shaping time constants
atmo = uacs_apply_override('atmo', atmo);

%% Sensor models
sensor.imu.rate_hz = 400.0;         % gyro and accelerometer update. ESTIMATE.
sensor.imu.gyro_noise_dps = 0.05;   % deg/s, 1-sigma. Datasheet class.
sensor.imu.gyro_bias_dps = 0.50;    % deg/s, constant bias after calibration.
sensor.imu.accel_noise_g = 0.003;   % g, 1-sigma.
sensor.imu.tau_s = 0.002;           % s, anti-alias and transport lag.

sensor.airspeed.rate_hz = 50.0;
sensor.airspeed.noise_mps = 0.30;   % m/s, 1-sigma.
sensor.airspeed.tau_s = 0.050;      % s, pitot lag.
sensor.airspeed.bias_mps = 0.50;

sensor.baro.rate_hz = 50.0;
sensor.baro.noise_m = 0.30;         % m, 1-sigma.
sensor.baro.tau_s = 0.020;

sensor.gnss.rate_hz = 5.0;
sensor.gnss.noise_m = 1.50;         % m, 1-sigma horizontal.
sensor.gnss.noise_mps = 0.10;       % m/s, 1-sigma velocity.
sensor.gnss.tau_s = 0.200;          % s, latency.

sensor.mag.rate_hz = 50.0;
sensor.mag.noise_deg = 1.0;

%% Sampling and simulation
sim.Ts_ctrl = 0.0025;      % s, 400 Hz inner loop. Matches typical gyro rate.
sim.Ts_outer = 0.010;      % s, 100 Hz attitude and energy loops.
sim.Ts_alloc = 0.010;      % s, 100 Hz control allocation and mixer.
sim.Ts_sensor = 0.0025;
sim.solver = 'ode4';       % fixed step, deterministic and code-generation safe
sim.solver_variable = 'ode45';   % cross-check solver for step-size independence
sim.rel_tol = 1e-8;
sim.abs_tol = 1e-10;
sim.t_end_s = 60.0;
sim = uacs_apply_override('sim', sim);

%% Controller bandwidth targets
ctrl.bw.rate_p_rad_s = 12.0;      % roll rate loop target crossover
ctrl.bw.rate_q_rad_s = 8.0;       % pitch rate loop target crossover
ctrl.bw.rate_r_rad_s = 5.0;       % yaw rate loop target crossover
ctrl.bw.att_phi_rad_s = 3.0;      % roll attitude loop, about 1/4 of the rate loop
ctrl.bw.att_theta_rad_s = 2.5;    % pitch attitude loop
ctrl.bw.speed_rad_s = 0.8;        % airspeed loop, slowest
ctrl.bw.height_rad_s = 0.4;       % height loop
ctrl.rate_filter_tau_s = 0.010;   % first-order filter on the D term
ctrl.anti_windup_Kb = 1.0;        % back-calculation gain
ctrl.int_limit = struct('phi', deg2rad(15), 'theta', deg2rad(10), ...
                        'V', 5.0, 'h', 20.0);
ctrl = uacs_apply_override('ctrl', ctrl);

% ctrl.int_limit above is the ERROR-domain bound, the persistent error the loop is
% meant to absorb; these are the STATE-domain bounds, in rad*s, (m/s)*s or m*s,
% related by the loop time constant 1/bw and scaled by
% ctrl.int_state_margin. The clamp is a safety net, not the anti-windup mechanism:
% back-calculation is, and at margin 1 the clamp took over and degraded the pitch
% response. V6 asserts the clamp stays out of the way.
ctrl.int_state_margin = 8.0;
ctrl = uacs_apply_override('ctrl', ctrl);
ctrl.int_state_limit.phi = ctrl.int_state_margin * ctrl.int_limit.phi / ctrl.bw.att_phi_rad_s;
ctrl.int_state_limit.theta = ctrl.int_state_margin * ctrl.int_limit.theta / ctrl.bw.att_theta_rad_s;
ctrl.int_state_limit.V = ctrl.int_state_margin * ctrl.int_limit.V / ctrl.bw.speed_rad_s;
ctrl.int_state_limit.h = ctrl.int_state_margin * ctrl.int_limit.h / ctrl.bw.height_rad_s;

% Rate-command clamps for the attitude outer loops: the outer loop is a
% proportional law on attitude error, so any attitude error becomes a rate command
% however large, and without a clamp the aileron saturates until the aircraft rolls
% inverted.
%
% Each limit is DERIVED from the steady-state balance of the moment against its own
% damping derivative at the cruise condition, taken at 70 percent so the loop still
% has headroom to correct. Steady rate from a full-scale surface:
%   roll   p_ss = (|Cl_da|*da_max / |Cl_p|) * 2*V/b
%   pitch  q_ss = (|Cm_de|*de_max / |Cm_q|) * 2*V/cbar
ctrl.rate_cmd_frac = 0.70;
ctrl = uacs_apply_override('ctrl', ctrl);
ctrl.rate_cmd_limit.p_rad_s = ctrl.rate_cmd_frac * ...
   (abs(aero.Cl_da) * deg2rad(act.aileron.max_deg) / abs(aero.Cl_p)) * ...
   2 * op.cruise.V_mps / geom.b;
ctrl.rate_cmd_limit.q_rad_s = ctrl.rate_cmd_frac * ...
   (abs(aero.Cm_de) * deg2rad(act.elevator.max_deg) / abs(aero.Cm_q)) * ...
   2 * op.cruise.V_mps / geom.cbar;
% Yaw keeps the coordinated-turn limit; it is filled in after the limits section
% below, because bank_max_deg is defined there
ctrl.rate_cmd_limit.r_rad_s = NaN;

%% Controller weights (LQR)
% Bryson-style starting values, overwritten by the design script only after
% recording why
lqr.long.q_diag = [1/2.0^2, 1/deg2rad(5)^2, 1/deg2rad(15)^2, 1/deg2rad(10)^2];
lqr.long.r_diag = [1/deg2rad(20)^2, 1/0.30^2];
lqr.lat.q_diag  = [1/deg2rad(5)^2, 1/deg2rad(60)^2, 1/deg2rad(30)^2, 1/deg2rad(20)^2];
lqr.lat.r_diag  = [1/deg2rad(20)^2, 1/deg2rad(20)^2];
lqr = uacs_apply_override('lqr', lqr);

%% Limits and failsafe
% V_min is the LARGER of the stall margin and the lowest speed the aircraft can
% actually trim; at this wing loading trim binds, so the stall number alone would
% promise a speed the aircraft cannot fly.
%
% The trimmable minimum is NOT computed here, because a closed-form Cm relation is
% badly wrong at this wing loading. init_uacs_twin solves it with the real trim
% solver and OVERWRITES these two fields, so never read V_min_mps without running
% init_uacs_twin first.
limits.stall_margin_factor = 1.20;
limits = uacs_apply_override('limits', limits);
limits.V_stall_mps = sqrt(2 * mass.m * env.g / ...
    (env.rho_sl * geom.S * aero.CL_max));
limits.V_min_stall_mps = limits.stall_margin_factor * limits.V_stall_mps;
limits.V_min_trim_mps = NaN;      % filled by init_uacs_twin via the trim solver
limits.V_min_mps = NaN;           % filled by init_uacs_twin, = max(stall, trim)
% V_max is a LIMIT for the flight envelope and it must be reachable: at this mass
% and installed power the thrust-limited level-flight ceiling is about 30.9 m/s,
% and the self-check asserts V_max does not exceed it
limits.V_max_mps = 30.0;
limits.bank_max_deg = 45.0;
limits.pitch_max_deg = 30.0;
limits.climb_rate_max_mps = 6.0;
limits.h_max_m = 400.0;
limits.nz_max = 3.0;
limits.nz_min = -1.5;
limits = uacs_apply_override('limits', limits);

% Yaw rate-command clamp, filled here because it needs bank_max_deg above; it
% matches the coordinated-turn limit the pilot stick already uses, so the clamp can
% never be tighter than what the stick itself commands
ctrl.rate_cmd_limit.r_rad_s = min(env.g * tan(deg2rad(limits.bank_max_deg)) / ...
   op.cruise.V_mps, ctrl.bw.rate_r_rad_s);

%% Provenance
% meta.revision is incremented on every edit that can change a design number, and
% every job log and result file must print it, so a measurement stays reproducible
meta.revision = 11;
meta.version = '0.5.0';
meta.date = '2026-10-11';
% Revision history is recorded in the project log, not here
meta.author = 'Ash';
meta.aircraft = 'twin-engine high-wing, 20 kg MTOW, wing-mounted counter-rotating props';
meta.estimated_fields = { ...
    'mass.items', 'mass.m', 'geom.b', 'geom.S', 'geom.taper', 'geom.dihedral_deg', ...
    'geom.Sh', 'geom.Sv', 'geom.lh', 'geom.lv', 'geom.iwing_deg', 'geom.ih_deg', ...
    'geom.aileron', 'geom.elevator', 'geom.rudder', ...
    'aero.CL0', 'aero.CL_alpha', 'aero.CD0', 'aero.e_oswald', 'aero.CD_alpha', ...
    'aero.CD_de', 'aero.Cm0', 'aero.Cm_q', 'aero.Cm_alphadot', ...
    'aero.CY_beta', 'aero.Cl_beta', 'aero.Cl_r', 'aero.Cl_dr', ...
    'aero.Cn_p', 'aero.Cn_da', 'aero.CL_max', 'aero.alpha_stall_deg', ...
    'act', 'prop.T_static_max', 'prop.throttle_exponent', 'prop.k_ram', ...
    'prop.power', 'prop.differential_limit', 'prop.esc', 'sensor', 'ctrl', 'lqr'};
meta.sized_fields = { ...
    'mass.Ixx', 'mass.Iyy', 'mass.Izz', 'mass.Ixz', 'mass.x_cg', ...
    'geom.cbar', 'geom.AR', 'geom.Vh', 'geom.Vv', 'aero.k_induced', ...
    'aero.CL_q', 'aero.CL_de', 'aero.Cm_alpha', 'aero.Cm_de', ...
    'aero.CY_dr', 'aero.Cl_p', 'aero.Cl_da', 'aero.Cn_beta', 'aero.Cn_r', ...
    'aero.Cn_dr', 'prop.power.eta_total_cruise', 'prop.z_thrust_offset', ...
    'prop.disk_area', 'prop.P_shaft_peak', 'prop.T_static_max'};

%% Pack
P = struct();
P.env = env;
P.mass = mass;
P.geom = geom;
P.aero = aero;
P.act = act;
P.prop = prop;
P.op = op;
P.atmo = atmo;
P.sensor = sensor;
P.sim = sim;
P.ctrl = ctrl;
P.lqr = lqr;
P.limits = limits;
P.meta = meta;

% Short aliases, so scripts can write rho, g, m, b, S, cbar, AR without
% digging into the struct. Kept in P.tags so init_uacs_twin can export them.
P.tags = struct( ...
    'rho', env.rho_sl, 'g', env.g, 'm', mass.m, ...
    'b', geom.b, 'S', geom.S, 'cbar', geom.cbar, 'AR', geom.AR);
end

%% Local functions
function [xcg, ycg, zcg] = local_cg(items)
m = cell2mat(items(:, 2));
x = cell2mat(items(:, 3));
y = cell2mat(items(:, 4));
z = cell2mat(items(:, 5));
mt = sum(m);
xcg = sum(m .* x) / mt;
ycg = sum(m .* y) / mt;
zcg = sum(m .* z) / mt;
end

function [Ixx, Iyy, Izz, Ixz, scale] = local_inertia(items)
m = cell2mat(items(:, 2));
x = cell2mat(items(:, 3));
y = cell2mat(items(:, 4));
z = cell2mat(items(:, 5));
r2 = x.^2 + y.^2 + z.^2;
% Point-mass inertia plus a distributed-mass term k*m*L^2 per axis, with L taken
% as a fraction of the mean distance of the items from the origin
L = 0.55 * mean(sqrt(r2));
scale = L^2;                 % m^2
Ixx = sum(m .* (y.^2 + z.^2)) + sum(m) * scale * 0.35;
Iyy = sum(m .* (x.^2 + z.^2)) + sum(m) * scale * 0.55;
Izz = sum(m .* (x.^2 + y.^2)) + sum(m) * scale * 0.55;
Ixz = -sum(m .* x .* z);
end

function T = local_thrust_table(prop, rho)
% LOCAL_THRUST_TABLE Available thrust versus advance speed at full throttle.
% Momentum theory relates IDEAL power to momentum flux, so the shaft power is
% scaled by the figure of merit before use.
V = prop.speed_table_V;
T = zeros(size(V));
P_ideal = prop.P_shaft_peak * prop.power.figure_of_merit;
for k = 1:numel(V)
    T(k) = local_thrust_at_speed(V(k), P_ideal, rho, prop.disk_area);
end
end

function T = local_thrust_at_speed(V, P_shaft, rho, A)
% Solve P_shaft = 2*rho*A*v_i*(V + v_i)^2 for v_i, then T = 2*rho*A*v_i*(V+v_i).
% The power equation balances the whole ideal-power budget, which is what caps
% available thrust at high advance speed.
if P_shaft <= 0
    T = 0;
    return;
end
if V <= 0
    % Closed form at the static point; this is the T_static_max expression
    vi = (P_shaft / (2 * rho * A))^(1/3);
    T = 2 * rho * A * vi^2;
    return;
end
lo = 0;
hi = (P_shaft / (2 * rho * A))^(1/3) + 1;
guard = 0;
while 2 * rho * A * hi * (V + hi)^2 < P_shaft
    hi = hi * 2;
    guard = guard + 1;
    if guard > 200
        error('uacs_twin_params:ThrustSolve', 'bisection bracket failed to expand');
    end
end
for iter = 1:200
    mid = 0.5 * (lo + hi);
    if 2 * rho * A * mid * (V + mid)^2 < P_shaft
        lo = mid;
    else
        hi = mid;
    end
end
vi = 0.5 * (lo + hi);
T = 2 * rho * A * vi * (V + vi);
end
