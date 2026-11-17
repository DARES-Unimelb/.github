% UACS_GO  Single entry point. Edit the parameters and run the models from here.
%
% Three ways to run it:
%   1. Edit the numbers in section 1 below, then press F5.
%   2. Double-click model\uacs_flight_bench.slx (flight bench) or model\uacs_aero_test.slx
%      (aerodynamics). Opening a model runs this file first.
%   3. Type uacs_go at the command line.
%
% Rules for editing:
%   - Only fields that already exist in params/uacs_twin_params.m can be overridden here.
%     A misspelled field name errors immediately.
%   - Fill in only the original values (marked ESTIMATE in the parameter file). Derived
%     values (marked SIZED) are recomputed, so editing them has no effect.
%   - Any value differing from the frozen baseline prints a PARAMETER OVERRIDES ACTIVE
%     banner at startup.

%% Parameters

%% 1.1 Mass and centre of gravity
% Rows are {name, mass kg, x m, y m, z m} in body axes (x forward, y right, z down)
% with the origin at the wing aerodynamic centre. Total mass, centre of gravity and the
% inertias are derived from this table. The payload row holds the 12 kg payload.
uacs_mass = {
   'wing',             1.10,  -0.05,  0.00,  0.00
   'fuselage_boom',    0.55,   0.00,  0.00,  0.00
   'payload',         12.00,  -0.22,  0.00,  0.02
   'battery',          2.10,  -0.15,  0.00,  0.03
   'tail',             0.20,   1.95,  0.00,  0.08
   'motor_esc_left',   0.17,  -0.05,  0.90, -0.10
   'motor_esc_right',  0.17,  -0.05, -0.90, -0.10
   'prop_left',        0.03,  -0.05,  0.90, -0.10
   'prop_right',       0.03,  -0.05, -0.90, -0.10
   'landing_gear',     0.20,  -0.10,  0.00,  0.18
   'servos_links',     0.20,  -0.10,  0.00,  0.05
   'margin',           0.25,   0.00,  0.00,  0.00
   };

%% 1.2 Wing and tail geometry
% Span and aspect ratio are the only free wing variables; area S = b^2/AR, mean chord
% cbar = S/b and the tail volumes are derived from them. Span is hard limited to 3.50 m.
uacs_wing.span_m       = 3.40;    % span m, limit 3.50
uacs_wing.aspect_ratio = 5.00;    % aspect ratio, free wing shape parameter
uacs_wing.taper        = 0.75;    % taper ratio
uacs_wing.dihedral_deg = 3.0;     % dihedral deg
uacs_wing.incidence_deg= 2.0;     % wing incidence deg

uacs_tail.h_area_ratio = 0.16;    % horizontal tail area / wing area
uacs_tail.h_arm_m      = 1.95;    % wing to tail aerodynamic centre arm m
uacs_tail.h_incidence_deg = -1.0; % horizontal tail incidence deg
uacs_tail.v_area_m2    = 0.600;   % vertical tail area m^2
uacs_tail.v_arm_m      = 1.95;    % vertical tail arm m

%% 1.3 Aerodynamic derivatives (read-only copy, not applied)
% To change these edit params/uacs_twin_params.m; the values below are a copy for reference.
uacs_aero.CL0           = 0.220;    % lift coefficient at zero angle of attack
uacs_aero.CL_alpha      = 4.65;     % lift curve slope, 1/rad
uacs_aero.CL_max        = 1.33;     % stall lift coefficient
uacs_aero.alpha_stall_deg = 14.0;   % stall angle of attack deg
uacs_aero.CD0           = 0.030;    % zero-lift drag coefficient
uacs_aero.e_oswald      = 0.780;    % Oswald efficiency, sets induced drag
uacs_aero.CD_alpha      = 0.100;    % drag term quadratic in angle of attack
uacs_aero.CD_de         = 0.010;    % elevator trim drag
uacs_aero.Cm0_wing      = -0.012;   % wing-body zero-lift pitching moment
uacs_aero.Cm_alpha_wing_body = 0.020;  % wing-body pitch stiffness
uacs_aero.Cm_q          = -12.236;  % pitch damping
uacs_aero.Cm_alphadot   = -2.500;   % downwash lag derivative
uacs_aero.tau_elevator  = 0.650;    % elevator effectiveness factor
uacs_aero.tau_rudder    = 0.500;    % rudder effectiveness factor
uacs_aero.CY_beta       = -0.750;   % side force derivative
uacs_aero.Cl_beta       = -0.090;   % dihedral effect
uacs_aero.Cl_p          = -0.388;   % roll damping
uacs_aero.Cl_r          = 0.050;    % roll-yaw cross derivative
uacs_aero.Cl_da         = 0.0412;   % aileron control power
uacs_aero.Cl_dr         = 0.010;    % rudder roll cross derivative
uacs_aero.Cn_p          = -0.030;   % yaw-roll cross derivative
uacs_aero.Cn_da         = -0.006;   % aileron adverse yaw

%% 1.4 Actuators
% tau_s is the first-order servo time constant, rate_deg_s the maximum rate deg/s.
uacs_act.aileron_max_deg  = 25.0;   uacs_act.aileron_rate_deg_s  = 120.0;  uacs_act.aileron_tau_s  = 0.045;
uacs_act.elevator_max_deg = 25.0;   uacs_act.elevator_rate_deg_s = 100.0;  uacs_act.elevator_tau_s = 0.045;
uacs_act.rudder_max_deg   = 25.0;   uacs_act.rudder_rate_deg_s   = 100.0;  uacs_act.rudder_tau_s   = 0.045;

%% 1.5 Propulsion
% Throttle is a fraction of peak shaft power, not a speed.
uacs_prop.peak_power_per_motor_W = 1500.0;  % peak shaft power per motor W
uacs_prop.diameter_m      = 0.4064;  % propeller diameter m
uacs_prop.figure_of_merit = 0.550;   % propeller figure of merit
uacs_prop.esc_tau_s       = 0.060;   % ESC first-order lag s
uacs_prop.y_motor_m       = 0.900;   % motor lateral station m, the differential thrust arm
uacs_prop.z_motor_m       = -0.100;  % motor vertical station m, thrust line offset
uacs_prop.differential_limit = 0.600;% differential throttle structural limit

%% 1.6 Control targets (PID)
% These seven bandwidths are design targets. The inner rate loop does not use bw_rate_*:
% local_inner_target in control/design_pid.m derives it from the body structural mode.
uacs_ctrl.bw_rate_p_rad_s     = 12.0;   % roll rate loop
uacs_ctrl.bw_rate_q_rad_s     =  8.0;   % pitch rate loop
uacs_ctrl.bw_rate_r_rad_s     =  5.0;   % yaw rate loop
uacs_ctrl.bw_att_phi_rad_s    =  3.0;   % roll attitude loop
uacs_ctrl.bw_att_theta_rad_s  =  2.5;   % pitch attitude loop
uacs_ctrl.bw_speed_rad_s      =  0.8;   % airspeed loop
uacs_ctrl.bw_height_rad_s     =  0.4;   % height loop
uacs_ctrl.rate_filter_tau_s   = 0.010;  % derivative term first-order filter time constant
uacs_ctrl.anti_windup_Kb      = 1.0;    % back-calculation anti-windup gain
uacs_ctrl.int_state_margin    = 8.0;    % integral state bound = margin * error bound / bandwidth
uacs_ctrl.rate_cmd_frac       = 0.70;   % rate command limit as a fraction of the full-stick steady rate
uacs_ctrl.int_limit_phi_deg   = 15.0;   % sustained error the roll loop may absorb deg
uacs_ctrl.int_limit_theta_deg = 10.0;   % pitch loop deg
uacs_ctrl.int_limit_V_mps     =  5.0;   % airspeed loop m/s
uacs_ctrl.int_limit_h_m       = 20.0;   % height loop m

%% 1.7 LQR weights
% Bryson form: q penalises states (larger is tighter), r penalises control
% (larger saves actuation).
uacs_lqr.long_q = [1/2.0^2, 1/deg2rad(5)^2, 1/deg2rad(15)^2, 1/deg2rad(10)^2];  % [dV dalpha dq dtheta]
uacs_lqr.long_r = [1/deg2rad(20)^2, 1/0.30^2];                                  % [dde dT]
uacs_lqr.lat_q  = [1/deg2rad(5)^2, 1/deg2rad(60)^2, 1/deg2rad(30)^2, 1/deg2rad(20)^2]; % [dbeta dp dr dphi]
uacs_lqr.lat_r  = [1/deg2rad(20)^2, 1/deg2rad(20)^2];                           % [dda ddr]

%% 1.8 Envelope, trim point, atmosphere
uacs_limits.bank_max_deg       = 45.0;   % maximum bank angle deg
uacs_limits.pitch_max_deg      = 30.0;   % maximum pitch angle deg
uacs_limits.V_max_mps          = 30.0;   % maximum airspeed m/s, must be reachable or the self-check rejects it
uacs_limits.climb_rate_max_mps =  6.0;   % maximum climb rate m/s
uacs_limits.h_max_m            = 400.0;  % maximum height m
uacs_limits.nz_max             =  3.0;   % maximum load factor
uacs_limits.nz_min             = -1.5;   % minimum load factor
uacs_cruise.V_mps              = 25.0;   % design trim airspeed m/s
uacs_cruise.h_m                = 150.0;  % design trim height m
uacs_atmo.model                = 'const';% 'const' sea-level density, or 'isa' varying with altitude

%% 1.9 Simulation duration and sample times
% These two are what the Simulink toolbar StopTime field shows:
%   uacs_sim.t_end_s      -> variable sim_t_end,      models under test and scenario jobs
%   uacs_sim.bench_run_s  -> variable sim_bench_t_end, interactive bench uacs_flight_bench
% A one-off run can also type a number straight into the StopTime field.
uacs_sim.t_end_s     = 60.0;     % run length s, scenario jobs
uacs_sim.bench_run_s = 20.0;     % bench run length s; about 7 s of wall clock per simulated second at 0.0025 s
uacs_sim.Ts_ctrl_s   = 0.0025;   % inner loop sample time, also the fixed-step solver step
uacs_sim.Ts_outer_s  = 0.0100;   % attitude, airspeed and height loop sample time
uacs_sim.Ts_alloc_s  = 0.0100;   % allocator sample time
uacs_sim.Ts_sensor_s = 0.0025;   % sensor sample time

%% 1.10 Disturbances: steady wind and gust
% The bench canvas also has a crosswind slider, added to the east component only.
uacs_dist.wind_ned_mps  = [0; 0; 0];  % steady wind, NED components [north east down] m/s
uacs_dist.gust.on       = false;      % gust master switch
uacs_dist.gust.amp_mps  = 3.0;        % gust amplitude m/s
uacs_dist.gust.freq_hz  = 0.5;        % gust frequency Hz, used as 2*pi*f
uacs_dist.gust.dir_ned  = [1; 0; 0];  % gust direction, NED; normalised to a unit vector before use

%% 1.11 Joystick (optional channel)
% JoystickID accepts 'None' or 'Joystick1' to 'Joystick16'. 'None' touches no hardware
% and outputs zero, so the bench runs without a stick. For a real stick set it to
% 'Joystick1', rerun run/build_bench_models.m (the block parameter is written at build
% time) and switch the bench stick source to the joystick. Set pitch_sign to -1 if the
% pitch direction comes out reversed.
uacs_joystick.id         = 'None';
uacs_joystick.pitch_sign = 1.0;

%% Machinery (do not edit)

% Pack the groups above and hand them to params/uacs_apply_config.m. uacs_aero is passed
% along as well but is not applied there, see section 1.3.
uacs = struct();
uacs.mass = uacs_mass;
uacs.wing = uacs_wing;
uacs.tail = uacs_tail;
uacs.aero = uacs_aero;
uacs.act = uacs_act;
uacs.prop = uacs_prop;
uacs.ctrl = uacs_ctrl;
uacs.lqr = uacs_lqr;
uacs.limits = uacs_limits;
uacs.cruise = uacs_cruise;
uacs.atmo = uacs_atmo;
uacs.sim = uacs_sim;
uacs.dist = uacs_dist;
uacs.joystick = uacs_joystick;
uacs_apply_config(uacs);

clear uacs uacs_mass uacs_wing uacs_tail uacs_aero uacs_act uacs_prop uacs_ctrl ...
   uacs_lqr uacs_limits uacs_cruise uacs_atmo uacs_sim uacs_dist uacs_joystick
