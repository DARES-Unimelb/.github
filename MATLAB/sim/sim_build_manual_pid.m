function info = sim_build_manual_pid(P, opts)
% SIM_BUILD_MANUAL_PID Build and save model/uacs_manual_pid.slx.
%
% Manual-flight main line: stick in, actuator commands out, the aircraft closed
% through the plant. Structure, from the outside in
%   pilot_to_command      stick -> [phi_c theta_c r_c V_c h_c]   (S-Function block)
%   pid_phi, pid_theta    attitude outer loops -> p_c, q_c
%   pid_p, pid_q, pid_r   rate loops -> surface increments
%   pid_V                 airspeed loop -> total thrust increment
%   pid_h                 altitude loop -> theta_c, clamped at the pitch limit
%   theta_switch          attitude mode follows the stick, altitude mode follows pid_h
%   G_surf                surfaces -> desired aerodynamic moment
%   allocate              moment and thrust -> [da de dr dth_l dth_r]
%
% Inputs come from the base workspace through From Workspace blocks, so a
% scenario is a matrix the run script builds: sim_stick_ts for the sticks and
% sim_wind_ts for the wind. Outputs: state, actuator commands, the pilot command
% vector, the three rate commands, the moment command and the thrust command.
%
% opts.name          model name and file stem, default 'uacs_manual_pid'
% opts.stick_source  'workspace' reads sim_stick_ts; 'slider' puts four Slider Gain
%                    blocks on the diagram so a person can fly it with the mouse.
%                    The rest of the diagram is IDENTICAL in both modes: the
%                    interactive bench is this same cascade, not a copy of it, so
%                    the two cannot drift apart.
% opts.instrument    add Displays and a Scope for interactive use
% opts.mode_switch   replace the flight-mode Constant with a Manual Switch, so a
%                    person can flip between attitude mode and altitude mode while
%                    the model runs. Double-click the switch to toggle it.
% opts.stop_time     what the model's StopTime parameter should be, as a string. Left
%                    alone it stays 'sim_t_end', the scenario default; the interactive
%                    bench passes its own shorter workspace variable so that a person
%                    flying it does not wait out the scripted 60 s every time.
% opts.reload_fcn    model callback run on open and before every simulation, so the
%                    model initialises its own base workspace and a person can just
%                    double-click the .slx
% opts.ensure_fcn    a lightweight callback for InitFcn that only re-initialises when
%                    the base workspace is actually missing something, so pressing Run
%                    repeatedly does not re-run the whole parameter chain each time

if nargin < 2
   opts = struct();
end
if ~isfield(opts, 'name') || isempty(opts.name)
   opts.name = 'uacs_manual_pid';
end
if ~isfield(opts, 'stick_source') || isempty(opts.stick_source)
   opts.stick_source = 'workspace';
end
if ~isfield(opts, 'instrument') || isempty(opts.instrument)
   opts.instrument = false;
end
if ~isfield(opts, 'mode_switch') || isempty(opts.mode_switch)
   opts.mode_switch = false;
end
if ~isfield(opts, 'reload_fcn') || isempty(opts.reload_fcn)
   opts.reload_fcn = '';
end
if ~isfield(opts, 'ensure_fcn') || isempty(opts.ensure_fcn)
   opts.ensure_fcn = '';
end
if ~isfield(opts, 'stop_time') || isempty(opts.stop_time)
   opts.stop_time = '';
end
slider_stick = strcmp(opts.stick_source, 'slider');
mdl = opts.name;
file = fullfile(sim_root(), 'model', [mdl '.slx']);
sim_new_model(mdl, file);
if ~isempty(opts.stop_time)
   set_param(mdl, 'StopTime', opts.stop_time);
end

% ---------------- sources ----------------
if slider_stick
   % Four Slider Gains as the virtual stick. A Slider Gain multiplies its input by the
   % slider value, so each is fed a Constant 1 and the slider itself becomes the stick
   % fraction: -1..1 for roll, pitch and yaw, 0..1 for throttle. The slider is live
   % while the simulation runs, which is what makes this a joystick rather than a
   % parameter form.
   sim_add_block(mdl, 'stick_one', 'simulink/Sources/Constant', sim_pos(0, 0, 60, 30), ...
      'Value', 'sim_c_one');
   slider_spec = {'stick_roll', 0, 0, -1, 1, '滚转杆 (−1 左, +1 右)'; ...
                  'stick_pitch', 0, 2, -1, 1, '俯仰杆 (−1 低头, +1 抬头)'; ...
                  'stick_yaw', 0, 4, -1, 1, '偏航杆 (−1 左, +1 右)'; ...
                  'stick_thr', 0, 6, 0, 1, '油门 (0..1)'};
   for ks = 1:size(slider_spec, 1)
      sim_add_block(mdl, slider_spec{ks, 1}, 'simulink/Math Operations/Slider Gain', ...
         sim_pos(slider_spec{ks, 2}, slider_spec{ks, 3}, 70, 40), ...
         'Low', num2str(slider_spec{ks, 4}), 'High', num2str(slider_spec{ks, 5}), ...
         'Gain', num2str(0.5 * (slider_spec{ks, 4} + slider_spec{ks, 5})));
   end
   % The throttle starts at the cruise trim throttle rather than mid-scale, so the
   % aircraft begins in equilibrium instead of accelerating away. uacs_go pushes
   % sim_thr_trim from the solved trim.
   set_param([mdl '/stick_thr'], 'Gain', 'sim_thr_trim');
   sim_add_block(mdl, 'stick_mux', 'simulink/Signal Routing/Mux', sim_pos(1, 3, 20, 110), ...
      'Inputs', '4');
else
   sim_add_block(mdl, 'stick_src', 'simulink/Sources/From Workspace', sim_pos(0, 0, 90, 40), ...
      'VariableName', 'sim_stick_ts', 'Interpolate', 'on', ...
      'OutputAfterFinalValue', 'Holding final value');
end
% The wind source feeds a 3 element NED port. It reads a timeseries rather than a
% matrix because a matrix is ambiguous: From Workspace treats column 1 as time only
% when SampleTime is unset, and as a signal when it is set. A timeseries carries its
% own time vector, so the port width is exactly the 3 wind components.
if slider_stick
   % Interactive crosswind: an east-component slider with the north and down components
   % held at zero, packed into the 3 element NED port.
   sim_add_block(mdl, 'wind_e', 'simulink/Math Operations/Slider Gain', sim_pos(0, 12, 70, 40), ...
      'Low', '-10', 'High', '10', 'Gain', '0');
   sim_add_block(mdl, 'wind_zero', 'simulink/Sources/Constant', sim_pos(0, 14, 60, 30), ...
      'Value', 'sim_c_zero');
   sim_add_block(mdl, 'wind_mux', 'simulink/Signal Routing/Mux', sim_pos(1, 13, 20, 70), ...
      'Inputs', '3');
   sim_connect(mdl, 'stick_one/1', 'wind_e/1');
   sim_connect(mdl, 'wind_zero/1', 'wind_mux/1', 'wind_e/1', 'wind_mux/2', ...
      'wind_zero/1', 'wind_mux/3');
   wind_out = 'wind_mux/1';
else
   sim_add_block(mdl, 'wind_src', 'simulink/Sources/From Workspace', sim_pos(0, 12, 90, 40), ...
      'VariableName', 'sim_wind_ned', 'Interpolate', 'on', ...
      'OutputAfterFinalValue', 'Holding final value');
   wind_out = 'wind_src/1';
end
sim_add_block(mdl, 'const_utrim', 'simulink/Sources/Constant', sim_pos(3, 8, 80, 30), ...
   'Value', 'sim_u_trim_surf');
sim_add_block(mdl, 'const_Ttrim', 'simulink/Sources/Constant', sim_pos(6, 11, 80, 30), ...
   'Value', 'sim_T_trim');
sim_add_block(mdl, 'const_zero', 'simulink/Sources/Constant', sim_pos(9, 11, 60, 30), ...
   'Value', 'sim_c_zero');
if opts.mode_switch
   % Flight mode as a switch a person can flip WHILE the simulation runs: the Switch
   % block's criterion is u2 ~= 0, so port 1 (the 0) is attitude mode and port 2 (the
   % 1) is altitude mode. The two input lines are named so the diagram says which is
   % which without opening the switch.
   sim_add_block(mdl, 'mode_att', 'simulink/Sources/Constant', sim_pos(1, 0, 90, 26), ...
      'Value', 'sim_c_zero');
   sim_add_block(mdl, 'mode_alt', 'simulink/Sources/Constant', sim_pos(2, 0, 90, 26), ...
      'Value', 'sim_c_one');
   sim_add_block(mdl, 'mode_sw', 'simulink/Signal Routing/Manual Switch', ...
      sim_pos(2, 1, 40, 40), 'sw', '1');
   sim_connect(mdl, 'mode_att/1', 'mode_sw/1', 'mode_alt/1', 'mode_sw/2');
   sim_name_line(mdl, 'mode_att', '0 姿态模式：俯仰杆直接控 theta');
   sim_name_line(mdl, 'mode_alt', '1 高度模式：俯仰杆控爬升率');
   mode_out = 'mode_sw/1';
else
   sim_add_block(mdl, 'const_mode', 'simulink/Sources/Constant', sim_pos(2, 1, 60, 30), ...
      'Value', 'sim_flight_mode');
   mode_out = 'const_mode/1';
end

% ---------------- pilot, air data, state split ----------------
sim_add_block(mdl, 'cmd_pilot', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(1, 0, 110, 50), 'FunctionName', 'sim_pilot_sfun');
sim_add_block(mdl, 'cmd_split', 'simulink/Signal Routing/Demux', sim_pos(2, 0, 40, 90), ...
   'Outputs', '5');
sim_add_block(mdl, 'meas', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(1, 3, 100, 40), 'FunctionName', 'sim_meas_sfun');
sim_add_block(mdl, 'meas_split', 'simulink/Signal Routing/Demux', sim_pos(2, 3, 40, 90), ...
   'Outputs', '4');
sim_add_block(mdl, 'x_split', 'simulink/Signal Routing/Demux', sim_pos(1, 5, 40, 260), ...
   'Outputs', '12');
sim_add_block(mdl, 'h_gain', 'simulink/Math Operations/Gain', sim_pos(2, 5, 40, 30), ...
   'Gain', 'sim_c_neg1');

% ---------------- PID cascade ----------------
sim_add_block(mdl, 'phi_err', 'simulink/Math Operations/Sum', sim_pos(3, 2, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(mdl, 'theta_switch', 'simulink/Signal Routing/Switch', sim_pos(3, 1, 60, 40), ...
   'Criteria', 'u2 ~= 0', 'Threshold', 'sim_c_zero');
sim_add_block(mdl, 'theta_err', 'simulink/Math Operations/Sum', sim_pos(3, 3, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(mdl, 'theta_lim', 'simulink/Discontinuities/Saturation', sim_pos(3, 7, 60, 30), ...
   'UpperLimit', 'sim_pid_h_lim', 'LowerLimit', 'sim_pid_h_lim_neg');
sim_add_block(mdl, 'p_err', 'simulink/Math Operations/Sum', sim_pos(5, 2, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(mdl, 'q_err', 'simulink/Math Operations/Sum', sim_pos(5, 3, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(mdl, 'r_err', 'simulink/Math Operations/Sum', sim_pos(5, 4, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(mdl, 'V_err', 'simulink/Math Operations/Sum', sim_pos(5, 5, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_block(mdl, 'h_err', 'simulink/Math Operations/Sum', sim_pos(5, 6, 40, 30), ...
   'Inputs', '+-', 'IconShape', 'rectangular');
sim_add_pid(mdl, 'pid_phi', sim_pos(4, 2, 120, 60), 'phi');
sim_add_pid(mdl, 'pid_theta', sim_pos(4, 3, 120, 60), 'theta');
sim_add_pid(mdl, 'pid_V', sim_pos(4, 5, 120, 60), 'V');
sim_add_pid(mdl, 'pid_h', sim_pos(4, 6, 120, 60), 'h');
sim_add_pid(mdl, 'pid_p', sim_pos(6, 2, 120, 60), 'p');
sim_add_pid(mdl, 'pid_q', sim_pos(6, 3, 120, 60), 'q');
sim_add_pid(mdl, 'pid_r', sim_pos(6, 4, 120, 60), 'r');

% Rate command clamps between the attitude loops and the rate loops. Without them the
% outer proportional law turns any attitude error into an arbitrarily large rate demand:
% the roll stick at its stop asked for 17.73 rad/s against an airframe that sustains
% 1.23, the aileron saturated, sideslip reached 9 degrees and the aircraft rolled
% inverted. The limits are derived in params from the full-surface steady rate at cruise
% taken at 70 percent, so they move with the geometry instead of being tuned by hand.
sim_add_block(mdl, 'p_c_lim', 'simulink/Discontinuities/Saturation', sim_pos(5, 1, 60, 30), ...
   'UpperLimit', 'sim_rate_cmd_lim_p', 'LowerLimit', '-sim_rate_cmd_lim_p');
sim_add_block(mdl, 'q_c_lim', 'simulink/Discontinuities/Saturation', sim_pos(5, 2, 60, 30), ...
   'UpperLimit', 'sim_rate_cmd_lim_q', 'LowerLimit', '-sim_rate_cmd_lim_q');
sim_add_block(mdl, 'r_c_lim', 'simulink/Discontinuities/Saturation', sim_pos(5, 3, 60, 30), ...
   'UpperLimit', 'sim_rate_cmd_lim_r', 'LowerLimit', '-sim_rate_cmd_lim_r');
sim_add_block(mdl, 'rate_mux', 'simulink/Signal Routing/Mux', sim_pos(7, 2, 30, 100), ...
   'Inputs', '3');

% ---------------- mixing: surfaces to moment, thrust, allocation ----------------
sim_add_block(mdl, 'utrim_split', 'simulink/Signal Routing/Demux', sim_pos(4, 8, 40, 90), ...
   'Outputs', '3');
sim_add_block(mdl, 'da_sum', 'simulink/Math Operations/Sum', sim_pos(5, 8, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(mdl, 'de_sum', 'simulink/Math Operations/Sum', sim_pos(5, 9, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(mdl, 'dr_sum', 'simulink/Math Operations/Sum', sim_pos(5, 10, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(mdl, 'surf_mux', 'simulink/Signal Routing/Mux', sim_pos(6, 8, 30, 110), ...
   'Inputs', '3');
sim_add_block(mdl, 'G_gain', 'simulink/Math Operations/Gain', sim_pos(7, 8, 80, 40), ...
   'Gain', 'sim_G_surf', 'Multiplication', 'Matrix(K*u)');
sim_add_block(mdl, 'T_sum', 'simulink/Math Operations/Sum', sim_pos(7, 11, 40, 30), ...
   'Inputs', '++', 'IconShape', 'rectangular');
sim_add_block(mdl, 'alloc', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(8, 9, 110, 60), 'FunctionName', 'sim_alloc_sfun');
sim_add_block(mdl, 'act_split', 'simulink/Signal Routing/Demux', sim_pos(9, 8, 40, 140), ...
   'Outputs', '5');
sim_add_block(mdl, 'u_mux', 'simulink/Signal Routing/Mux', sim_pos(10, 8, 30, 150), ...
   'Inputs', '6');
sim_add_plant_subsystem(mdl, 'plant', sim_pos(11, 8, 150, 150));

% ---------------- outputs ----------------
sim_add_block(mdl, 'x_state', 'simulink/Sinks/Out1', sim_pos(12, 8, 30, 14), 'Port', '1');
sim_add_block(mdl, 'u_act', 'simulink/Sinks/Out1', sim_pos(12, 9, 30, 14), 'Port', '2');
sim_add_block(mdl, 'cmd', 'simulink/Sinks/Out1', sim_pos(12, 10, 30, 14), 'Port', '3');
sim_add_block(mdl, 'rate_cmd', 'simulink/Sinks/Out1', sim_pos(12, 11, 30, 14), 'Port', '4');
sim_add_block(mdl, 'M_cmd', 'simulink/Sinks/Out1', sim_pos(12, 12, 30, 14), 'Port', '5');
sim_add_block(mdl, 'T_cmd', 'simulink/Sinks/Out1', sim_pos(12, 13, 30, 14), 'Port', '6');

% ---------------- wiring ----------------
if slider_stick
   stick_out = 'stick_mux/1';
   sim_connect(mdl, 'stick_one/1', 'stick_roll/1', 'stick_one/1', 'stick_pitch/1', ...
      'stick_one/1', 'stick_yaw/1', 'stick_one/1', 'stick_thr/1');
   sim_connect(mdl, 'stick_roll/1', 'stick_mux/1', 'stick_pitch/1', 'stick_mux/2', ...
      'stick_yaw/1', 'stick_mux/3', 'stick_thr/1', 'stick_mux/4');
else
   stick_out = 'stick_src/1';
end
sim_connect(mdl, stick_out, 'cmd_pilot/1', 'cmd_pilot/1', 'cmd_split/1', ...
   'cmd_pilot/1', 'cmd/1');
sim_connect(mdl, 'cmd_split/1', 'phi_err/1', 'cmd_split/2', 'theta_switch/3', ...
   'cmd_split/3', 'r_err/1', 'cmd_split/3', 'rate_mux/3', ...
   'cmd_split/4', 'V_err/1', 'cmd_split/5', 'h_err/1');
sim_connect(mdl, mode_out, 'theta_switch/2', 'theta_lim/1', 'theta_switch/1', ...
   'theta_switch/1', 'theta_err/1');
% The attitude loops feed the rate loops THROUGH the clamps, so a large attitude error
% can only ask for a rate the airframe can actually deliver, and the reported rate
% command is the clamped one.
sim_connect(mdl, 'phi_err/1', 'pid_phi/1', 'pid_phi/1', 'p_c_lim/1', ...
   'p_c_lim/1', 'p_err/1', 'p_c_lim/1', 'rate_mux/1', 'p_err/1', 'pid_p/1', ...
   'pid_p/1', 'da_sum/1');
sim_connect(mdl, 'theta_err/1', 'pid_theta/1', 'pid_theta/1', 'q_c_lim/1', ...
   'q_c_lim/1', 'q_err/1', 'q_c_lim/1', 'rate_mux/2', 'q_err/1', 'pid_q/1', ...
   'pid_q/1', 'de_sum/1');
sim_connect(mdl, 'r_err/1', 'r_c_lim/1', 'r_c_lim/1', 'pid_r/1', 'pid_r/1', 'dr_sum/1');
sim_connect(mdl, 'V_err/1', 'pid_V/1', 'pid_V/1', 'T_sum/1');
sim_connect(mdl, 'h_err/1', 'pid_h/1', 'pid_h/1', 'theta_lim/1');
sim_connect(mdl, 'rate_mux/1', 'rate_cmd/1');
sim_connect(mdl, 'const_utrim/1', 'utrim_split/1');
sim_connect(mdl, 'utrim_split/1', 'da_sum/2', 'utrim_split/2', 'de_sum/2', ...
   'utrim_split/3', 'dr_sum/2');
sim_connect(mdl, 'da_sum/1', 'surf_mux/1', 'de_sum/1', 'surf_mux/2', 'dr_sum/1', 'surf_mux/3');
sim_connect(mdl, 'surf_mux/1', 'G_gain/1', 'G_gain/1', 'alloc/1', 'G_gain/1', 'M_cmd/1');
sim_connect(mdl, 'const_Ttrim/1', 'T_sum/2', 'T_sum/1', 'alloc/2', 'T_sum/1', 'T_cmd/1');
sim_connect(mdl, 'alloc/1', 'act_split/1', 'alloc/1', 'u_act/1');
sim_connect(mdl, 'act_split/1', 'u_mux/1', 'act_split/2', 'u_mux/2', ...
   'act_split/3', 'u_mux/3', 'act_split/4', 'u_mux/4', ...
   'act_split/5', 'u_mux/5', 'const_zero/1', 'u_mux/6');
sim_connect(mdl, 'u_mux/1', 'plant/1', wind_out, 'plant/2', ...
   'plant/1', 'x_state/1', 'plant/1', 'x_split/1', 'plant/1', 'meas/1');
sim_connect(mdl, 'meas/1', 'meas_split/1', 'meas_split/1', 'V_err/2');
sim_connect(mdl, 'x_split/4', 'p_err/2', 'x_split/5', 'q_err/2', ...
   'x_split/6', 'r_err/2', 'x_split/7', 'phi_err/2', ...
   'x_split/8', 'theta_err/2', 'x_split/12', 'h_gain/1', 'h_gain/1', 'h_err/2');

% ---------------- interactive instrumentation ----------------
% Everything here is an output tap: nothing changes the controller, the allocator or the
% plant, and the same signals already reach the Outports for the scripted scenarios.
% The validated models are built with instrument off.
%
% Two choices come straight from watching somebody use the first version and not being
% able to tell what it was showing:
%   - every angle and every rate is converted to degrees. A raw radian reading is
%     unreadable while a slider is being dragged, and "0.0873" tells nobody anything.
%   - each scope is a COMMAND AGAINST RESPONSE bus, not a bus of raw states. The whole
%     point of this bench is to watch a loop track its command, and a scope of unaided
%     states cannot show that.
if opts.instrument
   deg_taps = {'phi_deg', 'x_split/7', 'phi 实际滚转角 [deg]'; ...
               'theta_deg', 'x_split/8', 'theta 实际俯仰角 [deg]'; ...
               'p_deg', 'x_split/4', 'p 实际滚转角速度 [deg/s]'; ...
               'q_deg', 'x_split/5', 'q 实际俯仰角速度 [deg/s]'; ...
               'r_deg', 'x_split/6', 'r 实际偏航角速度 [deg/s]'; ...
               'pc_deg', 'p_c_lim/1', 'p_c 速率指令 限幅后 [deg/s]'; ...
               'qc_deg', 'q_c_lim/1', 'q_c 速率指令 限幅后 [deg/s]'; ...
               'rc_deg', 'r_c_lim/1', 'r_c 速率指令 限幅后 [deg/s]'; ...
               'da_deg', 'act_split/1', 'da 副翼 [deg]'; ...
               'de_deg', 'act_split/2', 'de 升降舵 [deg]'; ...
               'dr_deg', 'act_split/3', 'dr 方向舵 [deg]'};
   for kt = 1:size(deg_taps, 1)
      sim_add_block(mdl, deg_taps{kt, 1}, 'simulink/Math Operations/Gain', ...
         sim_pos(1 + kt, 15, 55, 26), 'Gain', '180/pi');
      sim_connect(mdl, deg_taps{kt, 2}, [deg_taps{kt, 1} '/1']);
   end

   % Displays, in engineering units, named so the number under the block says what it is
   % without opening anything. A block NAME cannot contain a slash, because the slash is
   % the path separator: "p 滚转率 deg/s" makes Simulink try to add a block called
   % "deg" inside a block called "p 滚转率 deg", and it fails with "cannot be added".
   % So the units are spelled dps and mps here. The SIGNAL LINE names, which are labels
   % rather than paths, keep the readable deg/s and m/s.
   disp_taps = {'phi 滚转角 deg', 'phi_deg/1'; ...
                'theta 俯仰角 deg', 'theta_deg/1'; ...
                'p 滚转率 dps', 'p_deg/1'; ...
                'q 俯仰率 dps', 'q_deg/1'; ...
                'r 偏航率 dps', 'r_deg/1'; ...
                'p_c 指令 dps', 'pc_deg/1'; ...
                'q_c 指令 dps', 'qc_deg/1'; ...
                'r_c 指令 dps', 'rc_deg/1'; ...
                'V 空速 mps', 'meas_split/1'; ...
                'h 高度 m', 'h_gain/1'; ...
                'da 副翼 deg', 'da_deg/1'; ...
                'de 升降舵 deg', 'de_deg/1'; ...
                'dr 方向舵 deg', 'dr_deg/1'; ...
                '油门 左', 'act_split/4'; ...
                '油门 右', 'act_split/5'};
   for kt = 1:size(disp_taps, 1)
      sim_add_block(mdl, disp_taps{kt, 1}, 'simulink/Sinks/Display', ...
         sim_pos(1 + kt, 17, 80, 24));
      sim_connect(mdl, disp_taps{kt, 2}, [disp_taps{kt, 1} '/1']);
   end

   % Three scopes. The line names are what the Scope legend shows, so each trace is
   % labelled with the command or the response it is, and its unit.
   sim_add_block(mdl, 'track_mux', 'simulink/Signal Routing/Mux', sim_pos(2, 20, 20, 170), ...
      'Inputs', '8');
   track = {'cmd_split/1', 'phi_c 滚转指令 [rad]'; ...
            'x_split/7', 'phi 实际滚转 [rad]'; ...
            'theta_switch/1', 'theta_c 俯仰指令 [rad]'; ...
            'x_split/8', 'theta 实际俯仰 [rad]'; ...
            'cmd_split/4', 'V_c 空速指令 [m/s]'; ...
            'meas_split/1', 'V 实际空速 [m/s]'; ...
            'cmd_split/5', 'h_c 高度指令 [m]'; ...
            'h_gain/1', 'h 实际高度 [m]'};
   for kt = 1:size(track, 1)
      sim_connect(mdl, track{kt, 1}, sprintf('track_mux/%d', kt));
      sim_name_line(mdl, track{kt, 1}, track{kt, 2});
   end
   sim_add_block(mdl, 'track_scope', 'simulink/Sinks/Scope', sim_pos(3, 21, 70, 60));
   sim_connect(mdl, 'track_mux/1', 'track_scope/1');

   sim_add_block(mdl, 'rate_scope_mux', 'simulink/Signal Routing/Mux', ...
      sim_pos(12, 20, 20, 130), 'Inputs', '6');
   rates = {'pc_deg/1', 'p_c 滚转率指令 [deg/s]'; ...
            'p_deg/1', 'p 实际滚转率 [deg/s]'; ...
            'qc_deg/1', 'q_c 俯仰率指令 [deg/s]'; ...
            'q_deg/1', 'q 实际俯仰率 [deg/s]'; ...
            'rc_deg/1', 'r_c 偏航率指令 [deg/s]'; ...
            'r_deg/1', 'r 实际偏航率 [deg/s]'};
   for kt = 1:size(rates, 1)
      sim_connect(mdl, rates{kt, 1}, sprintf('rate_scope_mux/%d', kt));
      sim_name_line(mdl, rates{kt, 1}, rates{kt, 2});
   end
   sim_add_block(mdl, 'rate_scope', 'simulink/Sinks/Scope', sim_pos(13, 21, 70, 70));
   sim_connect(mdl, 'rate_scope_mux/1', 'rate_scope/1');

   sim_add_block(mdl, 'act_scope_mux', 'simulink/Signal Routing/Mux', ...
      sim_pos(20, 20, 20, 110), 'Inputs', '5');
   acts = {'da_deg/1', 'da 副翼 [deg]'; ...
           'de_deg/1', 'de 升降舵 [deg]'; ...
           'dr_deg/1', 'dr 方向舵 [deg]'; ...
           'act_split/4', '油门 左 0..1'; ...
           'act_split/5', '油门 右 0..1'};
   for kt = 1:size(acts, 1)
      sim_connect(mdl, acts{kt, 1}, sprintf('act_scope_mux/%d', kt));
      sim_name_line(mdl, acts{kt, 1}, acts{kt, 2});
   end
   sim_add_block(mdl, 'act_scope', 'simulink/Sinks/Scope', sim_pos(21, 21, 70, 60));
   sim_connect(mdl, 'act_scope_mux/1', 'act_scope/1');

   % Notes on the canvas. A bench nobody can interpret is a bench nobody uses, so the
   % purpose, the flying instructions and the pass criteria live on the diagram rather
   % than only in a document.
   local_note(mdl, 'note_purpose', [30 30 470 180], sprintf([ ...
      '虚拟操作杆台架\n\n' ...
      '这里面跑的就是受验的那套 PID 级联，接线与 model/uacs_manual_pid.slx 出自同一个生成' ...
      '器，不是副本。\n\n' ...
      '怎么飞：按 Run 开始，然后用鼠标拖动左边四个滑杆。滑杆在仿真运行中是活的，' ...
      '边跑边拖就改指令。\n\n' ...
      '仿真时长：工具栏 StopTime 框里的 sim_t_end。要改就改 uacs_go.m 第 1.9 节的 ' ...
      'uacs_sim.t_end_s，跑一次它再回来按 Run；也可以直接把那个框里的内容改成数字。']));
   local_note(mdl, 'note_watch', [500 30 980 250], sprintf([ ...
      '看什么，以及什么算好\n\n' ...
      '1. track_scope 指令对响应。拖滚转杆或俯仰杆，指令线先动、响应线跟着追。' ...
      '两者在 1 秒内贴合、松开后不回摆，就是 V6 判据里的跟踪合格。\n\n' ...
      '2. rate_scope 速率指令与限幅。这一条最值得看：把滚转杆一次拖到底，' ...
      'p_c 会在 +-0.4776 rad/s（27.4 deg/s）处变平，而 p 追不上那么快。' ...
      '那道平线就是速率指令限幅器在工作，它是当初把飞机从翻转里救回来的东西。\n\n' ...
      '3. act_scope 舵面与油门。看副翼随滚转杆反向、升降舵随俯仰杆反向、' ...
      '油门左右同步变化。油门分叉说明差动推力在工作。\n\n' ...
      '4. 满杆持续拉到底是一种故障注入：飞机应该收敛、侧滑有界、不掉高失控，' ...
      '而不是进入螺旋。']));
   local_note(mdl, 'note_mode', [500 270 980 130], sprintf([ ...
      '飞行模式切换\n\n' ...
      '图中部 mode_sw 是一个手动开关，仿真运行中双击它就能切换：\n' ...
      '  接 0 一路 = 姿态模式，俯仰杆直接给 theta 指令，高度由你自己配平；\n' ...
      '  接 1 一路 = 高度模式，俯仰杆变成爬升率指令，由 pid_h 环管住高度。\n\n' ...
      '切到高度模式后再拉杆，可以看到 h 缓慢跟上一个新高度，而 theta 被 pid_h 当作' ...
      '中间指令使用。']));

   % Name each outport's input line after its block. sim_run_model does this at run time
   % for the scripted jobs, but a person pressing Run gets out1..out6 and cannot look a
   % signal up by name. Exported labels follow the SIGNAL LINE name, not the Outport
   % block name, which is why the line is renamed and not just the block.
   ops = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Outport');
   for ko = 1:numel(ops)
      ph = get_param(ops{ko}, 'PortHandles');
      if ~isempty(ph.Inport) && ph.Inport > 0
         lh = get_param(ph.Inport, 'Line');
         if lh > 0
            set_param(lh, 'Name', get_param(ops{ko}, 'Name'));
         end
      end
   end
end

% PreLoadFcn initialises the base workspace when the model is opened, which is what makes
% a double-click on the .slx work. InitFcn is deliberately the LIGHT callback: pressing
% Run repeatedly must not re-run the whole parameter chain and its几分钟 of prints, so it
% only initialises when the base workspace is actually missing something.
if ~isempty(opts.reload_fcn)
   set_param(mdl, 'PreLoadFcn', opts.reload_fcn);
end
if ~isempty(opts.ensure_fcn)
   set_param(mdl, 'InitFcn', opts.ensure_fcn);
elseif ~isempty(opts.reload_fcn)
   set_param(mdl, 'InitFcn', opts.reload_fcn);
end

save_system(mdl, file);
info = struct('model', mdl, 'file', file);
info.census = sim_census(mdl);
info.opts = opts;
end

% ------------------------------------------------------------------------
function local_note(mdl, name, pos, txt)
% Put a text note on the diagram. Simulink.Annotation is the documented object; a
% 'built-in/Note' block would put a block-shaped element in the block census, which V11
% compares against a fixed count.
a = Simulink.Annotation([mdl '/' name]);
a.Text = txt;
a.Position = pos;
end
