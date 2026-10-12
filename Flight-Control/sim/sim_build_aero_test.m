function info = sim_build_aero_test(P, opts)
% SIM_BUILD_AERO_TEST Build and save model/uacs_aero_test.slx, the aerodynamic test page.
%
% Seven sliders, all live while the page runs:
%   V m/s, alpha deg, beta deg, altitude m, elevator deg, aileron deg, rudder deg
% and thirteen readouts taken from plant/aero_forces.m, the same function the flight
% models use: the six coefficients, the three wind-axis forces, the three body moments
% and the density. Nothing here is a second aerodynamic model; if a coefficient
% disagrees with the aircraft, the aircraft is the one that is right.
%
% Rates are held at zero and the attitude angles at alpha and zero, which is the
% steady case. Cm_alphadot therefore vanishes, because aero_forces evaluates it with
% alpha_dot = q and q is zero here.
%
% opts.reload_fcn  model callback run on open and before every simulation, so the page
%                  initialises P in the base workspace by itself

if nargin < 2
   opts = struct();
end
if ~isfield(opts, 'reload_fcn') || isempty(opts.reload_fcn)
   opts.reload_fcn = '';
end
if ~isfield(opts, 'ensure_fcn') || isempty(opts.ensure_fcn)
   opts.ensure_fcn = '';
end
mdl = 'uacs_aero_test';
file = fullfile(sim_root(), 'model', [mdl '.slx']);
sim_new_model(mdl, file);

% ---------------- sliders ----------------
% A Slider Gain multiplies its input by the slider value, so every slider is fed a
% Constant 1 and the slider itself is the value. Low and High set the range.
sim_add_block(mdl, 'one', 'simulink/Sources/Constant', sim_pos(0, 0, 50, 30), 'Value', 'sim_c_one');
spec = { ...
   's_v',     1,  '5',   '35',  '20',    'V 空速 m/s'; ...
   's_alpha', 3,  '-5',  '20',  '0',     'alpha 迎角 deg'; ...
   's_beta',  5,  '-15', '15',  '0',     'beta 侧滑角 deg'; ...
   's_h',     7,  '0',   '400', '150',   'h 高度 m'; ...
   's_de',    9,  '-25', '25',  '0',     'de 升降舵 deg（正 = 后缘下）'; ...
   's_da',    11, '-25', '25',  '0',     'da 副翼 deg（正 = 右副翼下）'; ...
   's_dr',    13, '-25', '25',  '0',     'dr 方向舵 deg（正 = 左偏航）'};
for k = 1:size(spec, 1)
   sim_add_block(mdl, spec{k, 1}, 'simulink/Math Operations/Slider Gain', ...
      sim_pos(spec{k, 2}, 0, 70, 40), 'Low', spec{k, 3}, 'High', spec{k, 4});
   % Gain is set in a SECOND call. Passing Low, High and Gain together leaves the order
   % of application undefined, and a Gain outside the block's current range is quietly
   % replaced by the default with only a warning: the airspeed slider then reads 1 m/s
   % instead of 20.
   set_param([mdl '/' spec{k, 1}], 'Gain', spec{k, 5});
   sim_connect(mdl, 'one/1', [spec{k, 1} '/1']);
end

% ---------------- degrees to radians ----------------
% Every angle channel is entered in degrees because that is how a person reads a trim
% table, and converted once here. All five, not just the first three: a conversion gain
% that is never created cannot be wired.
angle_spec = {'s_alpha', 4; 's_beta', 6; 's_de', 10; 's_da', 12; 's_dr', 14};
for k = 1:size(angle_spec, 1)
   sim_add_block(mdl, [angle_spec{k, 1} '_rad'], 'simulink/Math Operations/Gain', ...
      sim_pos(angle_spec{k, 2}, 0, 50, 30), 'Gain', 'pi/180');
   sim_connect(mdl, [angle_spec{k, 1} '/1'], [angle_spec{k, 1} '_rad/1']);
end

% ---------------- probe and readouts ----------------
sim_add_block(mdl, 'probe_mux', 'simulink/Signal Routing/Mux', sim_pos(2, 2, 20, 150), ...
   'Inputs', '7');
sim_connect(mdl, 's_v/1', 'probe_mux/1', 's_alpha_rad/1', 'probe_mux/2', ...
   's_beta_rad/1', 'probe_mux/3', 's_h/1', 'probe_mux/4', ...
   's_de_rad/1', 'probe_mux/5', 's_da_rad/1', 'probe_mux/6', 's_dr_rad/1', 'probe_mux/7');
sim_add_block(mdl, 'probe', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(3, 2, 110, 60), 'FunctionName', 'sim_aero_probe_sfun');
sim_connect(mdl, 'probe_mux/1', 'probe/1');
sim_add_block(mdl, 'probe_split', 'simulink/Signal Routing/Demux', sim_pos(4, 2, 40, 300), ...
   'Outputs', '13');

readouts = { ...
   'CL 升力系数'; 'CD 阻力系数'; 'CY 侧力系数'; ...
   'Cl 滚转力矩系数'; 'Cm 俯仰力矩系数'; 'Cn 偏航力矩系数'; ...
   'L 升力 N'; 'D 阻力 N'; 'Y 侧力 N'; ...
   'M_roll N*m'; 'M_pitch N*m'; 'M_yaw N*m'; 'rho kg/m^3'};
for k = 1:numel(readouts)
   sim_add_block(mdl, sprintf('d_%02d', k), 'simulink/Sinks/Display', sim_pos(5, k, 70, 24));
   sim_connect(mdl, sprintf('probe_split/%d', k), sprintf('d_%02d/1', k));
end

% A scope on the six coefficients, and a lift-drag scope that is the one a person
% actually watches while dragging alpha.
sim_add_block(mdl, 'coef_mux', 'simulink/Signal Routing/Mux', sim_pos(5, 15, 20, 130), ...
   'Inputs', '6');
sim_connect(mdl, 'probe_split/1', 'coef_mux/1', 'probe_split/2', 'coef_mux/2', ...
   'probe_split/3', 'coef_mux/3', 'probe_split/4', 'coef_mux/4', ...
   'probe_split/5', 'coef_mux/5', 'probe_split/6', 'coef_mux/6');
sim_add_block(mdl, 'coef_scope', 'simulink/Sinks/Scope', sim_pos(6, 15, 60, 40));
sim_connect(mdl, 'coef_mux/1', 'coef_scope/1');
sim_add_block(mdl, 'ld_mux', 'simulink/Signal Routing/Mux', sim_pos(5, 18, 20, 60), ...
   'Inputs', '2');
sim_connect(mdl, 'probe_split/1', 'ld_mux/1', 'probe_split/2', 'ld_mux/2');
sim_add_block(mdl, 'ld_scope', 'simulink/Sinks/Scope', sim_pos(6, 18, 60, 40));
sim_connect(mdl, 'ld_mux/1', 'ld_scope/1');

sim_add_block(mdl, 'probe_out', 'simulink/Sinks/Out1', sim_pos(5, 20, 30, 14), 'Port', '1');
sim_connect(mdl, 'probe/1', 'probe_out/1');
sim_name_line(mdl, 'probe', ['CL CD CY Cl Cm Cn L D Y M_roll M_pitch M_yaw rho']);

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
