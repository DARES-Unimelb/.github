function info = sim_build_plant_ref(P, opts)
% SIM_BUILD_PLANT_REF Build and save model/uacs_plant_ref.slx
%
% Integration and kinematics come from the Aerospace Blockset 6DOF (Euler Angles)
% block and everything else is ours, so the trajectories are comparable element by
% element with uacs_plant.slx, which has the same interface
%
%   in  1  u_act     6x1  [da de dr dth_l dth_r dT]
%   in  2  wind_ned  3x1  steady wind, NED, m/s
%   out 1  x_state   12x1 [u v w p q r phi theta psi pn pe pd]
%   out 2  aux       11x1 [rho V alpha beta qbar F_body' M_body']
%
% 6DOF output mapping, which cannot be read off the port numbers
%   our x(1:3)   <- 6DOF out 5  Vb          body velocity, m/s
%   our x(4:6)   <- 6DOF out 6  p q r       body rates, rad/s
%   our x(7:9)   <- 6DOF out 3  phi theta psi
%   our x(10:12) <- 6DOF out 2  Xe          inertial position, NED, m
% 6DOF out 1 Ve and out 4 DCMbe hold the same information in other coordinates and
% are left unconnected
%
% Block parameters are workspace variable names, never numbers; sim_push_ref_vars
% is what puts them in base

if nargin < 2
   opts = struct();
end
mdl = 'uacs_plant_ref';
file = fullfile(sim_root(), 'model', [mdl '.slx']);

% defaults so the model runs right after a build; a scenario overwrites them
if evalin('base', 'exist(''ref_x0_vb'', ''var'')') == 0
   sim_push_ref_vars(P, sim_plant_x0(), 0);
end

sim_new_model(mdl, file);

%% Sources
sim_add_block(mdl, 'u_act', 'simulink/Sources/In1', sim_pos(0, 1, 30, 14), 'Port', '1');
sim_add_block(mdl, 'wind_ned', 'simulink/Sources/In1', sim_pos(0, 2, 30, 14), 'Port', '2');
sim_add_block(mdl, 'h_isa', 'simulink/Sources/Constant', sim_pos(0, 5, 80, 30), ...
   'Value', 'ref_h_isa');
sim_add_block(mdl, 'h_mode', 'simulink/Sources/Constant', sim_pos(0, 7, 80, 30), ...
   'Value', 'ref_h_from_state');

%% Third party motion
sim_add_block(mdl, 'eom', 'aerolib6dof/6DoF (Euler Angles)', sim_pos(4, 1, 150, 150), ...
   'xme_0', 'ref_x0_pos', 'Vm_0', 'ref_x0_vb', 'eul_0', 'ref_x0_eul', ...
   'pm_0', 'ref_x0_om', 'Mass', 'ref_mass', 'Inertia', 'ref_inertia');
sim_add_block(mdl, 'isa', 'aerolibatmos/ISA Atmosphere Model', sim_pos(2, 5, 110, 60));

% ISA is evaluated at the frozen altitude ref_h_isa or at the geometric altitude
% -Xe(3), which is what the aircraft is really at; ref_h_from_state selects
% The switch threshold 0.5 sits between the two legal values 0 and 1
sim_add_block(mdl, 'xe_split', 'simulink/Signal Routing/Demux', sim_pos(2, 3, 40, 90), ...
   'Outputs', '3');
sim_add_block(mdl, 'h_state_gain', 'simulink/Math Operations/Gain', sim_pos(3, 3, 60, 30), ...
   'Gain', '-1');
sim_add_block(mdl, 'h_switch', 'simulink/Signal Routing/Switch', sim_pos(4, 5, 60, 40), ...
   'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');

%% Our aerodynamics
sim_add_block(mdl, 'aero', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(2, 7, 160, 130), 'FunctionName', 'sim_plant_ref_aero');
sim_add_block(mdl, 'x_mux', 'simulink/Signal Routing/Mux', sim_pos(6, 1, 20, 110), 'Inputs', '4');
sim_add_block(mdl, 'x_state', 'simulink/Sinks/Out1', sim_pos(7, 1, 30, 14), 'Port', '1');
sim_add_block(mdl, 'aux', 'simulink/Sinks/Out1', sim_pos(3, 7, 30, 14), 'Port', '2');

%% Wiring
% the aero block and the 6DOF block feed each other, which is no algebraic loop:
% the 6DOF outputs are integrator states
sim_connect(mdl, 'h_switch/1', 'isa/1');
sim_connect(mdl, 'h_isa/1', 'h_switch/3', 'h_mode/1', 'h_switch/2');
sim_connect(mdl, 'eom/2', 'xe_split/1', 'xe_split/3', 'h_state_gain/1', ...
   'h_state_gain/1', 'h_switch/1');
sim_connect(mdl, 'eom/5', 'aero/1', 'eom/6', 'aero/2', 'eom/3', 'aero/3', ...
   'wind_ned/1', 'aero/4', 'u_act/1', 'aero/5', 'isa/4', 'aero/6');
sim_connect(mdl, 'aero/1', 'eom/1', 'aero/2', 'eom/2');
sim_connect(mdl, 'eom/5', 'x_mux/1', 'eom/6', 'x_mux/2', 'eom/3', 'x_mux/3', 'eom/2', 'x_mux/4');
sim_connect(mdl, 'x_mux/1', 'x_state/1', 'aero/3', 'aux/1');

save_system(mdl, file);
info = struct('model', mdl, 'file', file);
info.census = sim_census(mdl);
info.opts = opts;
local_register_census(info);
end

%% Census registration
function local_register_census(info)
% merge this model into results/sim_build_census.mat, leaving the other models'
% entries in the file untouched
file = fullfile(sim_root(), 'results', 'sim_build_census.mat');
S = struct();
if exist(file, 'file')
   S = load(file);
end
if ~isfield(S, 'census') || ~isstruct(S.census)
   S.census = struct();
end
c = info.census;
e = struct();
e.name = info.model;
e.file = info.file;
e.top_count = c.top_count;
e.total_count = c.total_count;
e.line_count = c.line_count;
e.top_names = c.top_names;
e.inports = c.num_inports;
e.outports = c.num_outports;
S.census.(info.model) = e;
if ~isfield(S.census, 'stamp')
   S.census.stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end
if ~isfield(S.census, 'blk_kind')
   S.census.blk_kind = 'sfun';
end
save(file, '-struct', 'S');
end
