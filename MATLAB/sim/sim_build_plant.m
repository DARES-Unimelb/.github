function info = sim_build_plant(P, opts)
% SIM_BUILD_PLANT Build and save model/uacs_plant.slx.
%
% The plant model is the bare object: the 6 actuator channels and a steady wind
% in, the 12 states out. Throttle, elevator, aileron and rudder go to the
% S-Function unchanged, the sixth channel dT is left to the caller (the
% allocation path realises thrust through the two throttles).
%
%   in  1  u_act     6x1  [da de dr dth_l dth_r dT]
%   in  2  wind_ned  3x1  steady wind, NED, m/s
%   out 1  x_state   12x1 [u v w p q r phi theta psi pn pe pd]

if nargin < 2
   opts = struct();
end
mdl = 'uacs_plant';
file = fullfile(sim_root(), 'model', [mdl '.slx']);
sim_new_model(mdl, file);

sim_add_block(mdl, 'u_act', 'simulink/Sources/In1', sim_pos(0, 1, 30, 14), 'Port', '1');
sim_add_block(mdl, 'wind_ned', 'simulink/Sources/In1', sim_pos(0, 2, 30, 14), 'Port', '2');
sim_add_block(mdl, 'plant_eom', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(1, 1, 150, 130), 'FunctionName', 'sim_plant_sfun');
sim_add_block(mdl, 'x_state', 'simulink/Sinks/Out1', sim_pos(3, 1, 30, 14), 'Port', '1');
sim_connect(mdl, 'u_act/1', 'plant_eom/1', 'wind_ned/1', 'plant_eom/2', ...
   'plant_eom/1', 'x_state/1');

save_system(mdl, file);
info = struct('model', mdl, 'file', file);
info.census = sim_census(mdl);
info.opts = opts;
end
