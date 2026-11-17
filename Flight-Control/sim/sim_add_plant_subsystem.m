function info = sim_add_plant_subsystem(mdl, name, pos)
% SIM_ADD_PLANT_SUBSYSTEM Put the 12-state plant inside a controller model
%
%   info = sim_add_plant_subsystem(mdl, name, pos)
%
% The plant is embedded as a copy, not a model reference

sub = [mdl '/' name];
sim_add_block(mdl, name, 'simulink/Ports & Subsystems/Subsystem', pos);
sim_clear_model(sub);
sim_add_block(sub, 'u_act', 'simulink/Sources/In1', sim_pos(0, 1, 30, 14), 'Port', '1');
sim_add_block(sub, 'wind_ned', 'simulink/Sources/In1', sim_pos(0, 2, 30, 14), 'Port', '2');
sim_add_block(sub, 'plant_eom', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(1, 0, 150, 130), 'FunctionName', 'sim_plant_sfun');
sim_add_block(sub, 'x_state', 'simulink/Sinks/Out1', sim_pos(3, 1, 30, 14), 'Port', '1');
sim_connect(sub, 'u_act/1', 'plant_eom/1', 'wind_ned/1', 'plant_eom/2', ...
   'plant_eom/1', 'x_state/1');
info = struct('name', name, 'path', sub);
end
