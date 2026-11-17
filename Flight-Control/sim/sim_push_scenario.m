function S = sim_push_scenario(S)
% SIM_PUSH_SCENARIO Put the scenario inputs of one run into the base workspace
%
%   S   scenario struct, all fields optional except t_end; returned filled in
%   t_end        s, stop time of the run, default P.sim.t_end_s
%   x0           12x1 initial state, default from sim_plant_x0
%   wind         [t wn we wd] matrix, default still air
%   stick        [t roll pitch yaw throttle] matrix
%   ref          [t dV_c dtheta_c dphi_c dbeta_c] matrix
%   flight_mode  0 attitude mode, 1 altitude mode (the pitch channel owner)
%
% The three time series are plain matrices in From Workspace order, time in
% column 1

if ~isfield(S, 't_end') || isempty(S.t_end)
   Pb = evalin('base', 'P');
   S.t_end = Pb.sim.t_end_s;
end
if ~isfield(S, 'x0') || isempty(S.x0)
   S.x0 = sim_plant_x0();
end
if ~isfield(S, 'wind') || isempty(S.wind)
   S.wind = [0, 0, 0, 0];
end
if ~isfield(S, 'stick') || isempty(S.stick)
   S.stick = [0, 0, 0, 0, 0; S.t_end, 0, 0, 0, 0];
end
if ~isfield(S, 'ref') || isempty(S.ref)
   S.ref = [0, 0, 0, 0, 0; S.t_end, 0, 0, 0, 0];
end
if ~isfield(S, 'flight_mode') || isempty(S.flight_mode)
   S.flight_mode = 0;
end

assignin('base', 'sim_x0', S.x0(:));
% The plant wind port is 3 elements wide, so the signal is published as a
% timeseries of the three NED components and never as the raw 4-column matrix
wind = S.wind;
if isempty(wind)
   wind = [0, 0, 0, 0];
end
if size(wind, 2) < 4
   error('sim_push_scenario:WindShape', ...
      'wind must be [t wn we wd], got %d columns', size(wind, 2));
end
wind = wind(:, 1:4);
sim_wind_t = wind(:, 1);
sim_wind_ned = timeseries(wind(:, 2:4), sim_wind_t);
assignin('base', 'sim_wind_t', sim_wind_t);
assignin('base', 'sim_wind_ned', sim_wind_ned);
% legacy alias of sim_wind_ned: same 3-component signal, no time column
assignin('base', 'sim_wind_ts', sim_wind_ned);
assignin('base', 'sim_stick_ts', S.stick);
assignin('base', 'sim_lqr_ref_ts', S.ref);
assignin('base', 'sim_flight_mode', S.flight_mode);
assignin('base', 'sim_t_end', S.t_end);
end
