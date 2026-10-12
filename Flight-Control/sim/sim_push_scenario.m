function S = sim_push_scenario(S)
% SIM_PUSH_SCENARIO Put the scenario inputs of one run into the base workspace.
%
%   S = sim_push_scenario(S)
%
% Fields, all optional except t_end
%   t_end        s, stop time of the run
%   x0           12x1 initial state, default trim.x_star
%   wind         [t, wn, we, wd] matrix for the wind From Workspace block
%   stick        [t, roll, pitch, yaw, throttle] matrix for the stick source
%   ref          [t, dV_c, dtheta_c, dphi_c, dbeta_c] matrix for the LQR reference
%   flight_mode  0 attitude mode, 1 altitude mode (the pitch channel owner)
%
% The three time series are plain matrices, which is the From Workspace format
% with the time in column 1 and the signals in the remaining columns. They are
% rebuilt for every run, so no scenario can inherit the previous one's inputs.

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
% The plant's wind port is 3 components wide (NED), which is what sim_plant_sfun
% declares and what model/uacs_plant.slx exposes. Scenarios still pass a 4-column
% [t N E D] matrix, so the time column is kept and the first two wind components
% are taken, dropping the down component rather than feeding a 4-wide signal to a
% 3-wide port. Without this the model fails to compile with 'Output Port 1 of
% plant/wind_ned is a one dimensional vector with 4 elements' against 'Input Port
% 2 of plant/plant_eom is a one dimensional vector with 3 elements'.
% The wind matrix is [t wn we wd]: column 1 is TIME, columns 2 to 4 are the three NED
% components. The plant's wind port is 3 elements wide because it carries the wind
% VECTOR, so the time column must not be part of the signal.
%
% This was wrong twice, and both times for the same reason: treating the time column as
% one of the signals. The first version passed the whole 4 column matrix, so a time
% column arrived at a 3 wide port and the model failed to compile. The second version
% "fixed" the width by taking columns 1 to 3, which kept the TIME column as a signal
% and silently DROPPED the down component.
%
% The fix is not a choice of which columns to keep, it is to stop representing time as
% data: the signal is published as a timeseries, which carries its own time vector.
% A raw matrix cannot do this unambiguously. From Workspace reads a matrix in one of
% two ways depending on whether SampleTime is set: with SampleTime unset it treats
% column 1 as time and the rest as signals, with SampleTime set every column is a
% signal. That ambiguity is exactly what produced the wrong width twice.
%
% The down component is kept, not sacrificed. The three vertical wind cases are
% distinguishable and must not be collapsed: with W_d = 0 the air relative velocity
% equals the ground velocity on the vertical axis, so a scenario that drops W_d cannot
% tell a downdraught from level air.
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
% Legacy alias for any consumer still reading the old name. It holds the same 3
% component signal, WITHOUT a time column, which is what the port wants.
assignin('base', 'sim_wind_ts', sim_wind_ned);
assignin('base', 'sim_stick_ts', S.stick);
assignin('base', 'sim_lqr_ref_ts', S.ref);
assignin('base', 'sim_flight_mode', S.flight_mode);
assignin('base', 'sim_t_end', S.t_end);
end
