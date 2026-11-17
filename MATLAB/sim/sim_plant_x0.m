function x0 = sim_plant_x0()
% SIM_PLANT_X0 Initial state of the plant, read from the base workspace.
%
% Order of preference, so a scenario always says what it starts from:
%   sim_x0     the scenario's own initial state, when it sets one
%   trim.x_star the trim state, so an open-loop run starts in equilibrium
% Anything else is an error: a silent zero start would drop the aircraft.

if evalin('base', 'exist(''sim_x0'', ''var'')') == 1
   x0 = evalin('base', 'sim_x0');
elseif evalin('base', 'exist(''trim'', ''var'')') == 1
   x0 = evalin('base', 'trim.x_star');
else
   error('sim_plant_x0:NoInitialState', ...
      'set sim_x0 or a trim struct in the base workspace before simulating');
end
x0 = x0(:);
assert(numel(x0) == 12, 'sim_plant_x0:Size', 'initial state must have 12 elements');
end
