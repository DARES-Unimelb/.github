function x0 = sim_plant_x0()
% SIM_PLANT_X0 12x1 initial plant state, read from the base workspace
%
%   x0   sim_x0 when set, else trim.x_star; errors when neither exists

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
