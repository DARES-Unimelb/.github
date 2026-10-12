function src = sim_plant_source(mode)
% SIM_PLANT_SOURCE Get or set which plant right-hand side the models use.
%
%   sim_plant_source('eom_6dof')      use plant/eom_6dof.m
%   sim_plant_source('placeholder')   use the local placeholder
%   sim_plant_source('auto')          decide by file existence
%   src = sim_plant_source()          read the current decision
%
% The decision is cached because the plant derivative is called on every solver
% step. sim_probe_plant makes it once, from a real call, so a plant that exists
% but throws at run time such as a throttle model still reading a retired
% parameter cannot take a simulation down in the middle of a run.

persistent cached
if nargin > 0 && ~isempty(mode)
   cached = char(mode);
end
if isempty(cached)
   cached = 'auto';
end
src = cached;
end
