function src = sim_plant_source(mode)
% SIM_PLANT_SOURCE Get or set which plant right-hand side the models use
%
%   sim_plant_source('eom_6dof')      use plant/eom_6dof.m
%   sim_plant_source('placeholder')   use the local placeholder
%   sim_plant_source('auto')          decide by file existence
%   src = sim_plant_source()          read the cached decision

persistent cached
if nargin > 0 && ~isempty(mode)
   cached = char(mode);
end
if isempty(cached)
   cached = 'auto';
end
src = cached;
end
