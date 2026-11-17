function rep = sim_deps_reset()
% SIM_DEPS_RESET Forget cached dependency lookups and re-scan the path
%
%   rep = sim_deps_reset()
%
% Call after changing the MATLAB path, so new functions are picked up

clear sim_have
clear sim_plant_source
rehash path
rep = sim_deps_report();
end
