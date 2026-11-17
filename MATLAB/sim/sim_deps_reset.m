function rep = sim_deps_reset()
% SIM_DEPS_RESET Forget cached dependency lookups and re-scan the path.
%
% Every job calls this after init_uacs_twin, so a function that appeared since
% the last run is picked up instead of a stale cached answer.

clear sim_have
clear sim_plant_source
rehash path
rep = sim_deps_report();
end
