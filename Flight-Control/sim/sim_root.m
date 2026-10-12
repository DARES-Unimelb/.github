function root = sim_root()
% SIM_ROOT Project MATLAB root, derived from this file's own location.
%
% The Simulink helpers live in <root>/sim, so the root is one level up. Scripts
% use it instead of the caller's pwd, because the MCP runner leaves the working
% directory wherever the last script was.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
end
