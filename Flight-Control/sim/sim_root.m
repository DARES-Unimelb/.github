function root = sim_root()
% SIM_ROOT Absolute path of the project MATLAB root, one level above this file
%
%   root   project root, independent of the caller's working directory

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
end
