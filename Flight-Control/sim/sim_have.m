function tf = sim_have(fname)
% SIM_HAVE Cached existence test for a function on the MATLAB path
%
% The answer is cached in a persistent variable, so call sim_deps_reset after
% changing the path. Only a real file counts, not a variable of the same name

persistent cache
if isempty(cache)
   cache = struct();
end
key = matlab.lang.makeValidName(char(fname));
if ~isfield(cache, key)
   cache.(key) = (exist(char(fname), 'file') == 2);
end
tf = cache.(key);
end
