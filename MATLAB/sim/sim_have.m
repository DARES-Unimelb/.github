function tf = sim_have(fname)
% SIM_HAVE Cached existence test for a function on the MATLAB path.
%
% The Simulink blocks call this on every sample, so the answer is cached in a
% persistent variable. sim_deps_reset clears that cache after a path change.
% Only a real file counts: a variable with the same name is not a function.

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
