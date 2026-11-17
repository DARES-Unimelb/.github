function n = sim_push_vars(S)
% SIM_PUSH_VARS Copy a struct's fields into the base workspace
%
%   S   struct whose field names become base workspace variables
%   n   number of variables written

f = fieldnames(S);
for k = 1:numel(f)
   assignin('base', f{k}, S.(f{k}));
end
n = numel(f);
end
