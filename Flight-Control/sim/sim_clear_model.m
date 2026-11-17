function n = sim_clear_model(mdl)
% SIM_CLEAR_MODEL Remove every line and every block inside a model or subsystem
%
%   n = sim_clear_model(mdl)   number of blocks deleted

% find_system options must precede parameter name/value pairs, or they are read as block parameters
lns = find_system(mdl, 'FindAll', 'on', 'LookUnderMasks', 'all', ...
   'FollowLinks', 'on', 'Type', 'line');
% one call for the whole array: the first deletion invalidates the remaining handles
if ~isempty(lns)
   delete_line(lns);
end

blks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
% keep only true descendants, so the container survives
keep = ~strcmp(blks, mdl);
blks = blks(keep);
if ~isempty(blks)
   delete_block(blks);
end
n = numel(blks);
end
