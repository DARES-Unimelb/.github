function n = sim_clear_model(mdl)
% SIM_CLEAR_MODEL Remove every line and every block INSIDE a model or subsystem.
%
% Two rules, each learned from a real failure:
%
% 1. The container itself must be excluded from the block list. find_system with
%    SearchDepth 1 on a subsystem returns the subsystem as its own first element,
%    and delete_block on it removes the subsystem AND its children; the following
%    delete_block calls for those children then fail with
%    "Invalid Simulink object name: <sub>/In1", because by then the child is gone.
%    Only descendants are deleted here, so the container survives to be filled.
%
% 2. FindAll and the other find_system options must come BEFORE the parameter
%    name/value pairs. Supplied after them they are treated as block parameters,
%    which silently returns zero lines and makes every line deletion a no-op.
%
% Line handles are collected and handed to delete_line in ONE call, because a
% handle array is invalidated by the first deletion, so a loop would fail from the
% second element onwards.

lns = find_system(mdl, 'FindAll', 'on', 'LookUnderMasks', 'all', ...
   'FollowLinks', 'on', 'Type', 'line');
if ~isempty(lns)
   delete_line(lns);
end

blks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
% Keep only true descendants. String comparison rather than a leading slash test,
% so a model named like its own block cannot slip through.
keep = ~strcmp(blks, mdl);
blks = blks(keep);
if ~isempty(blks)
   delete_block(blks);
end
n = numel(blks);
end
