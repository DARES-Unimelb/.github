function C = sim_census(mdl)
% SIM_CENSUS Block inventory of a model
%
%   C = sim_census(mdl)
%   C.top_count    blocks at the top level of the model
%   C.total_count  blocks including every subsystem level
%   C.top_names    sorted top level block names
%   C.all_names    sorted full paths, so a moved or renamed block is visible
%   C.line_count   line segments
%   C.num_inports  root inports
%   C.num_outports root outports

C = struct();
top = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
allb = find_system(mdl, 'Type', 'Block');
C.top_count = numel(top);
C.total_count = numel(allb);
C.top_names = sort(local_tail(top));
C.all_names = sort(allb);
C.line_count = numel(find_system(mdl, 'FindAll', 'on', 'Type', 'line'));
C.num_inports = numel(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport'));
C.num_outports = numel(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Outport'));
C.checked = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end

function t = local_tail(paths)
t = cell(size(paths));
for k = 1:numel(paths)
   parts = strsplit(paths{k}, '/');
   t{k} = parts{end};
end
end
