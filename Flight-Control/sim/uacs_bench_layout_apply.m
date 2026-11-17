function rep = uacs_bench_layout_apply(mdl)
% UACS_BENCH_LAYOUT_APPLY Put the exported canvas layout back onto a rebuilt model
%
%   mdl   bench model name, already built and loaded
%   rep   applied and missing counts for blocks and for annotations
%
% The bench model is rebuilt as a whole, so positions moved by hand on the canvas
% are re-applied afterwards from uacs_bench_layout_list
%
% Only blocks that still exist are moved; a block added since the export keeps
% the position the build script gave it
%
% Annotations are matched on their text: first the whole text, then the first
% line, so an annotation whose body changed but whose title did not is still
% found; newlines in the exported data are the two characters \n and are
% restored before comparing

L = uacs_bench_layout_list();
rep = struct('applied', 0, 'missing', 0, 'notes_applied', 0, 'notes_missing', 0);

for k = 1:size(L.blocks, 1)
   blk = [mdl '/' L.blocks{k, 1}];
   if getSimulinkBlockHandle(blk) > 0
      set_param(blk, 'Position', L.blocks{k, 2});
      rep.applied = rep.applied + 1;
   else
      rep.missing = rep.missing + 1;
   end
end

notes = find_system(mdl, 'FindAll', 'on', 'Type', 'annotation');
for k = 1:size(L.notes, 1)
   want = strrep(L.notes{k, 1}, '\n', sprintf('\n'));
   want_first = local_first(want);
   hit = 0;
   for j = 1:numel(notes)
      if strcmp(get_param(notes(j), 'Text'), want)
         hit = notes(j);
         break
      end
   end
   if hit == 0
      for j = 1:numel(notes)
         if strcmp(local_first(get_param(notes(j), 'Text')), want_first)
            hit = notes(j);
            break
         end
      end
   end
   if hit > 0
      set_param(hit, 'Position', L.notes{k, 2});
      rep.notes_applied = rep.notes_applied + 1;
   else
      rep.notes_missing = rep.notes_missing + 1;
   end
end
end

%% Local helpers
function s = local_first(t)
% first line of the annotation text, the whole text when it has one line
nl = find(t == sprintf('\n'), 1);
if ~isempty(nl)
   t = t(1:nl - 1);
end
s = strtrim(t);
end
