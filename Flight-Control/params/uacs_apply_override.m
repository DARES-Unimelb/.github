function s = uacs_apply_override(group, s)
% UACS_APPLY_OVERRIDE Replace parameter values from the pending override table.
%
%   s = uacs_apply_override('geom', geom)
%
% Called from uacs_twin_params.m where a group's literals end and its derivation
% begins, so the derivation below recomputes from the overridden numbers
%
%   pairs are {dotted path, value} installed by init_uacs_twin through
%   setappdata; recursion reaches nested paths such as ctrl.bw.rate_p_rad_s

pairs = getappdata(0, 'uacs_override_pairs');
if isempty(pairs)
   return;
end
f = fieldnames(s);
for k = 1:numel(f)
   path = [group '.' f{k}];
   idx = find(strcmp(pairs(:, 1), path), 1);
   if ~isempty(idx)
      s.(f{k}) = pairs{idx, 2};
      % Record that this path was really replaced; init_uacs_twin asserts that
      % every override was applied somewhere
      done = getappdata(0, 'uacs_override_applied');
      done{end + 1} = path;
      setappdata(0, 'uacs_override_applied', done);
   elseif isstruct(s.(f{k})) && numel(s.(f{k})) == 1
      s.(f{k}) = uacs_apply_override(path, s.(f{k}));
   end
end
end
