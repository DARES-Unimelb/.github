function s = uacs_apply_override(group, s)
% UACS_APPLY_OVERRIDE Replace parameter values from the pending override table.
%
%   s = uacs_apply_override('geom', geom)
%
% Called from uacs_twin_params.m at every point where a group's literals end and its
% derivation begins. Applying an override AFTER the parameter file has returned would
% leave every dependent quantity describing the old aircraft: change the span and the
% area, the chord, the tail volume and six aerodynamic derivatives all stay at their
% old values, and the only thing that catches it is an assertion. Applying it here
% means the derivation lines below recompute from the overridden numbers.
%
% The table is an Nx2 cell of {dotted path, value} installed by init_uacs_twin through
% setappdata on the root object, and removed as soon as the parameter file has run, so
% no later caller can inherit it. Recursion handles nested paths, so a group call
% placed once still reaches ctrl.bw.rate_p_rad_s.
%
% A path that matches nothing is simply not applied here. init_uacs_twin checks every
% path against the finished P and raises on an unknown one, so a typo cannot pass.

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
      % Record that this path was really replaced. init_uacs_twin asserts that every
      % override was applied somewhere, so a group that carries no call is a loud
      % error rather than a knob that silently does nothing.
      done = getappdata(0, 'uacs_override_applied');
      done{end + 1} = path;
      setappdata(0, 'uacs_override_applied', done);
   elseif isstruct(s.(f{k})) && numel(s.(f{k})) == 1
      s.(f{k}) = uacs_apply_override(path, s.(f{k}));
   end
end
end
