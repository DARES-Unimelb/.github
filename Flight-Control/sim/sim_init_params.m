function [P, src] = sim_init_params()
% SIM_INIT_PARAMS Push the parameter set into base, recording a failed self check
%
%   [P, src] = sim_init_params()
%   src  'init_uacs_twin', or 'uacs_twin_params: self check FAILED: <reason>'
%
% A failed self check still leaves the models runnable, and src is reported in
% every result file, so a run against unverified parameters stays visible

warning('off', 'sim:ParamsSelfCheck');
try
   init_uacs_twin('quiet', true);
   src = 'init_uacs_twin';
catch err
   P0 = uacs_twin_params();
   assignin('base', 'P', P0);
   f = fieldnames(P0.tags);
   for k = 1:numel(f)
      assignin('base', f{k}, P0.tags.(f{k}));
   end
   src = sprintf('uacs_twin_params: self check FAILED: %s', err.message);
   warning('sim:ParamsSelfCheck', '%s', src);
end
P = evalin('base', 'P');
end
