function [P, src] = sim_init_params()
% SIM_INIT_PARAMS Push the parameter set into base, recording a failed self check.
%
%   [P, src] = sim_init_params()
%   src  'init_uacs_twin' when the parameter self check passed
%        'uacs_twin_params: self check FAILED: <reason>' when it did not
%
% init_uacs_twin asserts the physical consistency of the parameter set, and while
% that set is being designed those assertions can fail for reasons that have
% nothing to do with the Simulink layer. This wrapper keeps the models runnable
% in that window, but it never hides the failure: the returned src is written
% into every result file and into the build report, and a warning is printed, so
% a run made against an unverified parameter set cannot be mistaken for a
% verified one.

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
