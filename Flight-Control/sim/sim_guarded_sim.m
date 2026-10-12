function out = sim_guarded_sim(target, vars, varargin)
% SIM_GUARDED_SIM Run sim, retrying once when the shared base workspace was cleared.
%
%   out = sim_guarded_sim('uacs_plant', vars, 'ReturnWorkspaceOutputs', 'on')
%   out = sim_guarded_sim(in, vars)          % in is a Simulink.SimulationInput
%
% One MATLAB session is shared by several members and there is one base
% workspace. A model whose blocks read P, sim_x0 or ref_x0_* from base can lose
% them between the push and the solver's first derivative, and what comes back is
% "the plant cannot find P", which reads like a modelling fault rather than a
% workspace one. This was observed: a job lost P and its own stage flags part way
% through, because a probe in the same session cleared base.
%
% The retry is conditional on evidence: it happens only when a variable this
% caller pushed is actually absent from base. A genuine model or solver fault
% reports itself immediately, unretried.

if nargin < 2 || isempty(vars)
   vars = struct();
end
attempt = 0;
while true
   try
      out = sim(target, varargin{:});
      return;
   catch err
      missing = local_missing(vars);
      if attempt >= 1 || isempty(missing)
         rethrow(err);
      end
      attempt = attempt + 1;
      fprintf('sim_guarded_sim: retry after the base workspace lost %s\n', strjoin(missing, ', '));
      sim_push_vars(vars);
   end
end
end

function m = local_missing(vars)
m = {};
f = fieldnames(vars);
for k = 1:numel(f)
   if evalin('base', sprintf('exist(''%s'', ''var'')', f{k})) == 0
      m{end + 1} = f{k}; %#ok<AGROW>
   end
end
end
