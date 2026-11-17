function out = sim_guarded_sim(target, vars, varargin)
% SIM_GUARDED_SIM Run sim, retrying once when the shared base workspace was cleared
%
%   out = sim_guarded_sim('uacs_plant', vars, 'ReturnWorkspaceOutputs', 'on')
%   out = sim_guarded_sim(in, vars)          % in is a Simulink.SimulationInput
%
% vars is a struct of base workspace variables the model reads, and defaults to
% an empty struct. The retry happens only when one of them is really missing

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
