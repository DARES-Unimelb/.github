function sim_new_model(mdl, file)
% SIM_NEW_MODEL Start an empty model with the contract solver settings
%
%   mdl    model name
%   file   path the model will be saved to; the folder is created when missing
%
% A loaded copy is discarded and the file on disk deleted first, so a build never
% adds a block on top of a previous run's block of the same name
%
% Fixed step ode4 at the workspace expression sim_ts_ctrl, stop time sim_t_end

if bdIsLoaded(mdl)
   close_system(mdl, 0);
end
if exist(file, 'file')
   delete(file);
end
% save_system does not create directories, so make the parent folder now
parent = fileparts(file);
if ~isempty(parent) && ~exist(parent, 'dir')
   mkdir(parent);
end
new_system(mdl);
load_system(mdl);

set_param(mdl, ...
   'SolverType', 'Fixed-step', ...
   'Solver', 'ode4', ...
   'FixedStep', 'sim_ts_ctrl', ...
   'StopTime', 'sim_t_end', ...
   'SimulationMode', 'normal', ...
   'SaveTime', 'on', ...
   'SaveOutput', 'on', ...
   'SaveFormat', 'StructureWithTime', ...
   'ReturnWorkspaceOutputs', 'on', ...
   'AlgebraicLoopMsg', 'error', ...
   'SolverPrmCheckMsg', 'none');
end
