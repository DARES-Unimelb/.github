function sim_new_model(mdl, file)
% SIM_NEW_MODEL Start an empty model with the contract solver settings.
%
% Any loaded copy is discarded, the file on disk is removed, and the model is
% created and loaded empty. That is what makes the builders repeatable: a block
% is never added on top of a previous run's block of the same name, which is the
% failure the "A new block named ... cannot be added" message reports.
%
% architecture.md section 6: fixed step ode4 at P.sim.Ts_ctrl by default, with
% the variable-step solver reserved for the step-size independence cross-check.
% The sample time and stop time are workspace expressions, not numbers.

if bdIsLoaded(mdl)
   close_system(mdl, 0);
end
if exist(file, 'file')
   delete(file);
end
% save_system does not create directories, and a missing model/ folder would
% fail at the very end of the first build.
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
