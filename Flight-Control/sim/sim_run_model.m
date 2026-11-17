function [T, meta] = sim_run_model(mdl, opts)
% SIM_RUN_MODEL Simulate one built model and return its logged outputs
%
%   mdl    model name
%   opts   optional settings, any field may be omitted:
%     solver   'ode4' (default, fixed step P.sim.Ts_ctrl) or 'ode45'
%     t_end    stop time in s, default sim_t_end from the base workspace
%     tol      [rel abs] for the variable-step solver, default P.sim.rel_tol
%              and P.sim.abs_tol
%     max_step optional maximum step in s for the variable-step solver
%     states   true saves block states into meta.states, with the owning block of
%              each column in meta.state_blocks and the column offsets in
%              meta.state_col0
%   T      logged outputs, as returned by sim_read_yout
%   meta   model, solver, t_end, tol, n_samples, outputs, widths and timings

if nargin < 2
   opts = struct();
end
if ~isfield(opts, 'solver') || isempty(opts.solver)
   opts.solver = 'ode4';
end
if ~isfield(opts, 't_end') || isempty(opts.t_end)
   opts.t_end = evalin('base', 'sim_t_end');
end
if ~isfield(opts, 'tol') || isempty(opts.tol)
   Pb = evalin('base', 'P');
   opts.tol = [Pb.sim.rel_tol, Pb.sim.abs_tol];
end
opts.max_step = local_field(opts, 'max_step', []);
opts.states = logical(local_field(opts, 'states', false));

assignin('base', 'sim_t_end', opts.t_end);
if ~bdIsLoaded(mdl)
   load_system(mdl);
end
% exported labels follow the SIGNAL LINE name, so each outport line is renamed
% after its block or sim_read_yout falls back to out1..outN
ops = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Outport');
for k = 1:numel(ops)
   nm = get_param(ops{k}, 'Name');
   ph = get_param(ops{k}, 'PortHandles');
   if ~isempty(ph.Inport) && ph.Inport > 0
      lh = get_param(ph.Inport, 'Line');
      if lh > 0
         set_param(lh, 'Name', nm);
      end
   end
end
% SaveFormat is left unset: it conflicts with ReturnWorkspaceOutputs
set_param(mdl, 'StopTime', 'sim_t_end', 'SimulationMode', 'normal', ...
   'SaveTime', 'on', 'SaveOutput', 'on', ...
   'ReturnWorkspaceOutputs', 'on');
if opts.states
   set_param(mdl, 'SaveState', 'on', 'StateSaveName', 'sim_saved_states');
else
   set_param(mdl, 'SaveState', 'off');
end

switch lower(opts.solver)
   case 'ode4'
      set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
         'FixedStep', 'sim_ts_ctrl');
   case 'ode45'
      set_param(mdl, 'SolverType', 'Variable-step', 'Solver', 'ode45', ...
         'RelTol', 'P.sim.rel_tol', 'AbsTol', 'P.sim.abs_tol');
      if ~isempty(opts.max_step)
         set_param(mdl, 'MaxStep', num2str(opts.max_step));
      else
         set_param(mdl, 'MaxStep', 'auto');
      end
   otherwise
      error('sim_run_model:Solver', 'unknown solver %s', opts.solver);
end

t0 = tic;
try
   set_param(mdl, 'SimulationCommand', 'update');
catch ME
   local_rethrow(ME, 'update', mdl);
end
t_compile = toc(t0);
t1 = tic;
try
   out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
catch ME
   local_rethrow(ME, 'sim', mdl);
end
t_sim = toc(t1);

T = sim_read_yout(out);
meta = struct();
meta.model = mdl;
meta.solver = opts.solver;
meta.t_end = opts.t_end;
meta.tol = opts.tol;
meta.n_samples = numel(T.t);
meta.outputs = T.names;
meta.widths = T.widths;
meta.compile_s = t_compile;
meta.sim_s = t_sim;
meta.states = [];
meta.state_blocks = strings(0, 1);
meta.state_col0 = 0;
meta.state_time = [];
if opts.states
   vs = out.get('sim_saved_states');
   sg = vs.signals;
   meta.states = [sg.values];
   meta.state_blocks = arrayfun(@(s) string(s.blockName), sg);
   meta.state_col0 = [0, cumsum(arrayfun(@(s) size(s.values, 2), sg))];
   meta.state_time = vs.time(:);
end
if strcmpi(opts.solver, 'ode4')
   meta.fixed_step = evalin('base', 'sim_ts_ctrl');
else
   meta.fixed_step = NaN;
end
end

function v = local_field(s, f, d)
if isfield(s, f)
   v = s.(f);
else
   v = d;
end
end

function local_rethrow(ME, stage, mdl)
% the real diagnostics sit one level down in ME.cause, so spell them out
msg = sprintf('sim_run_model:%s failed on %s: %s', stage, mdl, ME.message);
for k = 1:numel(ME.cause)
   msg = sprintf('%s\n  cause %d: %s | %s', msg, k, ME.cause{k}.identifier, ME.cause{k}.message);
   for j = 1:numel(ME.cause{k}.cause)
      msg = sprintf('%s\n     sub: %s | %s', msg, ME.cause{k}.cause{j}.identifier, ...
         ME.cause{k}.cause{j}.message);
   end
end
error('sim_run_model:Stage', '%s', msg);
end
