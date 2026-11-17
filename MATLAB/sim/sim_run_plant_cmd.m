function [T, meta] = sim_run_plant_cmd(mdl, t, u_act, wind_ned, opts)
% SIM_RUN_PLANT_CMD Simulate a bare plant model from an open-loop command series.
%
%   [T, meta] = sim_run_plant_cmd('uacs_plant', t, u_act, wind_ned)
%   [T, meta] = sim_run_plant_cmd('uacs_plant_ref', t, u_act, wind_ned, opts)
%
% The plant models are objects with two root inports and no sources of their own,
% so the command has to come from outside. Simulink.SimulationInput with a
% Dataset of timeseries feeds them, and because both uacs_plant and
% uacs_plant_ref are driven through this one function with the same timeseries
% objects, the two models see bit-identical inputs. Driving one through
% LoadExternalInput and the other through a From Workspace block would have left
% their interpolation rules free to differ, which is a difference that would land
% in the cross-check undetected. The two mechanisms were measured against each
% other first and agreed to the last bit (results/v12_probe_ext.txt) before this
% choice was made.
%
% opts.solver   'ode4' (default, fixed step sim_ts_ctrl) or 'ode45'
% opts.tol      [rel abs] for the variable-step solver
% opts.vars     struct of base workspace variables pushed immediately before the
%               simulation. See sim_push_vars for why this is not left to the
%               caller's earlier push.
%
% T comes back through sim_read_yout, so a scenario reads signals by name.

if nargin < 5
   opts = struct();
end
if ~isfield(opts, 'solver') || isempty(opts.solver)
   opts.solver = 'ode4';
end
t = t(:);
n = numel(t);
u_act = reshape(u_act, n, []);
wind_ned = reshape(wind_ned, n, []);
if ~isfield(opts, 'tol') || isempty(opts.tol)
   Pb = evalin('base', 'P');
   opts.tol = [Pb.sim.rel_tol, Pb.sim.abs_tol];
end

assignin('base', 'sim_t_end', t(end));
% Re-push the modelling variables here, in the function that calls sim, because
% the shared base workspace can be cleared by another member between the caller's
% push and the solver's first derivative.
pushed = 0;
if isfield(opts, 'vars') && isstruct(opts.vars)
   pushed = sim_push_vars(opts.vars);
end
assignin('base', 'sim_t_end', t(end));
% Each run loads the model from disk, so a changed initial condition in the base
% workspace is compiled in. Reusing a model that is already loaded can leave the
% previous run's mask parameter values in the compiled model.
%
% The load is by FULL PATH. Loading by bare model name resolves only when model/ is
% on the MATLAB path or the model is already open, and this line closes the model
% first, so the bare name worked only while some other caller happened to have left
% model/ on the path. In the default state it failed with "Unable to find system or
% file 'uacs_plant'", which stopped the V12 calibrate and crosscheck stages.
if bdIsLoaded(mdl)
   close_system(mdl, 0);
end
model_file = fullfile(sim_root(), 'model', [mdl '.slx']);
if exist(model_file, 'file') == 2
   load_system(model_file);
else
   load_system(mdl);
end

% Exported signal labels follow the SIGNAL LINE name, not the Outport block name,
% so each line into an outport is renamed to match its block. Same rule as
% sim_run_model.
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

% Root inport names, in port order, so the Dataset elements match the ports.
ips = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport');
names = cell(1, numel(ips));
for k = 1:numel(ips)
   pr = str2double(get_param(ips{k}, 'Port'));
   names{pr} = get_param(ips{k}, 'Name');
end
assert(numel(names) == 2, 'sim_run_plant_cmd:Ports', ...
   '%s has %d root inports, this runner drives u_act and wind_ned', mdl, numel(names));

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(u_act, t), names{1});
ds = ds.addElement(timeseries(wind_ned, t), names{2});

in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', 'sim_t_end');
switch lower(opts.solver)
   case 'ode4'
      in = in.setModelParameter('SolverType', 'Fixed-step', 'Solver', 'ode4', ...
         'FixedStep', 'sim_ts_ctrl');
   case 'ode45'
      in = in.setModelParameter('SolverType', 'Variable-step', 'Solver', 'ode45', ...
         'RelTol', 'P.sim.rel_tol', 'AbsTol', 'P.sim.abs_tol');
   otherwise
      error('sim_run_plant_cmd:Solver', 'unknown solver %s', opts.solver);
end
in = in.setModelParameter('SimulationMode', 'normal', 'SaveTime', 'on', ...
   'SaveOutput', 'on', 'SaveFormat', 'StructureWithTime', 'ReturnWorkspaceOutputs', 'on');

t0 = tic;
out = sim_guarded_sim(in, local_vars(opts));
t_sim = toc(t0);

T = sim_read_yout(out);
meta = struct();
meta.model = mdl;
meta.solver = opts.solver;
meta.t_end = t(end);
meta.tol = opts.tol;
meta.n_samples = numel(T.t);
meta.outputs = T.names;
meta.widths = T.widths;
meta.sim_s = t_sim;
meta.vars_pushed = pushed;
if strcmpi(opts.solver, 'ode4')
   meta.fixed_step = local_field(opts.vars, 'sim_ts_ctrl', NaN);
   if isnan(meta.fixed_step)
      meta.fixed_step = evalin('base', 'sim_ts_ctrl');
   end
else
   meta.fixed_step = NaN;
end
end

function v = local_vars(opts)
v = struct();
if isfield(opts, 'vars') && isstruct(opts.vars)
   v = opts.vars;
end
end

function x = local_field(s, f, d)
if isstruct(s) && isfield(s, f)
   x = s.(f);
else
   x = d;
end
end
