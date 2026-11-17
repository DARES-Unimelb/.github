function [T, meta] = sim_run_plant_cmd(mdl, t, u_act, wind_ned, opts)
% SIM_RUN_PLANT_CMD Simulate a bare plant model from an open-loop command series
%
%   mdl       plant model name, uacs_plant or uacs_plant_ref
%   t         nTx1 time vector
%   u_act     nTx6 actuator commands [da de dr dth_l dth_r dT]
%   wind_ned  nTx3 steady wind in NED, m/s
%   opts      optional settings:
%     solver   'ode4' (default, fixed step sim_ts_ctrl) or 'ode45'
%     tol      [rel abs] for the variable-step solver
%     vars     struct pushed into the base workspace just before the simulation
%   T         logged outputs, as returned by sim_read_yout
%   meta      model, solver, t_end, tol, n_samples, outputs, widths and timings
%
% Both plant models are driven through this one function with the same timeseries
% objects, so they see identical inputs

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
% pushed here, next to the call to sim, because the shared base workspace can be
% cleared by another member in between
pushed = 0;
if isfield(opts, 'vars') && isstruct(opts.vars)
   pushed = sim_push_vars(opts.vars);
end
assignin('base', 'sim_t_end', t(end));
% each run reloads the model from <root>/model, so the current base workspace
% initial condition is compiled in; a bare name resolves only when model/ is on
% the path
if bdIsLoaded(mdl)
   close_system(mdl, 0);
end
model_file = fullfile(sim_root(), 'model', [mdl '.slx']);
if exist(model_file, 'file') == 2
   load_system(model_file);
else
   load_system(mdl);
end

% exported labels follow the SIGNAL LINE name, so each outport line is renamed
% after its block
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

% root inport names in port order, so the Dataset elements match the ports
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
