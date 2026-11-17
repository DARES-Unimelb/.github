function rep = sim_deps_report(smoke)
% SIM_DEPS_REPORT Which delivered modules this model layer will actually use
%
%   rep = sim_deps_report()      look the functions up only
%   rep = sim_deps_report(true)  also call the plant once, to catch a broken chain

if nargin < 1
   smoke = false;
end

f = {'eom_6dof', 'flow_state', 'thrust_moment', 'aero_forces', 'trim_cruise', ...
     'linearize_at', 'allocate', 'design_pid', 'design_lqr', 'pilot_to_command', ...
     'lqr_reference', 'lin_contract', 'step_metrics', 'prop_thrust_from_throttle', ...
     'throttle_to_thrust', 'thrust_from_speed'};
rep = struct();
rep.checked = datestr(now, 'yyyy-mm-dd HH:MM:SS');
for k = 1:numel(f)
   rep.(f{k}) = sim_have(f{k});
end

rep.plant_source = 'eom_6dof';
if ~rep.eom_6dof
   rep.plant_source = 'placeholder';
end
rep.meas_source = 'flow_state';
if ~rep.flow_state
   rep.meas_source = 'local';
end
rep.alloc_source = 'allocate';
if ~rep.allocate
   rep.alloc_source = 'placeholder';
end
rep.pilot_source = 'pilot_to_command';
if ~rep.pilot_to_command
   rep.pilot_source = 'placeholder';
end
rep.placeholder_used = ~all([rep.eom_6dof, rep.flow_state, rep.allocate, ...
   rep.pilot_to_command, rep.design_pid, rep.design_lqr, rep.trim_cruise]);

rep.smoke_ok = NaN;
rep.smoke_error = '';
if smoke
   try
      P = evalin('base', 'P');
      x = sim_plant_x0();
      u = zeros(6, 1);
      if evalin('base', 'exist(''trim'', ''var'')') == 1
         u = evalin('base', 'trim.u_star');
      end
      dx = sim_plant_rhs(0, x, u, [0; 0; 0], P);
      assert(numel(dx) == 12 && all(isfinite(dx)), 'plant returned an invalid derivative');
      rep.smoke_ok = true;
   catch err
      rep.smoke_ok = false;
      rep.smoke_error = sprintf('%s | %s', err.identifier, err.message);
   end
end
end
