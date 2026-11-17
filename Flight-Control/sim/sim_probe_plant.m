function probe = sim_probe_plant(P, x_star, u_star)
% SIM_PROBE_PLANT Evaluate the plant derivative once at trim and report the outcome
%
%   P, x_star, u_star   parameters plus the trim state and trim input
%   probe.source    'eom_6dof' when the real plant answered, else the fallback label
%   probe.ok        true when the derivative is finite and not identically zero
%   probe.dx        the derivative vector
%   probe.dx_norm   its 2-norm
%   probe.error     empty on success, the message when the call threw
%
% A derivative of exactly zero counts as failure: that is what a stub right-hand
% side returns, and the run would otherwise complete without ever moving

probe = struct();
probe.ok = false;
probe.dx = [];
probe.dx_norm = NaN;
probe.error = '';
probe.source = sim_plant_source();

try
   dx = eom_6dof(0, x_star(:), u_star(:), P);
   probe.dx = dx;
   if ~all(isfinite(dx))
      probe.error = 'derivative contains non-finite values';
      return;
   end
   probe.dx_norm = norm(dx);
   if probe.dx_norm == 0
      probe.error = 'derivative is identically zero, which means a stub right-hand side';
      return;
   end
   probe.ok = true;
   probe.source = 'eom_6dof';
catch ME
   probe.error = sprintf('%s | %s', ME.identifier, ME.message);
end
end
