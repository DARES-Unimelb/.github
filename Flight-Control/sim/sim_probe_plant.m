function probe = sim_probe_plant(P, x_star, u_star)
% SIM_PROBE_PLANT Call the plant derivative once at trim and report what happened.
%
%   probe = sim_probe_plant(P, trim.x_star, trim.u_star)
%
% Exists so that a plant which is present but BROKEN fails loudly and early,
% instead of taking a simulation down halfway through a run. The derivative is
% evaluated once here, at the trim point, and the outcome is recorded:
%
%   probe.source    'eom_6dof' when the real plant answered, otherwise the
%                   fallback label from sim_deps_reset
%   probe.ok        true when the derivative is finite and not identically zero
%   probe.dx        the derivative vector itself
%   probe.dx_norm   its 2-norm, printed in the build report
%   probe.error     empty on success, the message when the call threw
%
% A derivative of exactly zero is treated as failure: that is what a model with an
% unconnected or stub right-hand side returns, and it is the silent case that would
% otherwise produce a simulation that runs to completion and never moves.
%
% NOTE ON HISTORY: this file was once deleted by a cleanup pass that filtered on the
% "sim_probe*" name pattern, which also matched it even though it is load-bearing
% runtime code called from build_uacs_models, run_scenarios, run_manual_pid and
% run_lqr. It is restored here. Do not treat a name prefix as proof that a file is a
% throwaway probe; check for callers first.

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
