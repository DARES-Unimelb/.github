function tr = sim_v12_trim(P)
% SIM_V12_TRIM Cruise trim for the V12 cross check, from the current parameters.
%
% sim_load_design would also serve, but it designs PID and LQR gains on the way,
% and V12 is about the plant: it should not fail, or change its starting point,
% because a control design module is being edited. trim_cruise is the
% authoritative plant trim and it is the only input the cross check needs.
%
% A stored trim is deliberately not used. results/plant_baseline.mat is tied to
% the parameter revision it was computed under, and a stale initial state would
% put both models somewhere other than the design point without either of them
% reporting it.

tr = trim_cruise(P, P.op.cruise);
assert(tr.exitflag > 0, 'sim_v12_trim:Converged', ...
   'trim_cruise did not converge (exitflag %d)', tr.exitflag);
assert(tr.residual_norm_max < 1e-6, 'sim_v12_trim:Residual', ...
   'trim residual %.3e is above 1e-6, so the initial state is not an equilibrium', ...
   tr.residual_norm_max);
end
