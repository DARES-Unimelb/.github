function tr = sim_v12_trim(P)
% SIM_V12_TRIM Cruise trim for the V12 cross check, computed from P
%
%   P    parameter struct
%   tr   trim_cruise output; errors unless it converged with a residual below 1e-6

tr = trim_cruise(P, P.op.cruise);
assert(tr.exitflag > 0, 'sim_v12_trim:Converged', ...
   'trim_cruise did not converge (exitflag %d)', tr.exitflag);
assert(tr.residual_norm_max < 1e-6, 'sim_v12_trim:Residual', ...
   'trim residual %.3e is above 1e-6, so the initial state is not an equilibrium', ...
   tr.residual_norm_max);
end
