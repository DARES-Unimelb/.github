function probe_tail_for_vmin()
% PROBE_TAIL_FOR_VMIN Lowest speed that trims inside the elevator travel budget, against
% horizontal tail area.
%
%   probe_tail_for_vmin()
%
% The wing loading sets the stall speed, but on this aircraft the ELEVATOR runs out of
% travel well before the wing stalls, so the usable low-speed limit is set by trim. The
% static margin that comes with each tail area is reported alongside, so the trade
% between a lower trimmable speed and a larger tail is explicit.

root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root,'params'), fullfile(root,'plant'), fullfile(root,'run'));
base = uacs_twin_params();

fprintf('payload %.1f kg  all-up %.1f kg  elevator %.1f deg, budget %.0f%% = %.2f deg\n', ...
   base.mass.payload_kg, base.mass.m, base.act.elevator.max_deg, ...
   100*base.geom.elevator_trim_frac, base.geom.elevator_trim_frac*base.act.elevator.max_deg);
fprintf('stall speed %.2f m/s (CL_max %.2f)\n\n', ...
   sqrt(2*base.mass.m*base.env.g/(base.env.rho_sl*base.geom.S*base.aero.CL_max)), ...
   base.aero.CL_max);

fprintf('%8s %8s %8s %10s %10s %10s\n', 'Sh/S', 'Sh[m2]', 'Vh', 'V_trimmin', 'SM[%]', 'note');
for ShS = [0.10 0.12 0.14 0.16 0.18 0.20 0.25 0.30]
    P = base;
    P.geom.Sh_over_S = ShS;
    P.geom.Sh = ShS * P.geom.S;
    P.geom.Vh = P.geom.Sh * P.geom.lh / (P.geom.S * P.geom.cbar);
    % Recompute the derived coefficients that depend on the tail volume
    P.aero.CL_de = P.aero.tau_elevator * P.aero.CL_alpha * P.geom.Sh / P.geom.S;
    P.aero.Cm_alpha = P.aero.Cm_alpha_wing_body - P.geom.Vh * P.aero.CL_alpha;
    P.aero.Cm_q = -2 * P.aero.CL_alpha * P.geom.Vh * (P.geom.lh / P.geom.cbar);
    P.aero.Cm_de = -P.aero.CL_de * P.geom.Vh * (P.geom.lh / P.geom.cbar);

    Vmin = local_trim_min(P);
    SM = -P.aero.Cm_alpha / P.aero.CL_alpha - P.mass.x_cg / P.geom.cbar;
    note = '';
    if Vmin <= P.limits.V_stall_mps * 1.25
        note = 'stall-limited';
    end
    fprintf('%8.3f %8.4f %8.4f %10.2f %10.1f %10s\n', ...
       ShS, P.geom.Sh, P.geom.Vh, Vmin, 100*SM, note);
end
fprintf('\nPROBE DONE\n');
end

function Vmin = local_trim_min(P)
% Bisect for the lowest level-flight speed that trims inside the elevator budget.
lo = P.limits.V_stall_mps * 0.5;
hi = P.op.cruise.V_mps;
if local_frac(P, hi) > P.geom.elevator_trim_frac
    Vmin = NaN;
    return;
end
for k = 1:40
    mid = 0.5 * (lo + hi);
    if local_frac(P, mid) > P.geom.elevator_trim_frac
        lo = mid;
    else
        hi = mid;
    end
end
Vmin = 0.5 * (lo + hi);
end

function f = local_frac(P, V)
try
    tr = trim_cruise(P, struct('V_mps', V, 'gamma_deg', 0.0, 'h_m', P.op.cruise.h_m));
catch
    f = Inf;
    return;
end
if tr.exitflag <= 0 || tr.residual_norm_max > 1e-6
    f = Inf;
    return;
end
f = abs(tr.de_deg) / P.act.elevator.max_deg;
end
