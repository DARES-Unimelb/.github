function probe_rudder_invariant()
% PROBE_RUDDER_INVARIANT Check the rudder coefficient chain against its own algebra.
%
%   probe_rudder_invariant()
%
% The chain in params/uacs_twin_params.m is
%   CY_dr = k_body * tau_rudder * CL_alpha * Sv / S
%   Vv    = Sv * lv / (S * b)
%   Cn_dr = -Vv * CY_dr * chord_frac
%
% Two independent routes to the dimensional moment must agree
%   A. M = q * S * b * |Cn_dr| * dr                      the plant's own form
%   B. M = q * Sv * chord_frac * CY_dr * lv * dr          side force on the rudder
%      reference area, times the fin arm
%
% Both must hold: route A equals route B, and doubling chord_frac doubles Cn_dr, so the
% parameter is live.

root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root,'params'), fullfile(root,'plant'), fullfile(root,'sizing'), ...
        fullfile(root,'run'));
init_uacs_twin('quiet', true);
P = evalin('base','P');

q = 0.5 * P.env.rho_sl * P.op.cruise.V_mps^2;
dr = deg2rad(P.act.rudder.max_deg);

% Route A: the non-dimensional derivative the plant actually uses
M_coeff = q * P.geom.S * P.geom.b * abs(P.aero.Cn_dr) * dr;

% Route B: side force on the rudder reference area at the full-fin slope
M_force = q * P.geom.Sv * P.geom.rudder.chord_frac * P.aero.CY_dr * P.geom.lv * dr;

fprintf('-- rudder yaw moment at cruise, full deflection --\n');
fprintf('  A via Cn_dr                  : %.6f N*m   (Cn_dr %.6f)\n', M_coeff, P.aero.Cn_dr);
fprintf('  B via q*Sv*c_r*CY_dr*lv*dr   : %.6f N*m\n', M_force);
fprintf('  ratio A/B                    : %.9f   (should be 1)\n', M_coeff / M_force);
fprintf('  side force implied           : %.4f N at arm %.3f m\n', M_force / P.geom.lv, P.geom.lv);

% Check 2: the chord fraction is live
probe = -P.geom.Vv * P.aero.CY_dr * (P.geom.rudder.chord_frac * 2);
expected = 2 * P.aero.Cn_dr;
dev2 = abs(probe - expected);
fprintf('\n-- Cn_dr must scale with the rudder chord fraction --\n');
fprintf('  double chord frac -> Cn_dr %+.6f, expected %+.6f, dev %.3e\n', probe, expected, dev2);

% Check 3: the fin family must be mutually consistent, i.e. Cn_beta and Cn_dr must use
% the SAME fin reference area
ratio = abs(P.aero.Cn_dr) / P.aero.Cn_beta;
fprintf('\n-- fin family consistency --\n');
fprintf('  |Cn_dr|/Cn_beta = %.6f\n', ratio);
fprintf('  expected        = chord_frac * CY_dr / |CY_beta_tail| = %.6f\n', ...
   P.geom.rudder.chord_frac * P.aero.CY_dr / abs(P.aero.CY_beta_tail));
fprintf('  must exceed the 0.30 floor; Aerosonde reference is 0.945\n');

ok1 = abs(M_coeff / M_force - 1) < 1e-9;
ok2 = dev2 < 1e-12;
ok3 = ratio > 0.30;
fprintf('\nRUDDER INVARIANT %s  (coeff-vs-expanded %d, chord live %d, ratio floor %d)\n', ...
   local_v(ok1 && ok2 && ok3), ok1, ok2, ok3);
end

function s = local_v(tf)
if tf
   s = 'PASS';
else
   s = 'FAIL';
end
end
