function [lc, map] = lin_contract(lin)
% LIN_CONTRACT Put the longitudinal and lateral sub-models into the frozen
% sub-model contract ordering.
%
%   The contract fixes the sub-model states as
%     long [dV dalpha dq dtheta] x [dde dT]
%     lat  [dbeta dp dr dphi]   x [dda ddr]
%   linearize_at extracts its sub-matrices straight from the 12-state Jacobian,
%   whose velocity states are the body components u, w and v. A gain designed on
%   [du dw] with Bryson weights meant for [dV dalpha] is wrong by a factor of V0 in
%   the alpha channel, so the exact linear change of variables is applied here,
%   once, and the applied transform is reported:
%
%   dV     =  cos(a0)*du + sin(a0)*dw
%   dalpha = -sin(a0)*du/V0 + cos(a0)*dw/V0
%   dbeta  =  dv/V0                     (flow_state defines beta = asin(v/V))
%
%   lc   the input lin with A_long/B_long/A_lat/B_lat in contract coordinates
%   map  what was applied, for the report: .mode, .T_long, .T_lat, .V0, .alpha0
%
% If the delivered sub-model is already in air-data states the transform is
% skipped, so this stays correct when plant/linearize_at changes its extraction.

lc = lin;
map = struct('mode', 'identity', 'T_long', eye(4), 'T_lat', eye(4), ...
   'V0', NaN, 'alpha0', NaN, 'reason', '');

x_star = lin.x_star(:);
V0 = norm(x_star(1:3));
a0 = atan2(x_star(3), x_star(1));
map.V0 = V0;
map.alpha0 = a0;

names_long = local_names(lin, 'long_state_names');
names_lat = local_names(lin, 'lat_state_names');
body_long = any(strcmpi(names_long, 'w')) || any(strcmpi(names_long, 'u'));
body_lat = any(strcmpi(names_lat, 'v'));

T2 = [cos(a0), sin(a0); -sin(a0) / V0, cos(a0) / V0];
if body_long
   T_long = blkdiag(T2, eye(2));
   lc.A_long = T_long * lin.A_long / T_long;
   lc.B_long = T_long * lin.B_long;
   map.T_long = T_long;
   map.mode = 'rotated_uw_to_Valpha';
   map.reason = 'linearize_at extracted the body components [u w]';
end

if body_lat
   T_lat = diag([1 / V0, 1, 1, 1]);
   lc.A_lat = T_lat * lin.A_lat / T_lat;
   lc.B_lat = T_lat * lin.B_lat;
   map.T_lat = T_lat;
   if body_long
      map.mode = 'rotated_uw_to_Valpha_and_v_to_beta';
   else
      map.mode = 'scaled_v_to_beta';
   end
   map.reason = [map.reason, ' and v'];
end

% Absolute operating point of the two sub-models; the contract states are
% deviations from these, so a caller subtracts them before using the gains
lc.x_op_long = [V0; a0; x_star(5); x_star(8)];
lc.x_op_lat = [0; x_star(4); x_star(6); x_star(7)];
lc.u_op_long = lin.u_star([2 6]);
lc.u_op_full = lin.u_star(:);
end

%% Local helpers
function n = local_names(lin, field)
if isfield(lin, field)
   n = lin.(field);
else
   n = {};
end
end
