function [ref, info] = lqr_reference(P, lin, gains, cmd)
% LQR_REFERENCE Self-consistent reference state and feedforward for the LQR laws.
%
%   ref = lqr_reference(P, lin, gains, cmd)
%
%   cmd.dV      m/s   airspeed deviation from the design point, default 0
%   cmd.dtheta  rad   pitch attitude deviation, default 0
%   cmd.dphi    rad   bank angle deviation, default 0
%   cmd.dbeta   rad   sideslip deviation, default 0
%
% A reference state is only usable if it is an equilibrium of the model the gains
% were designed on, that is if A*x_ref + B*u_ff = 0; this solves that equation for
% the free states and the feedforward, given the commanded states, and returns
% both the deviation-coordinate set and the absolute set. The Jacobian is taken
% about the trim point, so in deviation coordinates the design point comes out as
% x_ref = 0 with u_ff = 0 and the only nonzero feedforward is u_star in absolute
% actuator units.
%
% info.residual_long / .lat are the norms of A*x_ref + B*u_ff, which V5 needs to
% be at machine level; anything larger means the commanded pair cannot be trimmed
% and the reference is not reachable.

if nargin < 4
   cmd = struct();
end
cmd = local_defaults(cmd);

A = gains.long.A;
B = gains.long.B;
dV = cmd.dV;
dth = cmd.dtheta;
% Unknowns [dalpha; dq; dde; dT]; rows 1..4 are the four state derivatives
Ml = [A(:, 2), A(:, 3), B(:, 1), B(:, 2)];
rl = -(A(:, 1) * dV + A(:, 4) * dth);
yl = Ml \ rl;
x_ref_l = [dV; yl(1); yl(2); dth];
u_ff_l = [yl(3); yl(4)];
res_l = A * x_ref_l + B * u_ff_l;

A = gains.lat.A;
B = gains.lat.B;
db = cmd.dbeta;
dphi = cmd.dphi;
% Unknowns [dp; dr; dda; ddr]
Mt = [A(:, 2), A(:, 3), B(:, 1), B(:, 2)];
rt = -(A(:, 1) * db + A(:, 4) * dphi);
yt = Mt \ rt;
x_ref_t = [db; yt(1); yt(2); dphi];
u_ff_t = [yt(3); yt(4)];
res_t = A * x_ref_t + B * u_ff_t;

ref = struct();
ref.long = struct();
ref.long.x_ref = x_ref_l;
ref.long.u_ff = u_ff_l;
ref.long.x_ref_abs = gains.long.x_op + x_ref_l;
ref.long.u_abs = gains.long.u_star_full;
ref.long.u_abs(gains.long.u_idx_full) = ...
   gains.long.u_star_full(gains.long.u_idx_full) + u_ff_l;
ref.long.r_int = gains.long.C_int * x_ref_l;
ref.long.x_ref_aug = [x_ref_l; 0];
ref.lat = struct();
ref.lat.x_ref = x_ref_t;
ref.lat.u_ff = u_ff_t;
ref.lat.x_ref_abs = gains.lat.x_op + x_ref_t;
ref.lat.u_abs = gains.long.u_star_full;
ref.lat.u_abs(gains.lat.u_idx_full) = ...
   gains.long.u_star_full(gains.lat.u_idx_full) + u_ff_t;
ref.lat.r_int = gains.lat.C_int * x_ref_t;
ref.lat.x_ref_aug = [x_ref_t; 0];

% The design point, for the report: both feedforward values side by side
ref.design_point = struct();
ref.design_point.x_ref_long = zeros(4, 1);
ref.design_point.u_ff_long = zeros(2, 1);
ref.design_point.u_abs_long = gains.long.u_star_full([2 6]);
ref.design_point.x_ref_lat = zeros(4, 1);
ref.design_point.u_ff_lat = zeros(2, 1);
ref.design_point.u_abs_lat = gains.long.u_star_full([1 3]);

info = struct();
info.cmd = cmd;
info.residual_long = norm(res_l);
info.residual_lat = norm(res_t);
info.reachable_long = info.residual_long < 1e-9;
info.reachable_lat = info.residual_lat < 1e-9;
info.cond_long = cond(Ml);
info.cond_lat = cond(Mt);
end

%% Local helpers
function c = local_defaults(c)
f = {'dV', 'dtheta', 'dphi', 'dbeta'};
for k = 1:numel(f)
   if ~isfield(c, f{k}) || isempty(c.(f{k}))
      c.(f{k}) = 0;
   end
end
end
