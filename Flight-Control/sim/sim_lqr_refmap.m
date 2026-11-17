function R = sim_lqr_refmap(P, lin, gains)
% SIM_LQR_REFMAP Linear map from commanded deviations to reference and feedforward
%
%   P, lin, gains   parameters, linearization and gain structs of the LQR design
%   R.long.Tref  4x2  [dV_c; dtheta_c]  -> x_ref_long
%   R.long.Uff   2x2  [dV_c; dtheta_c]  -> u_ff_long, [dde; dT]
%   R.lat.Tref   4x2  [dbeta_c; dphi_c] -> x_ref_lat
%   R.lat.Uff    2x2  [dbeta_c; dphi_c] -> u_ff_lat, [dda; ddr]
%   R.linearity  max difference between the map and a direct lqr_reference solve

unit = @(dV, dth, db, dph) struct('dV', dV, 'dtheta', dth, 'dbeta', db, 'dphi', dph);
r0 = local_ref(P, lin, gains, unit(0, 0, 0, 0));
rV = local_ref(P, lin, gains, unit(1, 0, 0, 0));
rT = local_ref(P, lin, gains, unit(0, 1, 0, 0));
rB = local_ref(P, lin, gains, unit(0, 0, 1, 0));
rP = local_ref(P, lin, gains, unit(0, 0, 0, 1));

R = struct();
R.long = struct();
R.long.Tref = [rV.long.x_ref, rT.long.x_ref];
R.long.Uff = [rV.long.u_ff, rT.long.u_ff];
R.lat = struct();
R.lat.Tref = [rB.lat.x_ref, rP.lat.x_ref];
R.lat.Uff = [rB.lat.u_ff, rP.lat.u_ff];

dV = 0.37;
dth = -0.21;
dph = 0.13;
db = -0.07;
rc = local_ref(P, lin, gains, unit(dV, dth, db, dph));
lin_long = R.long.Tref * [dV; dth];
lin_lat = R.lat.Tref * [db; dph];
uff_long = R.long.Uff * [dV; dth];
uff_lat = R.lat.Uff * [db; dph];
R.linearity = max([max(abs(lin_long - rc.long.x_ref)), ...
   max(abs(lin_lat - rc.lat.x_ref)), ...
   max(abs(uff_long - rc.long.u_ff)), ...
   max(abs(uff_lat - rc.lat.u_ff)), ...
   max(abs(r0.long.x_ref)), max(abs(r0.lat.x_ref))]);
R.residual_long = rc.residual_long;
R.residual_lat = rc.residual_lat;
R.basis_residual = [rV.residual_long, rT.residual_long, rB.residual_lat, rP.residual_lat];
end

%% Local helpers
function r = local_ref(P, lin, gains, cmd)
if sim_have('lqr_reference')
   % residuals are lqr_reference's second output; lift them onto r
   [r, info] = lqr_reference(P, lin, gains, cmd);
   r.residual_long = info.residual_long;
   r.residual_lat = info.residual_lat;
   r.info = info;
   return;
end
lc = lin_contract(lin);
r = struct('long', [], 'lat', [], 'residual_long', NaN, 'residual_lat', NaN);
% solve A*x_ref + B*u_ff = 0 for the free states and inputs
A = lc.A_long;
B = lc.B_long;
M = [A(:, 2), A(:, 3), B(:, 1), B(:, 2)];
y = M \ (-(A(:, 1) * cmd.dV + A(:, 4) * cmd.dtheta));
x_ref = [cmd.dV; y(1); y(2); cmd.dtheta];
u_ff = [y(3); y(4)];
r.long = struct('x_ref', x_ref, 'u_ff', u_ff);
r.residual_long = norm(A * x_ref + B * u_ff);
A = lc.A_lat;
B = lc.B_lat;
M = [A(:, 2), A(:, 3), B(:, 1), B(:, 2)];
y = M \ (-(A(:, 1) * cmd.dbeta + A(:, 4) * cmd.dphi));
x_ref = [cmd.dbeta; y(1); y(2); cmd.dphi];
u_ff = [y(3); y(4)];
r.lat = struct('x_ref', x_ref, 'u_ff', u_ff);
r.residual_lat = norm(A * x_ref + B * u_ff);
end
