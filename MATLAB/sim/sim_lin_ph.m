function [lin, info] = sim_lin_ph(P, trim)
% SIM_LIN_PH Numerical linearisation of the placeholder plant.
%
% Produces the same structure architecture.md 5.2 freezes for plant/linearize_at,
% by central differences on sim_plant_rhs_ph and the same change of variables
% control/lin_contract.m applies: body velocity components to air-data states.
%
% Used only when plant/trim_cruise.m or plant/linearize_at.m cannot run, and
% recorded in the results as lin_source = 'placeholder'.

hx = 1e-6;
hu = 1e-6;
n = 12;
m = 6;
x0 = trim.x_star(:);
u0 = trim.u_star(:);
f0 = sim_plant_rhs_ph(0, x0, u0, P);

A = zeros(n, n);
for j = 1:n
   dx = zeros(n, 1);
   dx(j) = hx * max(1, abs(x0(j)));
   fp = sim_plant_rhs_ph(0, x0 + dx, u0, P);
   fm = sim_plant_rhs_ph(0, x0 - dx, u0, P);
   A(:, j) = (fp - fm) / (2 * dx(j));
end
B = zeros(n, m);
for j = 1:m
   du = zeros(m, 1);
   du(j) = hu * max(1, abs(u0(j)));
   fp = sim_plant_rhs_ph(0, x0, u0 + du, P);
   fm = sim_plant_rhs_ph(0, x0, u0 - du, P);
   B(:, j) = (fp - fm) / (2 * du(j));
end

V0 = norm(x0(1:3));
a0 = atan2(x0(3), x0(1));
T2 = [cos(a0), sin(a0); -sin(a0) / V0, cos(a0) / V0];

idx_long_x = [1 3 5 8];
idx_long_u = [2 6];
idx_lat_x = [2 4 6 7];
idx_lat_u = [1 3];

T_long = blkdiag(T2, eye(2));
A_long_body = A(idx_long_x, idx_long_x);
B_long_body = B(idx_long_x, idx_long_u);
A_long = T_long * A_long_body / T_long;
B_long = T_long * B_long_body;

T_lat = diag([1 / V0, 1, 1, 1]);
A_lat = T_lat * A(idx_lat_x, idx_lat_x) / T_lat;
B_lat = T_lat * B(idx_lat_x, idx_lat_u);

lin = struct();
lin.A = A;
lin.B = B;
lin.C = eye(n);
lin.D = zeros(n, m);
lin.A_long = A_long;
lin.B_long = B_long;
lin.A_lat = A_lat;
lin.B_lat = B_lat;
lin.modes_long = local_modes(A_long);
lin.modes_lat = local_modes(A_lat);
lin.idx_long_x = idx_long_x;
lin.idx_long_u = idx_long_u;
lin.idx_lat_x = idx_lat_x;
lin.idx_lat_u = idx_lat_u;
lin.x_star = x0;
lin.u_star = u0;
lin.hx = hx;
lin.hu = hu;
lin.source = 'placeholder';

info = struct('step', hx, 'hx', hx, 'hu', hu, 'n_eval', 2 * (n + m) + 1, ...
   'source', 'placeholder', 'f0_norm', norm(f0(1:6)));
end

function modes = local_modes(A)
e = eig(A);
modes = struct('eigenvalue', {}, 'freq_nat_rad_s', {}, 'damping', {}, ...
   'real_part', {}, 'imag_part', {}, 'tau_s', {}, 'freq_hz', {}, 'name', {});
for k = 1:numel(e)
   lam = e(k);
   wn = abs(lam);
   zeta = -real(lam) / max(wn, eps);
   tau = Inf;
   if real(lam) < 0
      tau = -1 / real(lam);
   end
   modes(end + 1) = struct('eigenvalue', lam, 'freq_nat_rad_s', wn, ...
      'damping', zeta, 'real_part', real(lam), 'imag_part', imag(lam), ...
      'tau_s', tau, 'freq_hz', wn / (2 * pi), 'name', sprintf('mode%d', k)); %#ok<AGROW>
end
[~, order] = sort([modes.freq_nat_rad_s]);
modes = modes(order);
end
