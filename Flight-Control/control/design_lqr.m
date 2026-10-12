function gains = design_lqr(P, lin, opts)
% DESIGN_LQR Longitudinal and lateral-directional LQR for the UACS twin.
%
%   gains = design_lqr(P, lin)
%   gains = design_lqr(P, lin, struct('long_int_out', 'V', 'lat_int_out', 'phi'))
%
% Channels, all in the deviation coordinates fixed by architecture.md 5.2
%   long  x = [dV dalpha dq dtheta]  u = [dde dT]  integrator on the output
%         selected by opts.long_int_out, 'V' (default) or 'theta'
%   lat   x = [dbeta dp dr dphi]     u = [dda ddr] integrator on the output
%         selected by opts.lat_int_out, 'phi' (default) or 'beta'
%
% Both an augmented (2x5) and an unaugmented (2x4) gain are produced, because
% V5 requires the pair: without integral action a step on the regulated output
% settles with a steady-state error, with integral action it does not.
%
% Weights start from the Bryson values in P.lqr and are not edited. The
% integrator weight follows the same rule as the state it regulates:
% q_int = q_diag(index of that output), so the integrator carries the same
% tolerance as the state itself. Any change to a weight has to be recorded in
% docs/design_notes_control.md with its before value, after value and reason.
%
% Coordinate trap this function exists to close: A and B are Jacobians about
% the trim point, so the origin is the equilibrium and the feedforward in
% deviation coordinates is zero. The reference is not free either: for a
% commanded output r the reference state has to satisfy
% A*x_ref + B*u_ff = 0 with C*x_ref = r, which is what lqr_reference.m solves.
% Feeding absolute states to these gains parks the aircraft somewhere else.

if nargin < 3
   opts = struct();
end
if ~isfield(opts, 'long_int_out')
   opts.long_int_out = 'V';
end
if ~isfield(opts, 'lat_int_out')
   opts.lat_int_out = 'phi';
end

[lco, map] = lin_contract(lin);

gains = struct();
gains.created = datestr(now, 'yyyy-mm-dd HH:MM:SS');
gains.contract = map;
gains.source_step = local_field(lin, 'step', NaN);

% ================= longitudinal =================
A = lco.A_long;
B = lco.B_long;
q_diag = P.lqr.long.q_diag(:)';
r_diag = P.lqr.long.r_diag(:)';
idx_l = local_output_index(opts.long_int_out, {'V', 'theta'}, [1 4]);
q_int_l = q_diag(idx_l);
C_l = zeros(1, 4);
C_l(idx_l) = 1;

A_aug = [A, zeros(4, 1); C_l, 0];
B_aug = [B; zeros(1, 2)];
Q_aug = blkdiag(diag(q_diag), q_int_l);
R = diag(r_diag);

[K, ~, ~] = lqr(A_aug, B_aug, Q_aug, R);
K_unaug = lqr(A, B, diag(q_diag), R);

% Riccati self-check: lqr against care and against the Riccati residual.
[X, ~, ~] = care(A_aug, B_aug, Q_aug, R);
K_care = R \ (B_aug' * X);
ric = A_aug' * X + X * A_aug - X * B_aug * (R \ (B_aug' * X)) + Q_aug;

long = struct();
long.int_output = opts.long_int_out;
long.int_index = idx_l;
long.A = A;
long.B = B;
long.A_aug = A_aug;
long.B_aug = B_aug;
long.C_int = C_l;
long.Q = Q_aug;
long.R = R;
long.q_diag = q_diag;
long.r_diag = r_diag;
long.q_int = q_int_l;
long.K = K;
long.K_unaug = K_unaug;
long.K_care = K_care;
long.dK_care = max(abs(K(:) - K_care(:)));
long.riccati_residual = norm(ric, 'fro');
long.poles = eig(A_aug - B_aug * K);
long.poles_unaug = eig(A - B * K_unaug);
long.stable = all(real(long.poles) < 0);
long.stable_unaug = all(real(long.poles_unaug) < 0);
long.margins = local_loop_margins(A_aug, B_aug, K, {'de', 'dT'});
long.margins_unaug = local_loop_margins(A, B, K_unaug, {'de', 'dT'});

% V5 evidence on the augmented and unaugmented closed loop.
test_l = 2.0;                          % m/s, step on the regulated output
long.ss_test = local_ss_error(A, B, K_unaug, K, idx_l, test_l);
long.ss_test.unaug.augmented = false;
long.x_op = lco.x_op_long;
long.u_op = lco.u_op_long;
long.u_idx_full = [2 6];               % de and dT inside the 6-dim plant input
long.u_star_full = lco.u_op_full;
long.xop_names = {'V', 'alpha', 'q', 'theta'};
long.u_names = {'de', 'dT'};
long.law = ['u_dev = u_ff + K*([x_ref; 0] - [x_dev; z]); x_dev = x_abs - x_op; ' ...
   'z = integral(C_int*x_dev - r); u_abs(u_idx_full) = u_star_full(u_idx_full) + u_dev'];
gains.long = long;

% ================= lateral-directional =================
A = lco.A_lat;
B = lco.B_lat;
q_diag = P.lqr.lat.q_diag(:)';
r_diag = P.lqr.lat.r_diag(:)';
idx_t = local_output_index(opts.lat_int_out, {'phi', 'beta'}, [4 1]);
q_int_t = q_diag(idx_t);
C_t = zeros(1, 4);
C_t(idx_t) = 1;

A_aug = [A, zeros(4, 1); C_t, 0];
B_aug = [B; zeros(1, 2)];
Q_aug = blkdiag(diag(q_diag), q_int_t);
R = diag(r_diag);

[K, ~, ~] = lqr(A_aug, B_aug, Q_aug, R);
K_unaug = lqr(A, B, diag(q_diag), R);

[X, ~, ~] = care(A_aug, B_aug, Q_aug, R);
K_care = R \ (B_aug' * X);
ric = A_aug' * X + X * A_aug - X * B_aug * (R \ (B_aug' * X)) + Q_aug;

lat = struct();
lat.int_output = opts.lat_int_out;
lat.int_index = idx_t;
lat.A = A;
lat.B = B;
lat.A_aug = A_aug;
lat.B_aug = B_aug;
lat.C_int = C_t;
lat.Q = Q_aug;
lat.R = R;
lat.q_diag = q_diag;
lat.r_diag = r_diag;
lat.q_int = q_int_t;
lat.K = K;
lat.K_unaug = K_unaug;
lat.K_care = K_care;
lat.dK_care = max(abs(K(:) - K_care(:)));
lat.riccati_residual = norm(ric, 'fro');
lat.poles = eig(A_aug - B_aug * K);
lat.poles_unaug = eig(A - B * K_unaug);
lat.stable = all(real(lat.poles) < 0);
lat.stable_unaug = all(real(lat.poles_unaug) < 0);
lat.margins = local_loop_margins(A_aug, B_aug, K, {'da', 'dr'});
lat.margins_unaug = local_loop_margins(A, B, K_unaug, {'da', 'dr'});

test_t = deg2rad(10);                  % rad, bank step
lat.ss_test = local_ss_error(A, B, K_unaug, K, idx_t, test_t);
lat.x_op = lco.x_op_lat;
lat.u_op = lin.u_star([1 3]);          % da and dr at the design point
lat.u_idx_full = [1 3];                % da and dr inside the 6-dim plant input
lat.xop_names = {'beta', 'p', 'r', 'phi'};
lat.u_names = {'da', 'dr'};
lat.law = ['u_dev = u_ff + K*([x_ref; 0] - [x_dev; z]); x_dev = x_abs - x_op; ' ...
   'z = integral(C_int*x_dev - r); u_abs(u_idx_full) = u_star_full(u_idx_full) + u_dev'];
gains.lat = lat;

gains.lin_contract = map;
gains.weights_unchanged = true;
gains.weights_source = 'P.lqr.long.q_diag/r_diag, P.lqr.lat.q_diag/r_diag (Bryson)';
end

% ------------------------------------------------------------------------
function idx = local_output_index(name, names, idxs)
idx = idxs(strcmpi(name, names));
assert(isscalar(idx), 'design_lqr: unknown integrator output %s', name);
end

function v = local_field(s, f, d)
if isfield(s, f)
   v = s.(f);
else
   v = d;
end
end

function m = local_loop_margins(A, B, K, labels)
% One input broken at a time, the other loops left closed, which is the only
% honest way to read margins off a multiloop design. Fields per architecture
% 5.3: gm, pm, dm plus the crossover frequencies and the closed-loop poles.
n_in = size(B, 2);
m = struct('loop', {}, 'gm', {}, 'gm_db', {}, 'pm_deg', {}, 'dm_s', {}, ...
   'wcg_rad_s', {}, 'wcp_rad_s', {}, 'loop_stable', {}, 'closed_loop_poles', {});
poles = eig(A - B * K);
for j = 1:n_in
   k = setdiff(1:n_in, j);
   A_k = A - B(:, k) * K(k, :);
   e = struct('loop', labels{j}, 'gm', Inf, 'gm_db', Inf, 'pm_deg', Inf, ...
      'dm_s', Inf, 'wcg_rad_s', NaN, 'wcp_rad_s', NaN, ...
      'loop_stable', true, 'closed_loop_poles', poles);
   try
      S = allmargin(ss(A_k, B(:, j), K(j, :), 0));
      if ~isempty(S.GainMargin)
         [e.gm, i] = min(S.GainMargin);
         e.wcg_rad_s = S.GMFrequency(i);
         e.gm_db = 20 * log10(e.gm);
      end
      if ~isempty(S.PhaseMargin)
         [e.pm_deg, i] = min(S.PhaseMargin);
         e.wcp_rad_s = S.PMFrequency(i);
      end
      if ~isempty(S.DelayMargin)
         e.dm_s = min(S.DelayMargin);
      end
      e.loop_stable = all(S.Stable);
   catch err
      e.loop_stable = false;
      e.note = err.message;
   end
   m(end + 1) = e;
end
end

function s = local_ss_error(A, B, K_unaug, K, idx, r)
% Steady-state response of both closed loops to a step r on the regulated
% output. Unaugmented: u = -K*(x - x_ref) with x_ref = e_idx*r, so the loop
% parks at -(A-BK)\(B*K*x_ref) and misses r. Augmented: the integrator forces
% C*x = r exactly; solve the augmented equilibrium for x and z.
n = size(A, 1);
m_in = size(B, 2);
e = zeros(n, 1);
e(idx) = 1;
x_ref = e * r;

x_ss_u = -(A - B * K_unaug) \ (B * K_unaug * x_ref);
err_u = e' * x_ss_u - r;

Kx = K(:, 1:n);
Kz = K(:, n + 1:end);
M = [A - B * Kx, -B * Kz; e', 0];
sol = M \ [-B * Kx * x_ref; r];
x_ss_a = sol(1:n);
z_ss = sol(n + 1);
err_a = e' * x_ss_a - r;

s = struct();
s.output = idx;
s.step = r;
s.ref_state = x_ref;
s.unaug = struct('augmented', false, 'x_ss', x_ss_u, 'out_ss', e' * x_ss_u, ...
   'error', err_u, 'error_rel', err_u / r, 'z_ss', 0);
s.aug = struct('augmented', true, 'x_ss', x_ss_a, 'out_ss', e' * x_ss_a, ...
   'error', err_a, 'error_rel', err_a / r, 'z_ss', z_ss);
% Both feedforwards, so the two coordinate sets can never be confused.
s.u_ff_dev_unaug = zeros(m_in, 1);
s.u_ff_dev_aug = zeros(m_in, 1);
end
