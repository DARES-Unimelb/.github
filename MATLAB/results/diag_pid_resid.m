% diag_pid_resid Modal residues of the retained right half plane pole.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
wc = P.ctrl.bw; Tf = P.ctrl.rate_filter_tau_s;
lag.da = tf(1, [P.act.aileron.tau_s, 1]);
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag.da;
Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
Cp = ss(tf([Kp_p * Tf, Kp_p], [Tf, 1]));
Tp = feedback(Cp * Gp, 1);
np = size(Tp.A, 1);
Gphi = ss([Tp.A, zeros(np, 1); Tp.C, 0], [Tp.B; Tp.D], [zeros(1, np), 1], 0);

for kp = [0.5, 2, 6, 20]
   C = ss(tf([kp, kp * 0.5 * wc.att_phi_rad_s], [1, 0]));
   L = C * Gphi;
   fprintf('\n=== phi Kp %g ===\n', kp);
   local_modes(L);
end

Kp_q = 1 / abs(freqresp(ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * tf(1, [P.act.elevator.tau_s, 1]), ...
   1i * wc.rate_q_rad_s));
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * tf(1, [P.act.elevator.tau_s, 1]);
Tq = feedback(ss(tf([Kp_q * Tf, Kp_q], [Tf, 1])) * Gq, 1);
nq = size(Tq.A, 1);
Gtheta = ss([Tq.A, zeros(nq, 1); Tq.C, 0], [Tq.B; Tq.D], [zeros(1, nq), 1], 0);
for Kp = [0.1, 0.4, 0.8, 1.6]
   for sgn = [1, -1]
      C = ss(tf([sgn * Kp, sgn * Kp * 0.5 * wc.att_theta_rad_s], [1, 0]));
      fprintf('\n=== theta Kp %g sgn %+d ===\n', Kp, sgn);
      local_modes(C * Gtheta);
   end
end

function local_modes(L)
CL = feedback(L, 1);
[A, B, C, D] = ssdata(CL); %#ok<ASGLU>
p = eig(A);
[V, Dm] = eig(A);
W = inv(V);
res = zeros(numel(p), 1);
for k = 1:numel(p)
   res(k) = abs(C * V(:, k) * W(k, :) * B);
end
[~, ord] = sort(real(p), 'descend');
tot = sum(res);
for k = ord(1:min(4, numel(ord)))'
   fprintf('  pole %+12.6e %+12.6e i  residue %10.3e  share %10.3e\n', ...
      real(p(k)), imag(p(k)), res(k), res(k) / tot);
end
fprintf('  residue total %.4e, largest %.4e\n', tot, max(res));
w = logspace(-4, 4, 400);
H = squeeze(freqresp(CL, 1i * w));
fprintf('  |H| over 1e-4..1e4 rad/s: min %.4e max %.4e\n', min(abs(H)), max(abs(H)));
end
