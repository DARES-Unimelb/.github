% diag_pid_build Rebuild the outer loop plants from the raw sub-models.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
wc = P.ctrl.bw; Tf = P.ctrl.rate_filter_tau_s;

% ---- inner pitch rate loop, raw assembly ----
A1 = lc.A_long; B1 = lc.B_long(:, 1); C1 = [0 0 1 0]; D1 = 0;
lagc = ss(tf(1, [P.act.elevator.tau_s, 1]));
[A1, B1, C1, D1] = ssdata(ss(A1, B1, C1, D1) * lagc);
n1 = size(A1, 1);
Kp_q = 1 / abs(freqresp(ss(A1, B1, C1, D1), 1i * wc.rate_q_rad_s));
Ac = -1 / Tf; Bc = 1 / Tf; Cc = -Kp_q; Dc = Kp_q;
Acl = [A1 - B1 * (D1 * Cc + Dc * C1), -B1 * Cc; Bc * C1, Ac];
Bcl = [B1 * Dc; Bc];
Ccl = [C1, 0];
Dcl = D1 * Dc;
Tq = ss(Acl, Bcl, Ccl, Dcl);
fprintf('Tq poles: %s\n', mat2str(sort(pole(Tq), 'ComparisonMethod', 'real').', 6));
fprintf('Tq zeros: %s\n', mat2str(sort(zero(Tq), 'ComparisonMethod', 'real').', 6));

% ---- outer theta plant: Tq/s ----
n = size(Acl, 1);
At = [Acl, zeros(n, 1); Ccl, 0];
Bt = [Bcl; Dcl];
Ct = [zeros(1, n), 1];
Gtheta = ss(At, Bt, Ct, 0);
fprintf('Gtheta poles: %s\n', mat2str(sort(pole(Gtheta), 'ComparisonMethod', 'real').', 6));
fprintf('Gtheta zeros: %s\n', mat2str(sort(zero(Gtheta), 'ComparisonMethod', 'real').', 6));
fprintf('Gtheta rss %d\n', size(Gtheta.A, 1));

for sgn = [1, -1]
   for Kp = [0.05, 0.1, 0.2, 0.4, 0.8]
      Ki = 0.5 * wc.att_theta_rad_s * Kp;
      L = ss(tf([sgn * Kp, sgn * Ki], [1, 0])) * Gtheta;
      CL = feedback(L, 1);
      p = pole(CL);
      fprintf('sgn %+d Kp %6.3f worst %+12.5e n %d  top %s\n', sgn, Kp, max(real(p)), numel(p), ...
         mat2str(sort(real(p), 'descend').', 5));
   end
end

% ---- same for phi ----
A1 = lc.A_lat; B1 = lc.B_lat(:, 1); C1 = [0 1 0 0]; D1 = 0;
lagc = ss(tf(1, [P.act.aileron.tau_s, 1]));
[A1, B1, C1, D1] = ssdata(ss(A1, B1, C1, D1) * lagc);
Kp_p = 1 / abs(freqresp(ss(A1, B1, C1, D1), 1i * wc.rate_p_rad_s));
Ac = -1 / Tf; Bc = 1 / Tf; Cc = -Kp_p; Dc = Kp_p;
Acl = [A1 - B1 * (D1 * Cc + Dc * C1), -B1 * Cc; Bc * C1, Ac];
Bcl = [B1 * Dc; Bc];
Ccl = [C1, 0];
Tp = ss(Acl, Bcl, Ccl, D1 * Dc);
n = size(Acl, 1);
Gphi = ss([Acl, zeros(n, 1); Ccl, 0], [Bcl; D1 * Dc], [zeros(1, n), 1], 0);
fprintf('\nphi: Kp_p %.5f, Tp states %d\n', Kp_p, n);
for Kp = [0.5, 2, 6, 20]
   L = ss(tf([Kp, 0.5 * wc.att_phi_rad_s * Kp], [1, 0])) * Gphi;
   CL = feedback(L, 1);
   p = pole(CL);
   fprintf('phi Kp %6.3f worst %+12.5e n %d  top %s\n', Kp, max(real(p)), numel(p), ...
      mat2str(sort(real(p), 'descend').', 5));
end
