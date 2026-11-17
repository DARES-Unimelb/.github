% diag_pid_outer Debug the outer loop state-space construction.
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
fprintf('Gp states %d, Cp states %d\n', size(Gp.A, 1), size(Cp.A, 1));
Tp = feedback(Cp * Gp, 1);
fprintf('Tp poles: %s\n', mat2str(sort(pole(Tp), 'ComparisonMethod', 'real').', 6));
fprintf('Tp size: A %dx%d B %dx%d C %dx%d D %dx%d\n', size(Tp.A), size(Tp.B), size(Tp.C), size(Tp.D));
np = size(Tp.A, 1);
A = [Tp.A, zeros(np, 1); Tp.C, 0];
B = [Tp.B; Tp.D];
C = [zeros(1, np), 1];
Gphi = ss(A, B, C, 0);
fprintf('Gphi poles: %s\n', mat2str(sort(pole(Gphi), 'ComparisonMethod', 'real').', 6));
fprintf('Gphi zeros: %s\n', mat2str(sort(zero(Gphi), 'ComparisonMethod', 'real').', 6));
fprintf('Gphi dc gain: %g, tf check %g\n', dcgain(Gphi), dcgain(tf(Gphi)));
for kp = [0.5, 2, 8]
   for f = [0, 0.5]
      Cc = ss(tf([kp, kp * f * wc.att_phi_rad_s], [1, 0]));
      L = Cc * Gphi;
      CL = feedback(L, 1);
      p = pole(CL);
      fprintf('kp %6.3f f %4.2f worst %12.4e n %d poles %s\n', kp, f, max(real(p)), numel(p), ...
         mat2str(sort(p, 'ComparisonMethod', 'real').', 5));
   end
end
