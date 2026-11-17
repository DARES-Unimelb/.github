% diag_pid_ss Closed-loop poles from the state-space loop, and gain sweeps.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
wc = P.ctrl.bw;
Tf = P.ctrl.rate_filter_tau_s;
lag.da = tf(1, [P.act.aileron.tau_s, 1]);
lag.de = tf(1, [P.act.elevator.tau_s, 1]);
lag.dr = tf(1, [P.act.rudder.tau_s, 1]);
lag.T = tf(1, [P.prop.esc.tau_s, 1]);
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag.da;
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag.de;
Gr = ss(lc.A_lat, lc.B_lat(:, 2), [0 0 1 0], 0) * lag.dr;
GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * lag.T;

% state-space closed loop for C = Kp + Ki/s + Kd*s/(Tf*s+1)
clp = @(C, G) eig([G.A - G.B * (C.D * G.C), -G.B * C.C; ...
                   G.C * C.B, C.A]);

fprintf('--- state-space vs tf closed loop, rate loop p ---\n');
Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
Css = ss(tf([Kp_p * Tf, Kp_p], [Tf, 1]));
fprintf('ss poles: %s\n', mat2str(sort(clp(Css, Gp), 'ComparisonMethod', 'real').', 6));
fprintf('tf poles: %s\n', mat2str(sort(pole(feedback(tf(Css) * Gp, 1)), 'ComparisonMethod', 'real').', 6));

fprintf('\n--- rate loop p sweep, C = Kp + Kd*s/(Tf*s+1) ---\n');
for kp = [0.5, 1, 2, 5, 10, 20]
   for kd = [0, 0.01, 0.05, 0.2]
      C = ss(tf([kp * Tf + kd, kp], [Tf, 1]));
      p = clp(C, Gp);
      fprintf('  Kp %6.3f Kd %6.3f worst %12.4e\n', kp, kd, max(real(p)));
   end
end

fprintf('\n--- rate loop q sweep ---\n');
for kp = [0.5, 1, 2, 4, 8, 16]
   for kd = [0, 0.01, 0.05, 0.2]
      C = ss(tf([kp * Tf + kd, kp], [Tf, 1]));
      fprintf('  Kp %6.3f Kd %6.3f worst %12.4e\n', kp, kd, max(real(clp(C, Gq))));
   end
end

fprintf('\n--- rate loop r sweep ---\n');
for kp = [0.2, 0.5, 0.7, 1, 1.5, 2, 4]
   for kd = [0, 0.05, 0.2, 0.5]
      C = ss(tf([kp * Tf + kd, kp], [Tf, 1]));
      fprintf('  Kp %6.3f Kd %6.3f worst %12.4e\n', kp, kd, max(real(clp(C, Gr))));
   end
end

fprintf('\n--- phi plant, PI, Kp sweep at zero_frac 0.5 of wc=3 ---\n');
Tp = feedback(ss(tf([Kp_p * Tf, Kp_p], [Tf, 1])) * Gp, 1);
Aphi = [Tp.A, zeros(size(Tp.A, 1), 1); Tp.C, 0];
Bphi = [Tp.B; Tp.D];
Gphi = ss(Aphi, Bphi, [0 0 0 1], 0);
for kp = [0.5, 1, 2, 5, 6.1, 8, 12, 20, 50]
   for f = [0, 0.5]
      C = ss(tf([kp, kp * f * wc.att_phi_rad_s], [1, 0]));
      fprintf('  Kp %6.3f f %4.2f worst %12.4e\n', kp, f, max(real(clp(C, Gphi))));
   end
end

fprintf('\n--- theta plant, PI, f 0.5 of wc=2.5 ---\n');
Kp_q = 1 / abs(freqresp(Gq, 1i * wc.rate_q_rad_s));
Tq = feedback(ss(tf([Kp_q * Tf, Kp_q], [Tf, 1])) * Gq, 1);
Gtheta = ss([Tq.A, zeros(size(Tq.A, 1), 1); Tq.C, 0], [Tq.B; Tq.D], [0 0 0 1], 0);
for sgn = [1, -1]
   for kp = [0.2, 0.5, 1, 2, 4, 8, 16, 32]
      C = ss(tf([sgn * kp, sgn * kp * 0.5 * wc.att_theta_rad_s], [1, 0]));
      fprintf('  sgn %+d Kp %6.3f worst %12.4e\n', sgn, kp, max(real(clp(C, Gtheta))));
   end
end

fprintf('\n--- V plant, PI, f 0.2 of wc=0.8 ---\n');
for sgn = [1, -1]
   for kp = [0.05, 0.1, 0.2, 0.5, 1, 2, 4]
      C = ss(tf([sgn * kp, sgn * kp * 0.2 * wc.speed_rad_s], [1, 0]));
      fprintf('  sgn %+d Kp %6.3f worst %12.4e\n', sgn, kp, max(real(clp(C, GV))));
   end
end
