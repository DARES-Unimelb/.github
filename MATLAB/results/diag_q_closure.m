% diag_q_closure Resolve the claimed linear/nonlinear contradiction on the q loop.
% Three independent checks, none of which trusts the linearisation code blindly:
%   1. the closed-loop q coefficient, by hand, from A, B and the published gains
%   2. the B_long row-1/row-2 discrepancy the lead saw, as a basis transform
%   3. the same two partial derivatives measured on the nonlinear object directly
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_q_closure.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);

fprintf(fid, '=== 1. state and input ordering ===\n');
fprintf(fid, 'long states : %s\n', strjoin(lin.long_state_names(:).', ', '));
fprintf(fid, 'long inputs : %s\n', strjoin(lin.long_input_names(:).', ', '));
fprintf(fid, 'A_long row 3 = %s\n', mat2str(lc.A_long(3, :), 8));
fprintf(fid, '  so A_long(3,1)=dqdot/du %+.6f, A_long(3,2)=dqdot/dw %+.6f,\n', ...
   lc.A_long(3, 1), lc.A_long(3, 2));
fprintf(fid, '     A_long(3,3)=dqdot/dq %+.6f, A_long(3,4)=dqdot/dtheta %+.6f\n', ...
   lc.A_long(3, 3), lc.A_long(3, 4));
fprintf(fid, 'B_long(:,1)  = %s  [d/du for u w q theta]\n', mat2str(lc.B_long(:, 1).', 8));

fprintf(fid, '\n=== 2. the published q controller and its partial with respect to q ===\n');
S = load(fullfile(root, 'results', 'gains_baseline.mat'));
gq = S.gains_pid.q;
Kp = gq.Kp;
fprintf(fid, 'gains_pid.q.Kp = %+.6f, sign = %+d\n', Kp, gq.sign);
fprintf(fid, 'model wiring: err = q_c - q, de = Kp*err\n');
fprintf(fid, '  de = (%+.6f)*(q_c - q)\n', Kp);
fprintf(fid, '  d(de)/dq = -Kp = %+.6f   <-- the sign that decides the verdict\n', -Kp);
Aqq = lc.A_long(3, 3);
Bqe = lc.B_long(3, 1);
contrib = Bqe * (-Kp);
fprintf(fid, 'closed-loop dqdot/dq = A_long(3,3) + B_long(3,1)*d(de)/dq\n');
fprintf(fid, '                     = %+.6f + (%+.6f)*(%+.6f) = %+.6f\n', Aqq, Bqe, -Kp, Aqq + contrib);
fprintf(fid, 'verdict: %s  (the lead computed %+.4f by using d(de)/dq = Kp instead of -Kp)\n', ...
   ternary(Aqq + contrib < 0, 'STABLE', 'UNSTABLE'), Aqq + Bqe * Kp);

fprintf(fid, '\n=== 3. the B_long row 1 and row 2 discrepancy, as a basis transform ===\n');
a0 = deg2rad(trim.alpha_deg);
V0 = trim.V_mps;
fprintf(fid, 'alpha0 = %.6f rad, V0 = %.6f m/s\n', a0, V0);
% lc.B_long is ALREADY the contract basis; the raw linearize_at B is body axes.
% Go body -> contract, which is what lin_contract does.
i_du = 1; i_dw = 3;      % body state order u, w, q, theta
i_de = 2;                % body input order de, dT
Bbody = [lin.B(i_du, i_de); lin.B(i_dw, i_de)];
du_de = Bbody(1);
dw_de = Bbody(2);
dV_de = cos(a0) * du_de + sin(a0) * dw_de;
dalpha_de = (-sin(a0) * du_de + cos(a0) * dw_de) / V0;
fprintf(fid, 'raw body frame (linearize_at)      du/de = %+.6f   dw/de = %+.6f\n', du_de, dw_de);
fprintf(fid, 'contract frame (computed from body) dV/de = %+.6f   dalpha/de = %+.6f\n', dV_de, dalpha_de);
fprintf(fid, 'contract frame (lin_contract gives) dV/de = %+.6f   dalpha/de = %+.6f\n', ...
   lc.B_long(1, 1), lc.B_long(2, 1));
fprintf(fid, 'agreement: dV %.3e, dalpha %.3e\n', ...
   abs(dV_de - lc.B_long(1, 1)), abs(dalpha_de - lc.B_long(2, 1)));
fprintf(fid, 'the lead compared -0.254243 / -1.007097 against -0.387497 / -25.1757:\n');
fprintf(fid, '  -0.254243 is dV/de  (contract), -0.387497 is du/de  (body)\n');
fprintf(fid, '  -1.007097 is dalpha/de (contract), -25.1757 is dw/de (body)\n');
fprintf(fid, '  row 2 ratio %.6f against V0 = %.6f: the V0 factor is the whole difference\n', ...
   abs(dw_de / lc.B_long(2, 1)), V0);

fprintf(fid, '\n=== 4. nonlinear finite difference, independent of linearize_at ===\n');
x0 = trim.x_star(:);
u0 = trim.u_star(:);
d0 = eom_6dof(0, x0, u0, P);
fprintf(fid, 'trim qdot at u_star = %.6e (must be ~0)\n', d0(5));
h = 1e-7;
% dqdot/dq: perturb state 5
xp = x0; xp(5) = xp(5) + h;
xm = x0; xm(5) = xm(5) - h;
fd_q = (eom_6dof(0, xp, u0, P) - eom_6dof(0, xm, u0, P)) / (2 * h);
% dqdot/dw: perturb state 3
xp = x0; xp(3) = xp(3) + h;
xm = x0; xm(3) = xm(3) - h;
fd_w = (eom_6dof(0, xp, u0, P) - eom_6dof(0, xm, u0, P)) / (2 * h);
% dqdot/dde: perturb actuator 2
up = u0; up(2) = up(2) + h;
um = u0; um(2) = um(2) - h;
fd_de = (eom_6dof(0, x0, up, P) - eom_6dof(0, x0, um, P)) / (2 * h);
fprintf(fid, 'nonlinear dqdot/dq   = %+.6f   linearisation A_long(3,3) = %+.6f   ratio %.6f\n', ...
   fd_q(5), Aqq, fd_q(5) / Aqq);
fprintf(fid, 'nonlinear dqdot/dw   = %+.6f   linearisation A_long(3,2) = %+.6f   ratio %.6f\n', ...
   fd_w(5), lc.A_long(3, 2), fd_w(5) / lc.A_long(3, 2));
fprintf(fid, 'nonlinear dqdot/dde  = %+.6f   linearisation B_long(3,1) = %+.6f   ratio %.6f\n', ...
   fd_de(5), Bqe, fd_de(5) / Bqe);
fprintf(fid, 'nonlinear closed-loop dqdot/dq = %+.6f + (%+.6f)*(%+.6f) = %+.6f\n', ...
   fd_q(5), fd_de(5), -Kp, fd_q(5) + fd_de(5) * (-Kp));
fclose(fid);
type(out);

function s = ternary(c, a, b)
if c
   s = a;
else
   s = b;
end
end
