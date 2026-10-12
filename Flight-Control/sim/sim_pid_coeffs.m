function c = sim_pid_coeffs(g, ts)
% SIM_PID_COEFFS Discrete coefficients of one parallel PID loop.
%
%   c = sim_pid_coeffs(gains.q, P.sim.Ts_ctrl)
%   c.Kp, c.Ki, c.Dnum, c.Dden, c.Ts
%
% Continuous law, the one control/design_pid.m designs
%   u = Kp*e + Ki*int(e) + Kd*s/(Tf*s + 1) * e
% The proportional and integral parts are implemented by a Gain and a
% Discrete-Time Integrator, the filtered derivative by one Discrete Transfer Fcn
% obtained with the Tustin substitution s = (2/Ts)*(1 - z^-1)/(1 + z^-1):
%   D(z) = [2*Kd/Ts, -2*Kd/Ts] / [1 + 2*Tf/Ts, -1 + 2*Tf/Ts]
% Both vectors are normalised by the leading denominator term, which the
% Discrete Transfer Fcn block requires to be non-zero.
%
% The rate loops carry Kd = 0 from the design, and then Dnum is the scalar zero
% so the block is a pure zero rather than a two-tap cancellation.

if nargin < 2
   ts = evalin('base', 'sim_ts_ctrl');
end
c = struct();
c.Kp = g.Kp;
c.Ki = g.Ki;
c.Ts = ts;
if abs(g.Kd) < eps
   c.Dnum = 0;
   c.Dden = 1;
   return;
end
num = [2 * g.Kd / ts, -2 * g.Kd / ts];
den = [1 + 2 * g.Tf / ts, -1 + 2 * g.Tf / ts];
c.Dnum = num / den(1);
c.Dden = den / den(1);
end
