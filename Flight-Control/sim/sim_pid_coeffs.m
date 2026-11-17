function c = sim_pid_coeffs(g, ts)
% SIM_PID_COEFFS Discrete coefficients of one parallel PID loop
%
%   g    gain struct with fields Kp, Ki, Kd, Tf
%   ts   sample time in s, default sim_ts_ctrl from the base workspace
%   c    c.Kp, c.Ki, c.Ts, and the filtered derivative as c.Dnum over c.Dden
%
% The derivative is discretised with the Tustin substitution, and both vectors
% are normalised by the leading denominator term, which the Discrete Transfer
% Fcn block requires to be non-zero
%
% With Kd = 0 the derivative is a pure zero, Dnum = 0 and Dden = 1

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
