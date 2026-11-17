function [T, info] = prop_thrust_from_throttle(P, throttle, V)
% PROP_THRUST_FROM_THROTTLE Per-motor thrust from throttle setting and speed.
%
% The single authoritative propulsion map: the plant, the trim solve and the
% control allocation all call it, so they cannot disagree about how throttle
% becomes thrust
%
% Throttle is the fraction of peak shaft power, P_shaft = throttle^p *
% P_shaft_peak with p = P.prop.throttle_exponent. Momentum theory then gives
%   static:            T0 = (P_shaft*FM*sqrt(2*rho*A))^(2/3)  =>  T ~ P_shaft^(2/3)
%   advance speed V:   P_shaft = 2*rho*A*v_i*(V + v_i)^2,  T = 2*rho*A*v_i*(V+v_i)
%   thrust therefore grows as throttle^(2p/3), slower than power
%   the solve for v_i is monotone in shaft power, so bisection converges
%
% Usage:
%   T = prop_thrust_from_throttle(P, throttle, V)
%   [T, info] = prop_thrust_from_throttle(P, throttle, V)
%     throttle  0..1 scalar or array
%     V         m/s scalar or array, same size or scalar
%     T         N per motor
%     info      throttle_used, throttle_exponent, induced_velocity_mps

if nargin < 3
    V = 0;
end
throttle = min(max(throttle, P.prop.esc.min_throttle), P.prop.esc.max_throttle);

% Broadcast throttle and speed to a common shape up front; leaving the power
% vector scalar while the speed vector is not kills the loop below at k = 2
n = max(numel(throttle), numel(V));
if numel(throttle) ~= n && numel(throttle) ~= 1
    error('prop_thrust_from_throttle:Size', ...
        'throttle has %d elements, speed has %d, and neither is scalar', ...
        numel(throttle), numel(V));
end
if numel(V) ~= n && numel(V) ~= 1
    error('prop_thrust_from_throttle:Size', ...
        'throttle has %d elements, speed has %d, and neither is scalar', ...
        numel(throttle), numel(V));
end
% Broadcast a scalar argument to the common length and leave an array alone;
% V = V(ones(1,n)) would collapse an array to V(1) repeated
throttle = reshape(throttle, 1, []);
V = reshape(V, 1, []);
if isscalar(throttle)
    throttle = throttle(ones(1, n));
end
if isscalar(V)
    V = V(ones(1, n));
end

p = P.prop.throttle_exponent;
x = throttle .^ p;

rho = P.env.rho_sl;
A = P.prop.disk_area;
P_shaft = x * P.prop.P_shaft_peak;
P_ideal = P_shaft * P.prop.power.figure_of_merit;

% Static closed form, used directly at V = 0 and as the bracket scale elsewhere
T = zeros(size(x));
vi = zeros(size(x));
Vs = V;
for k = 1:numel(x)
    if P_ideal(k) <= 0
        T(k) = 0;
        vi(k) = 0;
        continue;
    end
    vik = local_solve_vi(P_ideal(k), Vs(k), rho, A);
    vi(k) = vik;
    T(k) = 2 * rho * A * vik * (Vs(k) + vik);
end

if nargout > 1
    info = struct('throttle_used', throttle, 'throttle_exponent', 2 * p / 3, ...
        'induced_velocity_mps', vi, 'P_shaft_W', P_shaft);
end
end

%% Local helpers
function vi = local_solve_vi(P_ideal, V, rho, A)
% Bisection on v_i for P_ideal = 2*rho*A*v_i*(V + v_i)^2, which is monotone
% increasing in v_i; bracket first, then 200 halvings
if V <= 0
    % Closed form avoids a needless iteration at the static point
    vi = (P_ideal / (2 * rho * A))^(1/3);
    return;
end
lo = 0;
hi = (P_ideal / (2 * rho * A))^(1/3) + 1;
guard = 0;
while 2 * rho * A * hi * (V + hi)^2 < P_ideal
    hi = hi * 2;
    guard = guard + 1;
    if guard > 200
        error('prop_thrust_from_throttle:Bracket', 'bisection bracket failed to expand');
    end
end
for iter = 1:200
    mid = 0.5 * (lo + hi);
    if 2 * rho * A * mid * (V + mid)^2 < P_ideal
        lo = mid;
    else
        hi = mid;
    end
end
vi = 0.5 * (lo + hi);
end
