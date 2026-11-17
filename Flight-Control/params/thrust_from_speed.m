function T = thrust_from_speed(P, V)
% THRUST_FROM_SPEED Per-motor thrust available at full throttle and speed V.
%
% Interpolates the momentum-theory curve built in uacs_twin_params, the single
% authoritative mapping from advance speed to available thrust
%
% Usage:
%   T = thrust_from_speed(P, V)     % V scalar or array, m/s; T same shape, N
%
%   Not a fitted ram-drag line: that slope feeds the thrust-speed coupling in A

Vtab = P.prop.thrust_speed_table_V;
Ttab = P.prop.thrust_speed_table_T;

Vv = max(V, 0);
T = interp1(Vtab, Ttab, Vv, 'linear', 'extrap');
% Extrapolation below the table start cannot be trusted, so clamp to the static
% value there; it only happens below 5 m/s, which is inside the stall margin
T(Vv <= Vtab(1)) = Ttab(1);
% No thrust above the speed where the propeller unloads completely
T(Vv >= Vtab(end)) = min(T(Vv >= Vtab(end)), Ttab(end));
end
