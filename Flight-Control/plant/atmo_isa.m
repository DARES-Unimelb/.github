function [rho, T, p, a] = atmo_isa(P, h, dT_sl)
% ATMO_ISA International Standard Atmosphere, troposphere, from P.env constants.
%
%   [rho, T, p, a] = atmo_isa(P, h)
%   [rho, T, p, a] = atmo_isa(P, h, dT_sl)
%
%   h      geometric altitude, m, scalar or array, -1000..11000
%   dT_sl  temperature offset from ISA at sea level, K, default P.atmo.dT_sl
%   rho    density kg/m^3, T temperature K, p pressure Pa, a speed of sound m/s
%
% ISA troposphere, constant lapse rate and hydrostatic equilibrium:
%   T   = T_sl + dT_sl - L*h
%   p   = p_sl * (T/(T_sl + dT_sl))^(g/(L*R))
%   rho = rho_sl * (T_sl/(T_sl + dT_sl)) * (T/(T_sl + dT_sl))^(g/(L*R) - 1)
%   a   = sqrt(gamma_air * R * T)
%
%   dT_sl holds sea level pressure fixed and raises the whole temperature
%   profile, so a hot day at the same altitude is less dense; the
%   T_sl/(T_sl + dT_sl) factor carries that, it is not cosmetic
%   Sea level density is the tabulated P.env.rho_sl, not p_sl/(R*T_sl)

if nargin < 3 || isempty(dT_sl)
   dT_sl = P.atmo.dT_sl;
end

h = h(:);
assert(all(isfinite(h)), 'atmo_isa:Range', 'altitude must be finite');
assert(all(h >= -1000) && all(h <= P.env.h_trop_m), 'atmo_isa:Range', ...
   sprintf('altitude %.1f m outside the troposphere model, valid -1000..%.0f m', ...
   min(h), P.env.h_trop_m));

% Below sea level the profile is clamped to the sea level state, not extrapolated
h_eff = max(h, 0);
expo = P.env.g / (P.env.L_isa * P.env.R_air);
T = P.env.T_sl + dT_sl - P.env.L_isa * h_eff;
ratio = T / (P.env.T_sl + dT_sl);
rho = P.env.rho_sl * (P.env.T_sl / (P.env.T_sl + dT_sl)) * ratio .^ (expo - 1);
p = P.env.p_sl * ratio .^ expo;
a = sqrt(P.env.gamma_air * P.env.R_air * T);
end
