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
% Equations, ISA troposphere (lapse rate constant, hydrostatic equilibrium):
%   T   = T_sl + dT_sl - L*h
%   p   = p_sl * (T/(T_sl + dT_sl))^(g/(L*R))
%   rho = rho_sl * (T_sl/(T_sl + dT_sl)) * (T/(T_sl + dT_sl))^(g/(L*R) - 1)
%   a   = sqrt(gamma_air * R * T)
% Sea level density is the tabulated P.env.rho_sl, not p_sl/(R*T_sl) which is
% 1.22494; the tabulated 1.225 is the standard's own value and is what every
% other constant in this project is referenced to. At dT_sl = 0 the T_sl/(T_sl +
% dT_sl) factor is exactly 1 and the three relations collapse to the standard
% form, which is what makes the Aerospace Blockset comparison bit exact.
%
% dT_sl holds the sea level PRESSURE fixed and raises the whole temperature
% profile, the usual "ISA + dT" reading: a hot day at the same geometric altitude
% is less dense, because the column is warmer and therefore taller. Shifting the
% temperature without the density factor would make it denser, which is the wrong
% sign, so the factor is not cosmetic.
%
% Constants and form are the ones the Aerospace Blockset ISA Atmosphere Model
% block uses: R = 287.0531 J/(kg*K). With R = 287.05 the density at 150 m comes
% out 1.9e-7 low, which is a density-law disagreement rather than round-off, so
% the value is fixed by measurement, not by taste. plant/flow_state.m is the only
% caller in flight; the check is results/atmo_wind_check.mat, which compares
% against the block itself at 0, 150 and 1000 m.
%
% The stratosphere is not implemented. h_max_m is 400 m, so an aircraft that
% asks for more than 11 km gets an error instead of an extrapolated density.

if nargin < 3 || isempty(dT_sl)
   dT_sl = P.atmo.dT_sl;
end

h = h(:);
assert(all(isfinite(h)), 'atmo_isa:Range', 'altitude must be finite');
assert(all(h >= -1000) && all(h <= P.env.h_trop_m), 'atmo_isa:Range', ...
   sprintf('altitude %.1f m outside the troposphere model, valid -1000..%.0f m', ...
   min(h), P.env.h_trop_m));

% Below sea level the profile is CLAMPED to the sea level state rather than
% extrapolated. Two reasons, and the second is the deciding one.
%   Physical: extrapolating the lapse rate downwards gives 1.2309 kg/m^3 at -50 m,
%   which says the air is 0.5 per cent DENSER than at sea level on a standard day.
%   That is not a defensible physical statement about a terrain following
%   reference height, it is an artefact of running a troposphere formula backwards.
%   Consistency: the Aerospace Blockset ISA Atmosphere Model block saturates its
%   altitude input at 0 m and returns 1.225000 at every negative height. Clamping
%   makes both models agree on the whole declared range, so the negative branch
%   stops being a documented divergence that every future reader has to be warned
%   about. The property that matters is unchanged: rho(h = 0) is exactly
%   P.env.rho_sl, so the constant density comparison stays bit exact.
h_eff = max(h, 0);
expo = P.env.g / (P.env.L_isa * P.env.R_air);
T = P.env.T_sl + dT_sl - P.env.L_isa * h_eff;
ratio = T / (P.env.T_sl + dT_sl);
rho = P.env.rho_sl * (P.env.T_sl / (P.env.T_sl + dT_sl)) * ratio .^ (expo - 1);
p = P.env.p_sl * ratio .^ expo;
a = sqrt(P.env.gamma_air * P.env.R_air * T);
end
