function [V, alpha, beta, qbar, rho] = flow_state(P, x)
% FLOW_STATE Air data from the body-axis velocity states.
%
%   [V, alpha, beta, qbar] = flow_state(P, x)
%   [V, alpha, beta, qbar, rho] = flow_state(P, x)
%
%   V      true airspeed, m/s
%   alpha  angle of attack, rad, positive nose-up
%   beta   sideslip angle, rad, positive nose-right
%   qbar   dynamic pressure, Pa
%   rho    density used, kg/m^3
%
% x(1:3) is the AIR-RELATIVE velocity in body axes, not the ground velocity.
% The wind shift lives at the model boundary and is applied once there:
%   sim/sim_plant_rhs.m       x_air = x - C_nb*wind_ned before eom_6dof
%   sim/sim_plant_ref_aero.m  the same shift before aero_forces
% so this function must never subtract a wind of its own, and a caller that
% hands it a ground velocity gets air data for the wrong flow.
%
% Density comes from P.atmo.model:
%   'const'  P.env.rho_sl everywhere, the frozen rev 9 behaviour
%   'isa'    plant/atmo_isa.m at the density altitude, which is -x(12) when
%            P.atmo.h_from_state is true and P.atmo.h_ref when it is false.
%            pd is positive downwards, so -pd is the geometric altitude.
% A P saved before rev 10 carries no atmo.model and reads as 'const', so old
% result files still replay with the density they were measured at.

u = x(1);
v = x(2);
w = x(3);

V = sqrt(u^2 + v^2 + w^2);
V_safe = max(V, 1e-6);   % guard against division by zero at standstill

alpha = atan2(w, u);
beta = asin(max(-1, min(1, v / V_safe)));

model = 'const';
if isfield(P.atmo, 'model') && ~isempty(P.atmo.model)
   model = P.atmo.model;
end
switch model
   case 'const'
      rho = P.env.rho_sl;
   case 'isa'
      if ~isfield(P.atmo, 'h_from_state') || P.atmo.h_from_state
         h = -x(12);
      else
         h = P.atmo.h_ref;
      end
      rho = atmo_isa(P, h);
   otherwise
      error('flow_state:AtmoModel', 'unknown P.atmo.model ''%s''', model);
end

qbar = 0.5 * rho * V^2;
end
