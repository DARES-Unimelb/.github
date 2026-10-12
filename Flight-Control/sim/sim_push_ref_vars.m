function S = sim_push_ref_vars(P, x0, h_isa, h_from_state)
% SIM_PUSH_REF_VARS Push the base workspace variables model/uacs_plant_ref.slx reads.
%
% The reference model holds no literal numbers: every block parameter is one of
% these names, so a run selects the initial condition and the atmosphere without
% editing the model.
%
%   ref_x0_vb, ref_x0_om, ref_x0_eul, ref_x0_pos
%        x0 split into the four 6DOF initial condition parameters
%   ref_mass, ref_inertia
%        fixed mass and the inertia tensor about the CG, Ixz sign included
%   ref_h_isa
%        altitude the ISA Atmosphere Model block is evaluated at, m, used when
%        ref_h_from_state is 0
%   ref_h_from_state
%        0 evaluates the ISA block at the frozen ref_h_isa, 1 evaluates it at the
%        geometric altitude the aircraft is at, -Xe(3), matching what
%        plant/flow_state.m does with P.atmo.h_from_state true

if nargin < 2 || isempty(x0)
   x0 = evalin('base', 'sim_x0');
end
if nargin < 3 || isempty(h_isa)
   h_isa = -x0(12);
end
if nargin < 4 || isempty(h_from_state)
   h_from_state = 0;
end
x0 = x0(:);
assert(numel(x0) == 12, 'sim_push_ref_vars:Size', 'initial state must have 12 elements');

% The 6DOF (Euler Angles) block wants the initial conditions in its own order:
% inertial position, body velocity, Euler angles, body rates. Careful: the
% velocity is body axes while the position is inertial axes, and both are NED.
S = struct();
S.ref_x0_pos = x0(10:12);
S.ref_x0_vb = x0(1:3);
S.ref_x0_eul = x0(7:9);
S.ref_x0_om = x0(4:6);
S.ref_mass = P.mass.m;
S.ref_inertia = [P.mass.Ixx_cg, 0, -P.mass.Ixz_cg; ...
                 0, P.mass.Iyy_cg, 0; ...
                 -P.mass.Ixz_cg, 0, P.mass.Izz_cg];
S.ref_h_isa = h_isa;
S.ref_h_from_state = double(h_from_state ~= 0);

f = fieldnames(S);
for k = 1:numel(f)
   assignin('base', f{k}, S.(f{k}));
end
end
