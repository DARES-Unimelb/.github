function S = sim_push_ref_vars(P, x0, h_isa, h_from_state)
% SIM_PUSH_REF_VARS Push the base workspace variables model/uacs_plant_ref.slx reads
%
%   P, x0        parameters and 12x1 initial state, x0 defaults to sim_x0
%   h_isa        ISA altitude in m, defaults to -x0(12)
%   h_from_state 1 evaluates ISA at the current state, 0 at h_isa, default 0
%   S            the pushed struct, also written to the base workspace
%
%   ref_x0_pos, ref_x0_vb, ref_x0_eul, ref_x0_om   x0 in the order the 6DOF
%       (Euler Angles) block wants
%   ref_mass, ref_inertia   mass and inertia tensor about the CG, Ixz sign kept
%   ref_h_isa, ref_h_from_state   the altitude pair above

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

% position is inertial axes and velocity is body axes, both NED
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
