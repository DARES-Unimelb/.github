function info = sim_alloc_wrap(P, M_cmd, T_cmd)
% SIM_ALLOC_WRAP Control allocation plus its own info struct.
%
%   info = sim_alloc_wrap(P, M_cmd, T_cmd)
%   info.act  5x1  [da de dr dth_l dth_r]
%   info.src  'allocate' when the delivered allocator was used
%
% Scenario scripts use this when they want allocate's diagnostics; the model
% path uses sim_alloc_act, which returns the bare five commands.

if nargin < 1 || isempty(P)
   P = evalin('base', 'P');
end
[da, de, dr, dth_l, dth_r] = sim_alloc_act(P, M_cmd, T_cmd);
info = struct();
info.act = [da; de; dr; dth_l; dth_r];
info.M_cmd = M_cmd(:);
info.T_cmd = T_cmd;
if sim_have('allocate')
   info.src = 'allocate';
   [~, ~, ~, ~, ~, a] = allocate(P, M_cmd, T_cmd);
   info.M_ach = a.M_ach;
   info.T_ach = a.T_ach;
   info.M_err_norm = a.M_err_norm;
   info.sat_any = a.sat_any;
   info.limits = a.limits;
else
   info.src = 'placeholder';
end
end
