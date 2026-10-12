function n = sim_push_vars(S)
% SIM_PUSH_VARS Put a struct's fields into the base workspace, and count them.
%
% One MATLAB session is shared by several members and there is one base
% workspace. A variable pushed at the start of a job can be gone by the time the
% solver asks for it, because somebody else's probe or control run cleared the
% workspace in between. The symptom is deceptive: the plant reports that it
% cannot find P, which reads like a modelling fault rather than a workspace one.
%
% Every simulation therefore re-pushes what it needs from inside the same
% function that calls sim, so the window between the push and the first
% derivative evaluation is as small as it can be made.

f = fieldnames(S);
for k = 1:numel(f)
   assignin('base', f{k}, S.(f{k}));
end
n = numel(f);
end
