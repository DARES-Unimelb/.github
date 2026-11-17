function uacs_bench_seed(mdl)
% UACS_BENCH_SEED Push the stick initial values from uacs_go.m into the bench block parameters.
%
% Set as the bench model PreLoadFcn, right after uacs_go. Dashboard controls only drive
% block parameters whose value is a literal, so the stick values have to be written in.
%
% mdl is passed explicitly: during PreLoadFcn the current system is not necessarily this model.

if nargin < 1 || isempty(mdl)
   error('uacs_bench_seed:NoModel', '要显式传模型名，不能靠 bdroot');
end
if ~bdIsLoaded(mdl)
   return
end
pairs = uacs_bench_seed_list();
n = 0;
for k = 1:size(pairs, 1)
   blk = [mdl '/' pairs{k, 1}];
   if getSimulinkBlockHandle(blk) <= 0
      continue
   end
   v = evalin('base', pairs{k, 2});
   assert(isnumeric(v) && isscalar(v), 'uacs_bench_seed:BadValue', ...
      '%s 对应的变量 %s 必须是标量数值，实际是 %s', blk, pairs{k, 2}, class(v));
   set_param(blk, 'Value', num2str(v, 17));
   n = n + 1;
end
fprintf('台架操纵量初值已从 uacs_go 推入 %d 个块\n', n);
end
