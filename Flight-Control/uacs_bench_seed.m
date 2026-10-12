function uacs_bench_seed(mdl)
% UACS_BENCH_SEED 把 uacs_go.m 的操纵量初值推进台架模型的字面值参数。
%
% 挂在台架模型的 PreLoadFcn 里，紧跟 uacs_go 之后。
%
% 为什么需要这一步：Dashboard 控件只驱动得了"值是字面量"的块参数。绑基础工作区变量是
% 假通路——那样绑能建起来、isBound 也返回真、控件自己的 setValue 也改得动那个变量，但
% 仿真运行中改那个变量，仿真完全不理。实测见 verify/bench_live_check.m 的 A 与 B。
% 所以操纵量必须落成字面量，代价是初值没法在运行时由变量决定。折中办法就是这一步。
%
% 为什么挂 PreLoadFcn 而不是 InitFcn：InitFcn 每次按 Run 都会跑，那样用户拖好的杆位会
% 在按 Run 的瞬间被重置回 uacs_go 的值。挂在打开模型时，一个会话里拖过的杆位就留着。
%
% mdl 由调用方以字面量传入，不靠 bdroot：PreLoadFcn 执行时当前系统未必已经是这个模型。

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
