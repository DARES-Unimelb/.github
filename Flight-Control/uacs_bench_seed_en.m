function uacs_bench_seed_en(mdl)
% UACS_BENCH_SEED_EN 英文副本的操纵量初值推送，作用与 uacs_bench_seed 相同，路径表用英文块名。
%
% 挂在 model/uacs_flight_bench_en.slx 的 PostLoadFcn 里。
%
% 为什么另起一份而不是给 uacs_bench_seed 加参数：那个函数在本会话里被调用过，MATLAB 按名字
% 缓存了它，改文件不生效。另起一个名字一定读当前内容。
%
% 为什么必须这一步：Dashboard 控件只驱动得了「值是字面量」的块参数。绑基础工作区变量时绑定
% 能建起来、isValid 也返回真、控件的 setValue 也改得动那个变量，但仿真运行中改那个变量仿真
% 完全不理。所以操纵量必须落成字面量，代价是初值没法由变量在运行时决定。
%
% 挂在 PostLoadFcn 而不是 InitFcn：InitFcn 每次按 Run 都跑，会把用户拖好的杆位重置回
% uacs_go.m 的值。

if nargin < 1 || isempty(mdl)
   error('uacs_bench_seed_en:NoModel', '要显式传模型名，不能靠 bdroot');
end
if ~bdIsLoaded(mdl)
   return
end

pairs = {
   'Pilot/k_roll',              'sim_stick_roll'
   'Pilot/k_pitch',             'sim_stick_pitch'
   'Pilot/k_yaw',               'sim_stick_yaw'
   'Pilot/k_thr',               'sim_stick_thr'
   'Pilot/js_on',               'sim_js_on'
   'Guidance and Mode/mode_const', 'sim_flight_mode'
   'Controller Select/k_sel',   'sim_ctrl_sel'
   'Atmosphere and Wind/wind_e', 'sim_dist_wind_e'
   };

n = 0;
for k = 1:size(pairs, 1)
   blk = [mdl '/' pairs{k, 1}];
   if getSimulinkBlockHandle(blk) <= 0
      continue
   end
   v = evalin('base', pairs{k, 2});
   assert(isnumeric(v) && isscalar(v), 'uacs_bench_seed_en:BadValue', ...
      '%s 对应的变量 %s 必须是标量数值，实际是 %s', blk, pairs{k, 2}, class(v));
   set_param(blk, 'Value', num2str(v, 17));
   n = n + 1;
end
fprintf('bench stick seed pushed into %d blocks\n', n);
end
