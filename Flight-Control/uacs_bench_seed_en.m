function uacs_bench_seed_en(mdl)
% UACS_BENCH_SEED_EN English-model copy of uacs_bench_seed, using English block names.
%
% Set as the PostLoadFcn of model/uacs_flight_bench_en.slx. Dashboard controls only drive
% block parameters whose value is a literal, so the stick values have to be written in.
%
% mdl is passed explicitly: during PostLoadFcn the current system is not necessarily this model.

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
