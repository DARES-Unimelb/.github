function bench_stick_plots()
% BENCH_STICK_PLOTS 满杆矩阵的图。读 results/bench_stick_traj.mat，写 results/figs/*.png。
%
% 图上的标签一律用英文：MATLAB 默认字体渲染中文有缺字风险，图要给人看，不能出现方框。
%
% 五张图
%   stick_single_roll.png    单轴高低满杆，PID 与 LQR 两列，指令跟踪、速率、舵面、空速高度
%   stick_single_pitch.png   同上，俯仰轴
%   stick_single_yaw.png     同上，偏航轴
%   stick_cross.png          单轴满杆时另外两个轴的峰值响应，看交叉耦合
%   stick_combo.png          两轴与三轴组合代表工况的舵面时间历程
%   stick_chatter.png        舵面换向次数与主频，量化颤振
%
% 本文件与 verify/bench_stick_figs.m 的区别：那个版本取姿态通道的下标算错了，pitch 那一列
% 会越界。函数在会话里被调用过就按名字缓存，改内容不生效，所以另起一个名字。

root = fileparts(fileparts(mfilename('fullpath')));
S = load(fullfile(root, 'results', 'bench_stick_traj.mat'));
traj = S.traj;
out = fullfile(root, 'results', 'figs');
if ~exist(out, 'dir')
   mkdir(out);
end
R2D = 180 / pi;
axes_name = {'roll', 'pitch', 'yaw'};

for a = 1:3
   f = figure('Visible', 'off', 'Position', [60 40 1180 980]);
   for c = 0:1
      id = 1 + 2 * (a - 1) + 1;
      pos = local_get(traj, id, c);
      neg = local_get(traj, id + 1, c);
      if pos == 0 || neg == 0
         continue
      end
      subplot(4, 2, 1 + c);
      local_plot_cmd(traj(pos), traj(neg), a, R2D, axes_name{a}, c);
      subplot(4, 2, 3 + c);
      local_plot_rate(traj(pos), traj(neg), a, R2D, axes_name{a});
      subplot(4, 2, 5 + c);
      local_plot_surf(traj(pos), traj(neg));
      subplot(4, 2, 7 + c);
      local_plot_vh(traj(pos), traj(neg));
   end
   exportgraphics(f, fullfile(out, sprintf('stick_single_%s.png', axes_name{a})), 'Resolution', 110);
   close(f);
end

f = figure('Visible', 'off', 'Position', [60 40 1100 420]);
for c = 0:1
   subplot(1, 2, c + 1);
   vals = zeros(6, 2);
   lab = cell(1, 6);
   for a = 1:3
      for s = 0:1
         id = 1 + 2 * (a - 1) + 1 + s;
         k = local_get(traj, id, c);
         if k == 0
            continue
         end
         tr = traj(k);
         idx = 2 * (a - 1) + 1 + s;
         lab{idx} = sprintf('%s %s', axes_name{a}, local_sgn(s));
         others = setdiff(1:3, a);
         peaks = zeros(1, 2);
         for j = 1:2
            if others(j) == 1
               peaks(j) = max(abs(tr.att(2, :) * R2D));
            elseif others(j) == 2
               peaks(j) = max(abs(tr.att(4, :) * R2D));
            else
               peaks(j) = max(abs(tr.rate(6, :) * R2D));
            end
         end
         vals(idx, :) = peaks;
      end
   end
   bar(vals);
   set(gca, 'XTickLabel', lab);
   legend({'peak of other axis 1', 'peak of other axis 2'}, 'Location', 'northwest');
   title(sprintf('%s  cross coupling peak', local_ctrl(c)));
   ylabel('deg or deg/s');
   grid on;
end
exportgraphics(f, fullfile(out, 'stick_cross.png'), 'Resolution', 110);
close(f);

f = figure('Visible', 'off', 'Position', [60 40 1180 760]);
sel = [8 10 12 14 16 18 20 22 24 26];
for c = 0:1
   for j = 1:numel(sel)
      k = local_get(traj, sel(j), c);
      if k == 0
         continue
      end
      tr = traj(k);
      subplot(5, 4, c * 10 + j);
      plot(tr.t_act, tr.act(1, :) * R2D, 'LineWidth', 0.8);
      hold on;
      plot(tr.t_act, tr.act(2, :) * R2D, 'LineWidth', 0.8);
      plot(tr.t_act, tr.act(3, :) * R2D, 'LineWidth', 0.8);
      grid on;
      xlim([0 tr.t_end]);
      ylim([-27 27]);
      title(sprintf('%d %s %s', tr.case_id, local_ctrl(c), tr.label), 'FontSize', 7);
      set(gca, 'FontSize', 6);
   end
end
exportgraphics(f, fullfile(out, 'stick_combo.png'), 'Resolution', 110);
close(f);

f = figure('Visible', 'off', 'Position', [60 40 1100 420]);
ch = zeros(numel(traj), 5);
for k = 1:numel(traj)
   tr = traj(k);
   ch(k, 1) = local_chat(tr.t_act, tr.act(2, :));
   ch(k, 2) = local_chat(tr.t_act, tr.act(1, :));
   ch(k, 3) = local_chat(tr.t_act, tr.act(3, :));
   ch(k, 4) = local_fdom(tr.t_fast, tr.rate(4, :) * R2D);
   ch(k, 5) = local_fdom(tr.t_act, tr.act(4, :));
end
subplot(1, 2, 1);
bar(ch(:, 1:3));
legend({'elevator', 'aileron', 'rudder'}, 'Location', 'northwest');
xlabel('record index in file'); ylabel('reversals in last 4 s');
title('surface chatter'); grid on;
subplot(1, 2, 2);
bar(ch(:, 4:5));
legend({'q dominant Hz', 'throttle dominant Hz'}, 'Location', 'northwest');
xlabel('record index in file'); ylabel('Hz');
title('dominant frequency in last 4 s'); grid on;
exportgraphics(f, fullfile(out, 'stick_chatter.png'), 'Resolution', 110);
close(f);

fprintf('图写到 %s\n', out);
end

% ------------------------------------------------------------------------
function k = local_get(traj, id, ctrl)
k = 0;
for i = 1:numel(traj)
   if traj(i).case_id == id && traj(i).ctrl == ctrl
      k = i;
      return
   end
end
end

function s = local_ctrl(c)
if c == 0
   s = 'PID';
else
   s = 'LQR';
end
end

function s = local_sgn(b)
if b == 0
   s = 'plus';
else
   s = 'minus';
end
end

function local_plot_cmd(tp, tn, a, R2D, nm, c)
if a == 3
   cp = tp.rate(5, :) * R2D;
   cn = tn.rate(5, :) * R2D;
   yp = tp.rate(6, :) * R2D;
   yn = tn.rate(6, :) * R2D;
   yl = 'yaw rate deg/s';
else
   ir = 2 * a - 1;
   cp = tp.att(ir, :) * R2D;
   cn = tn.att(ir, :) * R2D;
   yp = tp.att(ir + 1, :) * R2D;
   yn = tn.att(ir + 1, :) * R2D;
   yl = [nm ' deg'];
end
plot(tp.t_fast, cp, '--', 'LineWidth', 1.0);
hold on;
plot(tp.t_fast, yp, 'LineWidth', 1.4);
plot(tn.t_fast, cn, '--', 'LineWidth', 1.0);
plot(tn.t_fast, yn, 'LineWidth', 1.4);
grid on;
ylabel(yl);
title(sprintf('%s  %s full stick, solid actual, dashed command', local_ctrl(c), nm));
legend({'cmd +', 'act +', 'cmd -', 'act -'}, 'Location', 'best');
xlim([0 tp.t_end]);
end

function local_plot_rate(tp, tn, a, R2D, nm)
ir = 2 * a;
plot(tp.t_fast, tp.rate(ir, :) * R2D, 'LineWidth', 1.0);
hold on;
plot(tn.t_fast, tn.rate(ir, :) * R2D, 'LineWidth', 1.0);
grid on;
ylabel([nm ' rate deg/s']);
title(sprintf('%s rate, plus and minus cases', nm));
legend({'plus case', 'minus case'}, 'Location', 'best');
xlim([0 tp.t_end]);
end

function local_plot_surf(tp, tn)
plot(tp.t_act, tp.act(1, :) * 180 / pi, 'LineWidth', 1.0);
hold on;
plot(tp.t_act, tp.act(2, :) * 180 / pi, 'LineWidth', 1.0);
plot(tp.t_act, tp.act(3, :) * 180 / pi, 'LineWidth', 1.0);
plot(tn.t_act, tn.act(1, :) * 180 / pi, '--', 'LineWidth', 0.8);
plot(tn.t_act, tn.act(2, :) * 180 / pi, '--', 'LineWidth', 0.8);
plot(tn.t_act, tn.act(3, :) * 180 / pi, '--', 'LineWidth', 0.8);
grid on;
ylabel('surface deg');
title('aileron, elevator, rudder');
legend({'da +', 'de +', 'dr +', 'da -', 'de -', 'dr -'}, 'Location', 'best');
xlim([0 tp.t_end]);
end

function local_plot_vh(tp, tn)
yyaxis left;
plot(tp.t_fast, tp.inst(1, :), 'LineWidth', 1.2);
hold on;
plot(tn.t_fast, tn.inst(1, :), 'LineWidth', 1.2);
ylabel('V m/s');
yyaxis right;
plot(tp.t_fast, tp.inst(2, :), 'LineWidth', 1.0);
plot(tn.t_fast, tn.inst(2, :), 'LineWidth', 1.0);
ylabel('h m');
grid on;
title('airspeed and altitude');
legend({'V +', 'V -', 'h +', 'h -'}, 'Location', 'best');
xlim([0 tp.t_end]);
end

function c = local_chat(t, x)
k = find(t >= t(end) - 4, 1);
if isempty(k)
   k = 1;
end
d = diff(x(k:end));
d(abs(d) < 1e-9) = 0;
c = sum(diff(sign(d)) ~= 0);
end

function f = local_fdom(t, x)
k = find(t >= t(end) - 4, 1);
if isempty(k)
   k = 1;
end
y = x(k:end) - mean(x(k:end));
dt = median(diff(t));
n = numel(y);
if n < 8
   f = 0;
   return
end
Y = abs(fft(y));
Y = Y(2:floor(n / 2));
if isempty(Y)
   f = 0;
   return
end
[~, i] = max(Y);
f = i / (n * dt);
end
