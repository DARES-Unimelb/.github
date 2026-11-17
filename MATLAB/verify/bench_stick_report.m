function rep = bench_stick_report()
% BENCH_STICK_REPORT 满杆矩阵的结果整理：指标表、分组表与图。
%
% 读 results/bench_stick_traj.mat，写
%   results/bench_stick_metrics.csv  每个工况一行，列名就是指标名
%   docs/_work/stick_tables.md       分组排好的 markdown 表，供报告正文取用
%   results/figs/stick_*.png         轨迹图
%
% 通道口径（与 sim/sim_build_bench.m 的记录说明一致）
%   inst 七路  V m/s、h m、alpha deg、phi deg、theta deg、失速标志、r deg/s
%   att  四路  phi_c、phi、theta_c、theta，rad
%   rate 六路  p_c、p、q_c、q、r_c、r，rad/s
%   act  五路  da、de、dr rad，左油门、右油门 0 到 1
%
% 末段口径一律取最后 25 % 的时间窗取均值。包线与舵面限幅是 rev 11 的冻结值，与
% results/bench_stick_meta.txt 里记的一致。

root = fileparts(fileparts(mfilename('fullpath')));
traj_file = fullfile(root, 'results', 'bench_stick_traj.mat');
assert(exist(traj_file, 'file') == 2, 'bench_stick_report:NoData', '没有 %s', traj_file);
S = load(traj_file);
traj = S.traj;
n = numel(traj);

R2D = 180 / pi;
V_max_env = 30.0;
bank_max = 45.0;
pitch_max = 30.0;
alpha_stall = 14.0;
da_lim = 25.0;
de_lim = 25.0;
dr_lim = 25.0;

M = repmat(local_blank_metrics(), n, 1);
for k = 1:n
   tr = traj(k);
   tg = tr.t_fast(:)';
   tc = tr.t_act(:)';
   V = tr.inst(1, :);
   h = tr.inst(2, :);
   alpha = tr.inst(3, :);
   stall = tr.inst(6, :);
   phi = tr.att(2, :) * R2D;
   phi_c = tr.att(1, :) * R2D;
   theta = tr.att(4, :) * R2D;
   theta_c = tr.att(3, :) * R2D;
   p = tr.rate(2, :) * R2D;
   q = tr.rate(4, :) * R2D;
   r = tr.rate(6, :) * R2D;
   q_c = tr.rate(3, :) * R2D;
   da = tr.act(1, :) * R2D;
   de = tr.act(2, :) * R2D;
   dr = tr.act(3, :) * R2D;
   thl = tr.act(4, :);
   thr = tr.act(5, :);
   thr_mean = 0.5 * (thl + thr);

   m = local_blank_metrics();
   m.case_id = tr.case_id;
   m.ctrl = tr.ctrl;
   m.roll = tr.roll;
   m.pitch = tr.pitch;
   m.yaw = tr.yaw;
   m.thr_stick = tr.thr;
   m.label = tr.label;
   m.wall_s = tr.wall_s;
   m.t_end = tr.t_end;

   m.V_min = min(V); m.V_max = max(V); m.V_fin = V(end); m.V_ss = local_ss(tg, V);
   m.h_min = min(h); m.h_max = max(h); m.h_fin = h(end); m.h_ss = local_ss(tg, h);
   m.h_rate_end = (h(end) - h(find(tg >= tr.t_end - 2, 1))) / 2;
   m.alpha_min = min(alpha); m.alpha_max = max(alpha); m.alpha_ss = local_ss(tg, alpha);

   m.phi_c_fin = phi_c(end); m.phi_fin = phi(end);
   m.phi_max = max(phi); m.phi_min = min(phi); m.phi_ss = local_ss(tg, phi);
   m.theta_c_fin = theta_c(end); m.theta_fin = theta(end);
   m.theta_max = max(theta); m.theta_min = min(theta); m.theta_ss = local_ss(tg, theta);

   m.p_max = max(p); m.p_min = min(p); m.q_max = max(q); m.q_min = min(q);
   m.r_max = max(r); m.r_min = min(r);
   m.q_c_max = max(q_c); m.q_c_min = min(q_c);

   m.da_min = min(da); m.da_max = max(da); m.da_fin = da(end);
   m.de_min = min(de); m.de_max = max(de); m.de_fin = de(end);
   m.dr_min = min(dr); m.dr_max = max(dr); m.dr_fin = dr(end);
   m.thl_min = min(thl); m.thl_max = max(thl); m.thl_fin = thl(end);
   m.thr_min = min(thr); m.thr_max = max(thr); m.thr_fin = thr(end);
   m.thl_ss = local_ss(tc, thl);

   m.sat_da = mean(abs(da) >= 0.99 * da_lim);
   m.sat_de = mean(abs(de) >= 0.99 * de_lim);
   m.sat_dr = mean(abs(dr) >= 0.99 * dr_lim);
   m.sat_thl = mean(thl >= 0.99);

   m.chat_da = local_chat(tc, da);
   m.chat_de = local_chat(tc, de);
   m.chat_dr = local_chat(tc, dr);
   m.chat_thl = local_chat(tc, thl);
   m.f_dom_de = local_fdom(tc, de);
   m.f_dom_thl = local_fdom(tc, thl);
   m.f_dom_q = local_fdom(tg, q);

   m.rms_e_phi = sqrt(mean((phi_c - phi).^2));
   m.rms_e_theta = sqrt(mean((theta_c - theta).^2));
   m.rms_e_q = sqrt(mean((q_c - q).^2));
   m.rms_e_V = sqrt(mean((25.0 - V).^2));
   m.ss_e_phi = local_ss(tg, phi_c - phi);
   m.ss_e_theta = local_ss(tg, theta_c - theta);
   m.ss_e_V = local_ss(tg, 25.0 - V);

   m.frac_phi = mean(abs(phi) > bank_max);
   m.frac_theta = mean(abs(theta) > pitch_max);
   m.frac_Vhi = mean(V > V_max_env);
   m.frac_alpha = mean(abs(alpha) > alpha_stall);
   m.stall_frac = mean(stall > 0.5);

   M(k) = m;
end

% ---------------- CSV ----------------
fn = fieldnames(M);
csv = fullfile(root, 'results', 'bench_stick_metrics.csv');
fid = fopen(csv, 'w', 'n', 'UTF-8');
assert(fid > 0, 'bench_stick_report:NoCsv', '写不了 %s', csv);
fprintf(fid, '%s', fn{1});
for j = 2:numel(fn)
   fprintf(fid, ',%s', fn{j});
end
fprintf(fid, '\n');
for k = 1:n
   for j = 1:numel(fn)
      v = M(k).(fn{j});
      if j > 1
         fprintf(fid, ',');
      end
      if ischar(v)
         fprintf(fid, '%s', v);
      else
         fprintf(fid, '%.10g', v);
      end
   end
   fprintf(fid, '\n');
end
fclose(fid);

% ---------------- 分组表 ----------------
md = fullfile(root, 'docs', 'report_sources', 'stick_tables.md');
if ~exist(fullfile(root, 'docs', 'report_sources'), 'dir')
   mkdir(fullfile(root, 'docs', 'report_sources'));
end
mfid = fopen(md, 'w', 'n', 'UTF-8');
assert(mfid > 0, 'bench_stick_report:NoMd', '写不了 %s', md);
fprintf(mfid, '# 满杆矩阵数据表（自动生成）\n\n');
fprintf(mfid, '工况共 %d 段，每段仿真 %.0f s，控制器 0 是 PID，1 是 LQR。\n', n, traj(1).t_end);
fprintf(mfid, '末段均值指最后 25 %% 时间窗的均值。角度单位为度，速率单位为度每秒，\n');
fprintf(mfid, '空速单位为 m/s，高度单位为 m，油门为 0 到 1。\n\n');

fam = {'基准', 1:1; '单轴满杆', 2:7; '两轴组合', 8:19; '三轴组合', 20:27};
for f = 1:size(fam, 1)
   fprintf(mfid, '## %s\n\n', fam{f, 1});
   fprintf(mfid, '| id | 控制器 | 工况 | V 末值 | h 末值 | h 变化率 | 俯仰指令 | 俯仰末值 | 俯仰误差 |\n');
   fprintf(mfid, '|---|---|---|---|---|---|---|---|---|\n');
   for id = fam{f, 2}
      for c = 0:1
         k = local_find(M, id, c);
         if k == 0
            continue
         end
         m = M(k);
         fprintf(mfid, '| %d | %s | %s | %.2f | %.1f | %+.2f | %+.2f | %+.2f | %+.2f |\n', ...
            m.case_id, local_ctrl(m.ctrl), m.label, m.V_fin, m.h_fin, m.h_rate_end, ...
            m.theta_c_fin, m.theta_fin, m.ss_e_theta);
      end
   end
   fprintf(mfid, '\n');
end
fclose(mfid);

rep = struct();
rep.n = n;
rep.csv = csv;
rep.md = md;
fprintf('整理 %d 段\n', n);
fprintf('指标表 %s\n', csv);
fprintf('分组表 %s\n', md);
end

% ------------------------------------------------------------------------
function m = local_blank_metrics()
m = struct('case_id', 0, 'ctrl', 0, 'roll', 0, 'pitch', 0, 'yaw', 0, 'thr_stick', 0, ...
   'label', '', 'wall_s', 0, 't_end', 0, ...
   'V_min', 0, 'V_max', 0, 'V_fin', 0, 'V_ss', 0, ...
   'h_min', 0, 'h_max', 0, 'h_fin', 0, 'h_ss', 0, 'h_rate_end', 0, ...
   'alpha_min', 0, 'alpha_max', 0, 'alpha_ss', 0, ...
   'phi_c_fin', 0, 'phi_fin', 0, 'phi_max', 0, 'phi_min', 0, 'phi_ss', 0, ...
   'theta_c_fin', 0, 'theta_fin', 0, 'theta_max', 0, 'theta_min', 0, 'theta_ss', 0, ...
   'p_max', 0, 'p_min', 0, 'q_max', 0, 'q_min', 0, 'r_max', 0, 'r_min', 0, ...
   'q_c_max', 0, 'q_c_min', 0, ...
   'da_min', 0, 'da_max', 0, 'da_fin', 0, ...
   'de_min', 0, 'de_max', 0, 'de_fin', 0, ...
   'dr_min', 0, 'dr_max', 0, 'dr_fin', 0, ...
   'thl_min', 0, 'thl_max', 0, 'thl_fin', 0, 'thl_ss', 0, ...
   'thr_min', 0, 'thr_max', 0, 'thr_fin', 0, ...
   'sat_da', 0, 'sat_de', 0, 'sat_dr', 0, 'sat_thl', 0, ...
   'chat_da', 0, 'chat_de', 0, 'chat_dr', 0, 'chat_thl', 0, ...
   'f_dom_de', 0, 'f_dom_thl', 0, 'f_dom_q', 0, ...
   'rms_e_phi', 0, 'rms_e_theta', 0, 'rms_e_q', 0, 'rms_e_V', 0, ...
   'ss_e_phi', 0, 'ss_e_theta', 0, 'ss_e_V', 0, ...
   'frac_phi', 0, 'frac_theta', 0, 'frac_Vhi', 0, 'frac_alpha', 0, 'stall_frac', 0);
end

function y = local_ss(t, x)
k = find(t >= t(end) * 0.75, 1);
if isempty(k)
   k = 1;
end
y = mean(x(k:end));
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
% 末段 4 s 内除直流外幅值最大的频率，用来量化颤振的基频。
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
f = (i) / (n * dt);
end

function k = local_find(M, id, ctrl)
k = 0;
for i = 1:numel(M)
   if M(i).case_id == id && M(i).ctrl == ctrl
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
