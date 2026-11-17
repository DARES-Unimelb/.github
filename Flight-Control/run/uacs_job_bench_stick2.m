% UACS_JOB_BENCH_STICK2 The same full-stick matrix as uacs_job_bench_stick, plus a progress
% line written and flushed to disk after every segment.
%
% Usage
%   addpath('<repo>/run');
%   assignin('base', 'stick_budget_s', 5400);
%   run_job('bench_stick2')
%
% Both jobs share results/bench_stick_traj.mat, so progress carries over between them.
% Progress goes to results/bench_stick_progress.txt, one line per finished segment.

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_stick2');
end
root = fileparts(fileparts(this));
addpath(root, fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');

mdl = 'uacs_flight_bench';
H = 12;
fast_dt = 0.005;
act_dt = 0.01;
budget_s = 400;
if evalin('base', 'exist(''stick_budget_s'', ''var'')')
   budget_s = double(evalin('base', 'stick_budget_s'));
end

pos_word = {'右滚满', '拉杆满', '右舵满'};
neg_word = {'左滚满', '推杆满', '左舵满'};
case_stick = zeros(0, 3);
case_label = {};
case_stick(end + 1, :) = [0 0 0];
case_label{end + 1} = '基准 全杆回中';
for a = 1:3
   for s = [1 -1]
      v = [0 0 0];
      v(a) = s;
      case_stick(end + 1, :) = v;
      if s > 0
         case_label{end + 1} = pos_word{a};
      else
         case_label{end + 1} = neg_word{a};
      end
   end
end
prs = [1 2; 1 3; 2 3];
for i = 1:3
   for s1 = [1 -1]
      for s2 = [1 -1]
         v = [0 0 0];
         v(prs(i, 1)) = s1;
         v(prs(i, 2)) = s2;
         case_stick(end + 1, :) = v;
         w = cell(1, 3);
         for a = 1:3
            if v(a) > 0
               w{a} = pos_word{a};
            elseif v(a) < 0
               w{a} = neg_word{a};
            else
               w{a} = '';
            end
         end
         case_label{end + 1} = strjoin(w(~cellfun(@isempty, w)), '+');
      end
   end
end
for s1 = [1 -1]
   for s2 = [1 -1]
      for s3 = [1 -1]
         v = [s1 s2 s3];
         case_stick(end + 1, :) = v;
         w = cell(1, 3);
         for a = 1:3
            if v(a) > 0
               w{a} = pos_word{a};
            else
               w{a} = neg_word{a};
            end
         end
         case_label{end + 1} = strjoin(w, '+');
      end
   end
end
n_case = size(case_stick, 1);
assert(n_case == 27, 'uacs_job_bench_stick2:BadTable', '工况表应是 27 段，实际 %d 段', n_case);

evalin('base', 'uacs_go');
was_loaded = bdIsLoaded(mdl);
load_system(fullfile(root, 'model', [mdl '.slx']));
stop_fcn = get_param(mdl, 'StopFcn');
set_param(mdl, 'StopFcn', '');
P = evalin('base', 'P');
thr_stick = evalin('base', 'sim_thr_trim');

traj_file = fullfile(root, 'results', 'bench_stick_traj.mat');
tmp_file = fullfile(root, 'results', 'bench_stick_traj_tmp.mat');
if exist(traj_file, 'file')
   S = load(traj_file);
   traj = S.traj;
else
   traj = struct([]);
end
done = false(n_case, 2);
for k = 1:numel(traj)
   done(traj(k).case_id, traj(k).ctrl + 1) = true;
end
n_done = sum(done(:));
fprintf('进度 %d/%d 段，本次预算 %.0f s\n', n_done, n_case * 2, budget_s);
pfid = fopen(fullfile(root, 'results', 'bench_stick_progress.txt'), 'a');

t_budget = tic;
n_this = 0;
stop_now = false;
for ci = 1:n_case
   for ct = 0:1
      if stop_now
         break
      end
      if done(ci, ct + 1)
         continue
      end
      if toc(t_budget) > budget_s
         stop_now = true;
         break
      end
      set_param([mdl '/控制器选择/k_sel'], 'Value', num2str(ct));
      set_param([mdl '/驾驶员/k_roll'], 'Value', num2str(case_stick(ci, 1)));
      set_param([mdl '/驾驶员/k_pitch'], 'Value', num2str(case_stick(ci, 2)));
      set_param([mdl '/驾驶员/k_yaw'], 'Value', num2str(case_stick(ci, 3)));
      set_param([mdl '/驾驶员/k_thr'], 'Value', num2str(thr_stick, 17));

      t_one = tic;
      so = sim(mdl, 'StopTime', num2str(H), 'ReturnWorkspaceOutputs', 'on');
      wall = toc(t_one);

      ls = so.logsout;
      gi = ls.get('inst_log').Values;
      at = ls.get('att_log').Values;
      ra = ls.get('rate_log').Values;
      ac = ls.get('act_log').Values;
      tg = (0:fast_dt:H)';
      tc = (0:act_dt:H)';
      tr = struct();
      tr.case_id = ci;
      tr.ctrl = ct;
      tr.roll = case_stick(ci, 1);
      tr.pitch = case_stick(ci, 2);
      tr.yaw = case_stick(ci, 3);
      tr.thr = thr_stick;
      tr.label = case_label{ci};
      tr.t_end = H;
      tr.t_fast = tg;
      tr.inst = interp1(gi.Time, gi.Data, tg)';
      tr.att = interp1(at.Time, at.Data, tg)';
      tr.rate = interp1(ra.Time, ra.Data, tg)';
      tr.t_act = tc;
      tr.act = interp1(ac.Time, ac.Data, tc)';
      tr.wall_s = wall;
      if isempty(traj)
         traj = tr;
      else
         traj(end + 1) = tr;
      end
      save(tmp_file, 'traj');
      movefile(tmp_file, traj_file, 'f');
      n_this = n_this + 1;
      fprintf('[%2d/%2d] %-22s ctrl %d  wall %6.1f s  V %6.2f  theta %+7.2f  de %+7.2f  thr %5.3f\n', ...
         ci, n_case, case_label{ci}, ct, wall, tr.inst(1, end), ...
         tr.att(4, end) * 180 / pi, tr.act(2, end) * 180 / pi, tr.act(4, end));
      if pfid > 0
         fprintf(pfid, '%s  id %2d ctrl %d %-22s wall %6.1f s  V_fin %7.3f  theta_fin %+7.3f  de_fin %+7.3f  total %d\n', ...
            datestr(now, 'yyyy-mm-dd HH:MM:SS'), ci, ct, case_label{ci}, wall, ...
            tr.inst(1, end), tr.att(4, end) * 180 / pi, tr.act(2, end) * 180 / pi, n_done + n_this);
         fclose(pfid);
         pfid = fopen(fullfile(root, 'results', 'bench_stick_progress.txt'), 'a');
      end
   end
end
if pfid > 0
   fclose(pfid);
end

done(:) = false;
for k = 1:numel(traj)
   done(traj(k).case_id, traj(k).ctrl + 1) = true;
end
mfid = fopen(fullfile(root, 'results', 'bench_stick_meta.txt'), 'w', 'n', 'UTF-8');
assert(mfid > 0, 'uacs_job_bench_stick2:NoMeta', '写不了 results/bench_stick_meta.txt');
fprintf(mfid, '台架满杆矩阵  %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(mfid, '每段仿真时长 %.1f s，快信号步长 %.4f s，作动器步长 %.4f s\n', H, fast_dt, act_dt);
fprintf(mfid, '油门杆位 %.17g，对应空速指令 %.4f m/s\n', thr_stick, ...
   P.limits.V_min_mps + thr_stick * (P.limits.V_max_mps - P.limits.V_min_mps));
fprintf(mfid, '参数集 rev %g / %s，包线 V %.4f 到 %.4f m/s，滚转 ±%.2f 度，俯仰 ±%.2f 度\n', ...
   P.meta.revision, P.meta.version, P.limits.V_min_mps, P.limits.V_max_mps, ...
   P.limits.bank_max_deg, P.limits.pitch_max_deg);
fprintf(mfid, '舵面限幅 副翼 ±%.4f 度，升降舵 ±%.4f 度，方向舵 ±%.4f 度，失速迎角 %.4f 度\n', ...
   P.act.aileron.max_deg, P.act.elevator.max_deg, P.act.rudder.max_deg, P.aero.alpha_stall_deg);
fprintf(mfid, '控制器 0 是 PID，1 是 LQR\n');
fprintf(mfid, '信号通道：inst_log 七路 V h alpha phi theta stall r；att_log 四路 phi_c phi theta_c theta；\n');
fprintf(mfid, '           rate_log 六路 p_c p q_c q r_c r；act_log 五路 da de dr dth_l dth_r\n');
fprintf(mfid, '\n%4s %4s %6s %6s %6s %4s  %s\n', 'id', 'ctrl', 'roll', 'pitch', 'yaw', 'done', 'label');
for ci = 1:n_case
   for ct = 0:1
      if done(ci, ct + 1)
         mark = 'x';
      else
         mark = ' ';
      end
      fprintf(mfid, '%4d %4d %6g %6g %6g %4s  %s\n', ci, ct, case_stick(ci, 1), ...
         case_stick(ci, 2), case_stick(ci, 3), mark, case_label{ci});
   end
end
fclose(mfid);

set_param([mdl '/控制器选择/k_sel'], 'Value', 'sim_ctrl_sel');
set_param([mdl '/驾驶员/k_roll'], 'Value', 'sim_stick_roll');
set_param([mdl '/驾驶员/k_pitch'], 'Value', 'sim_stick_pitch');
set_param([mdl '/驾驶员/k_yaw'], 'Value', 'sim_stick_yaw');
set_param([mdl '/驾驶员/k_thr'], 'Value', 'sim_stick_thr');
set_param(mdl, 'StopFcn', stop_fcn);
if ~was_loaded
   close_system(mdl, 0);
end
fprintf('本次完成 %d 段，累计 %d/%d 段\n', n_this, n_done + n_this, n_case * 2);
if n_done + n_this >= n_case * 2
   fprintf('矩阵已跑满\n');
else
   fprintf('还没跑满，再调一次本作业接着跑\n');
end
fprintf('JOB BODY DONE\n');
