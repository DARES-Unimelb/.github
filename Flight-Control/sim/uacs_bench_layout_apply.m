function rep = uacs_bench_layout_apply(mdl)
% UACS_BENCH_LAYOUT_APPLY 把导出的画布布局套回刚重建的模型。
%
%   rep = uacs_bench_layout_apply(mdl)
%
% 台架模型由 sim_build_bench 整份重建，重建会先删掉旧文件，所以用户在画布上摆过的位置
% 必须在建完之后再套回去。数据来自 sim/uacs_bench_layout_list.m，由 run_job('bench_layout')
% 从盘上那份 .slx 导出。
%
% 只套还存在的块；新建的块在数据里没有记录，留在生成脚本给的位置上。返回值里报出套了几
% 个、丢了几个，构建日志里能看见。
%
% 注释按正文匹配。注释没有独立于正文的身份，get_param 取到的 Name 就是从正文派生的，所以
% 只能拿正文当键。先按整段正文精确匹配，不中再按首行标题匹配：这样正文改了而标题没改的
% 注释也认得回来。导出件里的换行被写成两字符的 \n，比较前要先还原。

L = uacs_bench_layout_list();
rep = struct('applied', 0, 'missing', 0, 'notes_applied', 0, 'notes_missing', 0);

for k = 1:size(L.blocks, 1)
   blk = [mdl '/' L.blocks{k, 1}];
   if getSimulinkBlockHandle(blk) > 0
      set_param(blk, 'Position', L.blocks{k, 2});
      rep.applied = rep.applied + 1;
   else
      rep.missing = rep.missing + 1;
   end
end

notes = find_system(mdl, 'FindAll', 'on', 'Type', 'annotation');
for k = 1:size(L.notes, 1)
   want = strrep(L.notes{k, 1}, '\n', sprintf('\n'));
   want_first = local_first(want);
   hit = 0;
   for j = 1:numel(notes)
      if strcmp(get_param(notes(j), 'Text'), want)
         hit = notes(j);
         break
      end
   end
   if hit == 0
      for j = 1:numel(notes)
         if strcmp(local_first(get_param(notes(j), 'Text')), want_first)
            hit = notes(j);
            break
         end
      end
   end
   if hit > 0
      set_param(hit, 'Position', L.notes{k, 2});
      rep.notes_applied = rep.notes_applied + 1;
   else
      rep.notes_missing = rep.notes_missing + 1;
   end
end
end

% ------------------------------------------------------------------------
function s = local_first(t)
% 注释的首行标题。没有换行就是整段。
nl = find(t == sprintf('\n'), 1);
if ~isempty(nl)
   t = t(1:nl - 1);
end
s = strtrim(t);
end
