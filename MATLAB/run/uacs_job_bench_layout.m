% UACS_JOB_BENCH_LAYOUT Job entry: export the bench canvas layout and scope diagnostics.
%
% Run with:  addpath('C:\Ash\Projects\DARES_UACS\MATLAB\run'); run_job('bench_layout')
% Output: results/job_bench_layout.log        状态与失败原因
%         results/bench_layout.txt            诊断件
%         sim/uacs_bench_layout_list.m        画布布局数据，自动生成，重建时套回
%
% 为什么要有这个作业：台架模型由 sim_build_bench 整份重建，重建会先删掉旧文件。用户在画布
% 上手工摆过的位置只存在于盘上那个 .slx 里，必须先抽成数据，重建时再套回去。
%
% 本作业只读模型，不保存。脚本里不写局部函数：带局部函数的作业脚本会被 MATLAB 按函数缓存，
% 改了盘上的文件之后 evalin 仍可能跑到旧文本（实测踩过一次）。
%
% 下面这段参数转储是内联的，就是为了不引入局部函数。

this = mfilename('fullpath');
if isempty(this)
   this = which('uacs_job_bench_layout');
end
root = fileparts(fileparts(this));
addpath(root, fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions;
rehash path;
fprintf('job root %s\n', root);

mdl = 'uacs_flight_bench';
was_open = bdIsLoaded(mdl);
load_system(fullfile(root, 'model', [mdl '.slx']));
out_txt = fullfile(root, 'results', 'bench_layout.txt');
fid = fopen(out_txt, 'w', 'n', 'UTF-8');
assert(fid > 0, 'uacs_job_bench_layout:NoFile', '写不了 %s', out_txt);

fprintf(fid, '[模型]\n');
fprintf(fid, '  顶层块 %d 个，总块 %d 个\n', ...
   numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'Block')), ...
   numel(find_system(mdl, 'Type', 'Block')));
cb_list = {'PreLoadFcn', 'PostLoadFcn', 'InitFcn', 'StartFcn', 'StopFcn'};
for k = 1:numel(cb_list)
   v = get_param(mdl, cb_list{k});
   if isempty(v)
      v = '<空>';
   end
   fprintf(fid, '  %-12s %s\n', cb_list{k}, v);
end

fprintf(fid, '\n[画布内示波器]\n');
ds = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'DashboardScope');
for k = 1:numel(ds)
   fprintf(fid, '  -- %s  Position %s\n', ds{k}, mat2str(get_param(ds{k}, 'Position')));
   b = get_param(ds{k}, 'Binding');
   fprintf(fid, '     Binding 类 %s，元素 %d 个\n', class(b), numel(b));
   if iscell(b)
      for j = 1:numel(b)
         fprintf(fid, '       第 %d 路 -> %s 端口 %d\n', j, ...
            b{j}.BlockPath.getBlock(1), b{j}.OutputPortIndex);
      end
   end
   dp = get_param(ds{k}, 'DialogParameters');
   fld = fieldnames(dp);
   for j = 1:numel(fld)
      try
         v = get_param(ds{k}, fld{j});
         if ischar(v)
            fprintf(fid, '    %-26s = %s\n', fld{j}, v);
         elseif isnumeric(v) && isscalar(v)
            fprintf(fid, '    %-26s = %g\n', fld{j}, v);
         else
            fprintf(fid, '    %-26s = <%s %s>\n', fld{j}, class(v), mat2str(size(v)));
         end
      catch
         fprintf(fid, '    %-26s = <读不出>\n', fld{j});
      end
   end
end

fprintf(fid, '\n[兜底示波器]\n');
fb = [mdl '/兜底示波器 三段'];
if getSimulinkBlockHandle(fb) > 0
   fprintf(fid, '  Position %s，NumInputPorts %s，LayoutDimensionsString %s，OpenAtSimulationStart %s\n', ...
      mat2str(get_param(fb, 'Position')), get_param(fb, 'NumInputPorts'), ...
      get_param(fb, 'LayoutDimensionsString'), get_param(fb, 'OpenAtSimulationStart'));
   ph = get_param(fb, 'PortHandles');
   n = 0;
   for k = 1:numel(ph.Inport)
      if get_param(ph.Inport(k), 'Line') > 0
         n = n + 1;
      end
   end
   fprintf(fid, '  接上信号的输入口 %d 个\n', n);
else
   fprintf(fid, '  盘上没有这条块\n');
end

fprintf(fid, '\n[操纵台与仪表清点]\n');
bt_list = {'KnobBlock', 'SliderBlock', 'ToggleSwitchBlock', 'RotarySwitchBlock', ...
   'CircularGaugeBlock', 'LampBlock', 'Display'};
for k = 1:numel(bt_list)
   bs = find_system(mdl, 'SearchDepth', 1, 'BlockType', bt_list{k});
   for j = 1:numel(bs)
      fprintf(fid, '  %-20s %-26s %s\n', bt_list{k}, ...
         strrep(bs{j}, [mdl '/'], ''), mat2str(get_param(bs{j}, 'Position')));
   end
end

fprintf(fid, '\n[顶层其它块]\n');
top = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
for k = 1:numel(top)
   nm = strrep(top{k}, [mdl '/'], '');
   fprintf(fid, '  %-26s %-22s %s\n', nm, ...
      get_param(top{k}, 'BlockType'), mat2str(get_param(top{k}, 'Position')));
end

% ---------------- 布局导出 ----------------
blocks = find_system(mdl, 'Type', 'Block');
bp = cell(numel(blocks), 2);
for k = 1:numel(blocks)
   bp{k, 1} = strrep(blocks{k}, [mdl '/'], '');
   bp{k, 2} = get_param(blocks{k}, 'Position');
end
% FindAll 为 on 时 find_system 返回句柄数组，不是路径元胞，所以按数组下标取。
notes = find_system(mdl, 'FindAll', 'on', 'Type', 'annotation');
np = cell(numel(notes), 2);
for k = 1:numel(notes)
   np{k, 1} = get_param(notes(k), 'Text');
   np{k, 2} = get_param(notes(k), 'Position');
end

out = fullfile(root, 'sim', 'uacs_bench_layout_list.m');
f = fopen(out, 'w', 'n', 'UTF-8');
assert(f > 0, 'uacs_job_bench_layout:NoFile', '写不了 %s', out);
fprintf(f, 'function L = uacs_bench_layout_list()\n');
fprintf(f, '%% UACS_BENCH_LAYOUT_LIST 台架画布布局，从盘上模型读出，供重建时原样套回。\n');
fprintf(f, '%%\n');
fprintf(f, '%% 本文件由 run_job(''bench_layout'') 自动生成，不要手改，改了下次导出会被覆盖。\n');
fprintf(f, '%% 台架模型是整份重建的，重建会先删掉旧文件；用户在画布上摆过的位置只有这份数据记得住。\n');
fprintf(f, '%% 要更新：在界面上摆好并保存 .slx，然后重跑 run_job(''bench_layout'')。\n');
fprintf(f, '%%\n');
fprintf(f, '%% L.blocks  N x 2，第一列是去掉模型名的块路径，第二列是 [左 上 右 下]。\n');
fprintf(f, '%% L.notes   N x 2，第一列是注释正文的首行标题，第二列是 [左 上 右 下]。\n');
fprintf(f, '%%            注释没有独立于正文的身份，get_param 的 Name 就是从正文派生的，\n');
fprintf(f, '%%            所以只能拿首行当键：标题不改，位置就认得回来。\n');
fprintf(f, '\nL = struct();\n');
fprintf(f, 'L.blocks = {\n');
for k = 1:size(bp, 1)
   fprintf(f, '   ''%s'', %s\n', strrep(bp{k, 1}, '''', ''''''), mat2str(bp{k, 2}));
end
fprintf(f, '   };\n');
fprintf(f, 'L.notes = {\n');
for k = 1:size(np, 1)
   t = np{k, 1};
   nl = find(t == sprintf('\n'), 1);
   if ~isempty(nl)
      t = t(1:nl - 1);
   end
   t = strtrim(t);
   fprintf(f, '   ''%s'', %s\n', strrep(t, '''', ''''''), mat2str(np{k, 2}));
end
fprintf(f, '   };\n');
fprintf(f, 'end\n');
fclose(f);
fprintf(fid, '\n[布局导出]\n  块 %d 个，注释 %d 条，写入 %s\n', size(bp, 1), size(np, 1), out);
fclose(fid);
fprintf('诊断件 %s\n布局数据 %s（块 %d，注释 %d）\n', out_txt, out, size(bp, 1), size(np, 1));
% 只关自己加载的那一次。用户可能在界面上开着同一个模型，关掉会让他看不见窗口；有未保存
% 改动时更不能关，那会把他的摆位丢掉。
if was_open
   fprintf('模型本来就在界面上开着，保持打开\n');
elseif strcmp(get_param(mdl, 'Dirty'), 'off')
   close_system(mdl, 0);
else
   fprintf('模型有未保存改动，保持打开，不关\n');
end
fprintf('JOB BODY DONE\n');
