function sim_name_line(mdl, blk, name)
% SIM_NAME_LINE Name a block's output signal line
%
%   mdl    model name
%   blk    block path, optionally with a trailing '/port' as sim_connect takes
%   name   new line name
%
% Names the line rather than the block, so signal labels and Scope legends follow it

% a trailing pure-number segment is the port index and is stripped
parts = strsplit(blk, '/');
if numel(parts) > 1 && ~isnan(str2double(parts{end}))
   parts(end) = [];
   blk = strjoin(parts, '/');
end

lh = -1;
ph = get_param([mdl '/' blk], 'PortHandles');
for k = 1:numel(ph.Outport)
   if ph.Outport(k) > 0
      l = get_param(ph.Outport(k), 'Line');
      if l > 0
         lh = l;
         break;
      end
   end
end
assert(lh > 0, 'sim_name_line:NoLine', '%s has no output line to name', blk);
set_param(lh, 'Name', name);
end
