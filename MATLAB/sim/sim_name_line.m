function sim_name_line(mdl, blk, name)
% SIM_NAME_LINE Name a block's output line, so a label or a Scope legend reads as
% something a person can use.
%
%   sim_name_line('uacs_flight_bench', 'x_split/7', 'phi 实际滚转 [rad]')
%
% The LINE is named, not the block. Simulink's exported signal label follows the signal
% line, and a Scope block's legend follows the names of the lines feeding its Mux, so
% naming only the blocks leaves both of them reading out1, out2, and a legend of blanks.
% Renaming the line also labels that net everywhere it runs, which is what a reader wants.

% The callers write their taps as 'blk/port' because that is the form sim_connect takes,
% so a trailing pure-number segment is stripped here rather than keeping two parallel
% lists of block names and port numbers.
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
