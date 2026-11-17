function T = sim_read_yout(out, names)
% SIM_READ_YOUT Turn a simulation output object into a time vector and signals
%
%   out     Simulink.SimulationOutput carrying a yout entry
%   names   optional cell array of outport names to keep, all of them when omitted
%   T.t       nTx1 time vector
%   T.y       nTxN, the kept outport signals concatenated
%   T.names   one name per outport
%   T.widths  width of each outport, so a caller can split T.y
%   T.by_name struct with one field per outport
%
% Handles either save format, so a scenario does not depend on the config set

y = out.get('yout');
T = struct('t', [], 'y', [], 'names', {{}}, 'widths', [], 'by_name', struct());
if isa(y, 'Simulink.SimulationData.Dataset')
   n = y.numElements;
   for k = 1:n
      el = y.getElement(k);
      if isempty(T.t)
         T.t = el.Values.Time;
      end
      T.names{end + 1} = el.Name; %#ok<AGROW>
      T.by_name.(matlab.lang.makeValidName(el.Name)) = el.Values.Data;
   end
elseif isstruct(y) && isfield(y, 'signals')
   T.t = y.time;
   for k = 1:numel(y.signals)
      nm = y.signals(k).label;
      if isempty(nm)
         nm = sprintf('out%d', k);
      end
      T.names{end + 1} = nm; %#ok<AGROW>
      vals = y.signals(k).values;
      if ndims(vals) > 2 %#ok<ISMAT>
         vals = squeeze(vals);
      end
      T.by_name.(matlab.lang.makeValidName(nm)) = vals;
   end
else
   error('sim_read_yout:Format', 'unsupported yout class %s', class(y));
end

widths = zeros(1, numel(T.names));
for k = 1:numel(T.names)
   v = T.by_name.(matlab.lang.makeValidName(T.names{k}));
   widths(k) = size(v, 2);
end
T.widths = widths;
Y = zeros(numel(T.t), sum(widths));
c = 0;
for k = 1:numel(T.names)
   v = T.by_name.(matlab.lang.makeValidName(T.names{k}));
   Y(:, c + (1:widths(k))) = v;
   c = c + widths(k);
end
T.y = Y;

if nargin > 1 && ~isempty(names)
   keep = ismember(T.names, names);
   T.names = T.names(keep);
   T.widths = T.widths(keep);
   idx = [];
   c = 0;
   for k = 1:numel(widths)
      if keep(k)
         idx = [idx, c + (1:widths(k))]; %#ok<AGROW>
      end
      c = c + widths(k);
   end
   T.y = T.y(:, idx);
end
end
