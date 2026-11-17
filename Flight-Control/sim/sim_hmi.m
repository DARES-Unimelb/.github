function out = sim_hmi(action, varargin)
% SIM_HMI Create a canvas Dashboard control or readout and bind it programmatically
%
%   sim_hmi('knob',    mdl, name, pos, owner, [lo hi])
%   sim_hmi('slider',  mdl, name, pos, owner, [lo hi])
%   sim_hmi('toggle',  mdl, name, pos, owner, labels)
%   sim_hmi('rotary',  mdl, name, pos, owner, labels)
%   sim_hmi('gauge',   mdl, name, pos, [lo hi], sig)
%   sim_hmi('lamp',    mdl, name, pos, sig)
%   sim_hmi('display', mdl, name, pos, sig)
%   sim_hmi('scope',   mdl, name, pos, sig)          sig must be a cell array
%   sim_hmi('note',    mdl, name, pos, text)
%   sim_hmi('note',    mdl, name, pos, text, 'header')
%
%   out is the new block path, or the Annotation object for 'note'
%   owner is the block a control drives; the control binds to its Value parameter,
%   so that parameter must hold a literal
%   sig is {block path, output port}, or an N-by-2 cell array for several signals
%
% Scope Binding must be a cell array; Gauge, Lamp and Display take a single object
% Dashboard blocks have no ports: they are bound, never wired

switch lower(action)
   case 'knob'
      out = local_scaled('built-in/KnobBlock', varargin{:});
   case 'slider'
      out = local_scaled('built-in/SliderBlock', varargin{:});
   case 'toggle'
      out = local_switch('built-in/ToggleSwitchBlock', varargin{:});
   case 'rotary'
      out = local_switch('built-in/RotarySwitchBlock', varargin{:});
   case 'gauge'
      out = local_readout('built-in/CircularGaugeBlock', true, varargin{:});
   case 'lamp'
      out = local_readout('built-in/LampBlock', false, varargin{:});
   case 'display'
      out = local_readout('built-in/DisplayBlock', false, varargin{:});
   case 'scope'
      out = local_scope(varargin{:});
   case 'note'
      out = local_note(varargin{:});
   otherwise
      error('sim_hmi:BadAction', '未知的 action：%s', action);
end
end

%% Local helpers
function blk = local_scaled(typ, mdl, name, pos, owner, lim)
% numeric control, knob or slider; the range goes into the block, the binding to owner
blk = [mdl '/' name];
add_block(typ, blk, 'Position', pos);
assert(numel(lim) == 2 && lim(1) < lim(2), 'sim_hmi:BadLimits', ...
   '%s 的量程要写成 [下界 上界] 且下界小于上界', name);
set_param(blk, 'Limits', lim);
local_bind_param(blk, owner);
end

function blk = local_switch(typ, mdl, name, pos, owner, labels)
% switch control; States is a struct array with one Value and Label per position
blk = [mdl '/' name];
add_block(typ, blk, 'Position', pos);
labels = labels(:)';
n = numel(labels);
states = struct('Value', num2cell(0:(n - 1)), 'Label', labels);
set_param(blk, 'States', states);
local_bind_param(blk, owner);
end

function blk = local_readout(typ, has_limits, mdl, name, pos, varargin)
% readout, bound to a signal; has_limits sits before mdl to match the dispatch above
blk = [mdl '/' name];
add_block(typ, blk, 'Position', pos);
if has_limits
   lim = varargin{1};
   varargin(1) = [];
   set_param(blk, 'Limits', lim);
end
specs = local_specs(varargin{:});
assert(numel(specs) == 1, 'sim_hmi:BadSignal', '%s 只接一路信号', name);
set_param(blk, 'Binding', specs{1});
end

function blk = local_scope(mdl, name, pos, varargin)
% dashboard scope; Binding must be a cell array, a single object is rejected
blk = [mdl '/' name];
add_block('built-in/DashboardScope', blk, 'Position', pos);
specs = local_specs(varargin{:});
set_param(blk, 'Binding', specs);
end

function a = local_note(mdl, name, pos, txt, kind)
% canvas note; 'header' is large and bold, anything else is body text
if nargin < 5
   kind = 'text';
end
a = Simulink.Annotation([mdl '/' name]);
a.Text = txt;
a.Position = pos;
if strcmp(kind, 'header')
   a.FontSize = 14;
   a.FontWeight = 'bold';
end
end

%% Binding helpers
function local_bind_param(blk, owner)
% bind a control to a block parameter whose value is a literal
v = get_param(owner, 'Value');
assert(~isnan(str2double(v)), 'sim_hmi:NotLiteral', ...
   '%s 的 Value 是表达式 %s，控件绑不上；操纵量要先落成字面量再绑定', owner, v);
b = Simulink.HMI.ParamSourceInfo;
b.BlockPath = Simulink.BlockPath({owner});
b.ParamName = 'Value';
set_param(blk, 'Binding', b);
rb = get_param(blk, 'Binding');
assert(rb.isBound(), 'sim_hmi:BindFailed', '%s 绑不到 %s 的 Value', blk, owner);
end

function specs = local_specs(varargin)
% normalise {block path, port} or an N-by-2 cell array into SignalSpecification objects
if numel(varargin) == 1 && iscell(varargin{1})
   pairs = varargin{1};
elseif numel(varargin) == 2
   pairs = {varargin{1}, varargin{2}};
else
   error('sim_hmi:BadSignal', '信号要写成 {块路径, 端口} 或 N×2 的元胞表');
end
assert(size(pairs, 2) == 2, 'sim_hmi:BadSignal', '信号表必须是 N×2');
specs = cell(1, size(pairs, 1));
for k = 1:size(pairs, 1)
   s = Simulink.HMI.SignalSpecification;
   s.BlockPath = Simulink.BlockPath({pairs{k, 1}});
   s.OutputPortIndex = pairs{k, 2};
   specs{k} = s;
end
end
