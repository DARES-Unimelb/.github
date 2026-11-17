function out = sim_hmi(action, varargin)
% SIM_HMI 建画布上的 Dashboard 控件与仪表，并程序化绑定。
%
% 用法
%   sim_hmi('knob',    mdl, name, pos, owner, [lo hi])
%   sim_hmi('slider',  mdl, name, pos, owner, [lo hi])
%   sim_hmi('toggle',  mdl, name, pos, owner, labels)
%   sim_hmi('rotary',  mdl, name, pos, owner, labels)
%   sim_hmi('gauge',   mdl, name, pos, [lo hi], sig)
%   sim_hmi('lamp',    mdl, name, pos, sig)
%   sim_hmi('display', mdl, name, pos, sig)
%   sim_hmi('scope',   mdl, name, pos, sig)          sig 必须是元胞表
%   sim_hmi('note',    mdl, name, pos, text)
%   sim_hmi('note',    mdl, name, pos, text, 'header')  分带标题，大字号加粗
%
%   owner 是控件要驱动的那个块，控件绑的是它的 Value 参数，所以那个参数必须是字面量。
%   sig 写成 {块路径, 输出端口号}，多路就写成 N×2 元胞表。
%
% 两类绑定，都是官方 set_param 接口
%   控件绑块参数    Simulink.HMI.ParamSourceInfo        带 BlockPath / ParamName
%   仪表绑信号      Simulink.HMI.SignalSpecification
%
% 五条实测出来的硬约束，都写成了断言或固定写法
%   1. 控件绑基础工作区变量是**假的通路**。那样绑能建起来、isBound 也返回真、控件自己
%      的 setValue 也改得动那个变量，但仿真运行中改那个变量，仿真完全不理：实测末态
%      俯仰角 0.013 deg，与不动一样。绑字面值参数则在运行中立刻生效：9.272 deg，干预
%      本身只要 0.04 s。所以控件一律绑参数，参数一律先落成字面量。
%   2. Scope 的 Binding 必须是元胞数组，Gauge / Lamp / Display 必须是单个对象。
%      给错了两边的报错话术不一样，很容易看反。
%   3. 开关类块的 States 必须是 N 元结构体数组，字段 Value 与 Label，Value 是标量实数。
%   4. Dashboard 块没有任何端口。它不接信号线，只接绑定，所以画布上不会因为它多出
%      一根线，也不能用 add_line 连它。
%   5. 控件驱动不了"值是表达式"的参数：Constant 的 Value 写成 sim_stick_pitch 时绑不上，
%      写 0.656 才绑得上。落字面量这一步在 sim_build_bench 里做，见 uacs_bench_seed。

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

% ------------------------------------------------------------------------
function blk = local_scaled(typ, mdl, name, pos, owner, lim)
% 数值控件：旋钮与滑杆。量程写进块参数，绑定走 owner 的 Value 参数。
blk = [mdl '/' name];
add_block(typ, blk, 'Position', pos);
assert(numel(lim) == 2 && lim(1) < lim(2), 'sim_hmi:BadLimits', ...
   '%s 的量程要写成 [下界 上界] 且下界小于上界', name);
set_param(blk, 'Limits', lim);
local_bind_param(blk, owner);
end

function blk = local_switch(typ, mdl, name, pos, owner, labels)
% 开关类控件：拨动开关与旋转开关。States 是 N 元结构体数组。
blk = [mdl '/' name];
add_block(typ, blk, 'Position', pos);
labels = labels(:)';
n = numel(labels);
states = struct('Value', num2cell(0:(n - 1)), 'Label', labels);
set_param(blk, 'States', states);
local_bind_param(blk, owner);
end

function blk = local_readout(typ, has_limits, mdl, name, pos, varargin)
% 读数块：圆形仪表、指示灯、数字显示。绑定走信号。has_limits 排在 mdl 前面，因为
% 分派那一层是按"块类型 + 是否有量程 + 其余参数"拼的。
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
% 示波器。Binding 必须是元胞数组，给单个对象会被拒。
blk = [mdl '/' name];
add_block('built-in/DashboardScope', blk, 'Position', pos);
specs = local_specs(varargin{:});
set_param(blk, 'Binding', specs);
end

function a = local_note(mdl, name, pos, txt, kind)
% 画布上的说明。Simulink.Annotation 是文档对象；用 'built-in/Note' 会往块普查里多塞
% 一个块。R2024a 的 AnnotationType 是只读参数，赋值直接报 read-only，所以画不出区域
% 框，分带只用两种字号区分：'header' 是标题，'text' 是正文。
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

% ------------------------------------------------------------------------
function local_bind_param(blk, owner)
% 把控件绑到一个"值是字面量"的块参数上。
%
% 这是实测选出来的唯一可行绑定。绑基础工作区变量那条路看着也通：能建起来、isBound 返回
% 真、控件自己的 setValue 也真的改得动那个变量，但仿真运行中改那个变量，仿真完全不理。
% 见 verify/bench_live_check.m 的 A 与 B 两个分支。
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
% 把 {块路径, 端口} 或 N×2 元胞表统一成 Simulink.HMI.SignalSpecification 元胞数组。
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
