function path = sim_add_block(mdl, name, lib, pos, varargin)
% SIM_ADD_BLOCK add_block with a position and parameter pairs, returning the path.
%
% Wrapper only, so the builders read as a list of blocks instead of a list of
% add_block calls with repeated [mdl '/name'] strings.
%
% 块名里不能有斜杠：斜杠是 Simulink 的路径分隔符，带斜杠的名字会被解析成"某某块里面的
% 某某块"，报 A new block named ... cannot be added，而报错信息里看不出是斜杠的问题。
% 这里挡一道，把原因写清楚。实测踩过两次，第二次是给自己新加的三盏状态灯起的名字。

assert(isempty(strfind(name, '/')), 'sim_add_block:SlashInName', ... %#ok<STREMP>
   '块名不能含斜杠（Simulink 的路径分隔符）：%s。单位写 mps 或 deg 这样的缩写', name);

path = [mdl '/' name];
add_block(lib, path, 'Position', pos, varargin{:});
end
