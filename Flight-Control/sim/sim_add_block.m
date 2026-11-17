function path = sim_add_block(mdl, name, lib, pos, varargin)
% SIM_ADD_BLOCK add_block wrapper that returns the new block path
%
%   path = sim_add_block(mdl, name, lib, pos, 'Param', value, ...)
%
% name must not contain a slash, which Simulink reads as a path separator

assert(isempty(strfind(name, '/')), 'sim_add_block:SlashInName', ... %#ok<STREMP>
   '块名不能含斜杠（Simulink 的路径分隔符）：%s。单位写 mps 或 deg 这样的缩写', name);

path = [mdl '/' name];
add_block(lib, path, 'Position', pos, varargin{:});
end
