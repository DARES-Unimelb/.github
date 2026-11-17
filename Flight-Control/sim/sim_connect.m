function sim_connect(varargin)
% SIM_CONNECT add_line with autorouting, from 'src/port' to 'dst/port' pairs
%
%   sim_connect(mdl, 'a/1', 'b/1')
%   sim_connect(mdl, 'a/1', 'b/1', 'c/1', 'd/2')   two lines in one call

mdl = varargin{1};
for k = 2:2:numel(varargin)
   add_line(mdl, varargin{k}, varargin{k + 1}, 'autorouting', 'on');
end
end
