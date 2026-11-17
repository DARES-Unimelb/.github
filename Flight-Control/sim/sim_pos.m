function p = sim_pos(col, row, w, h)
% SIM_POS Pixel position vector for a block on the layout grid
%
%   col, row   zero-based grid cell
%   w, h       block size in pixels, default 80 by 40
%   p          [left top right bottom]

if nargin < 3
   w = 80;
end
if nargin < 4
   h = 40;
end
x = 40 + 200 * col;
y = 40 + 110 * row;
p = [x, y, x + w, y + h];
end
