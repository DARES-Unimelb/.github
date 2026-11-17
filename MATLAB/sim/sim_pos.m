function p = sim_pos(col, row, w, h)
% SIM_POS Grid position for a block, so the generated diagrams stay readable.
%
% col and row are zero based; w and h are the block size in pixels.

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
