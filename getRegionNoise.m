function [ noise ] = getRegionNoise( noisy, rect, inside, orig )
%GETREGIONNOISE 
%   noisy       Noisy image
%   rect        Rectangle position (see getrect)
%   inside      Uses values inside the rectangle if true (default)
%   orig        Reference image if available (can be omitted)

if nargin < 3
    inside = true;
end
if nargin < 4
    orig = zeros(size(noisy));
end

pos = round(rect);
xrange = pos(1)+1:pos(1)+pos(3);
yrange = pos(2)+1:pos(2)+pos(4);

Idx = false(size(noisy));
Idx(xrange,yrange) = true;

% Invert for outside values
if ~inside
    Idx = ~Idx;
end

noise = noisy(Idx) - orig(Idx);
noise = noise(:);

end

