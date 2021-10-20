function [diff] = checkFrameDiff(frame)
%checkFrameDiff Here we check the difference of frames between samples 
%   The ideal number of frame separation is 8, which is what is seen on the
%   commercially available code

diff = [frame; 0] - [0;frame];

diff(1) = [];
diff(end) = [];

end

