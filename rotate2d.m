function [ points ] = rotate2d( angle , points)
%ROTATE2D - rotates points in 2D
% angle: radians to rotate
% points: 2 x n matrix of points [x; y]
    
    r = [cos(angle), -sin(angle);
         sin(angle), cos(angle)];

    points = (points' * r)';

end

