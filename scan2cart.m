function [ d ] = scan2cart( angles, scan, maxrange )
%SCAN2CART - Convert lidar scan to cartiesan coordinates

    % Remove out of range measurements
    I = (scan >= maxrange);
    scan(I) = [];
    angles(I)    = [];
    
    % Convert lidar to cartisian coordinates
    [x, y] = pol2cart(angles, scan);
    d = [x; y];

end

