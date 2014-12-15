function [ mi, di ] = match( m, d )
% MATCH - Brute force Point to Point search
%
% Author: Dereck Wonnacott (2014)

    % For each point in d find closest point in m
    Mdist = zeros(size(d,2), size(m,2));

    for i=1:size(d,2)
        dp = d(:,i);

        for j=1:size(m,2)
            mp = m(:,j); 

            Mdist(i, j) = sum((dp - mp).^2);
        end    
    end
    [V, I] = min(Mdist, [], 2);
    
    % Outlier Rejection (X * STD) 
    i = V <= (mean(V) + std(V) * 0.25);
    mi = I(i);
    
    di = 1:size(d,2);
    di = di(i);

end