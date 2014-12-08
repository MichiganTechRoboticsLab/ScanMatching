%function [ tr, tt ] = icp3(m, d)
%ICP Ittertive Closest Point
%   Author: Dereck Wonnacott (2014)
%   
%   I wanted better insight into the errors I was seeing in other ICP
%   implementations, so I made my own implementation.
%
%   

if 1
    clear
    load('SimpleTest1.mat') % m, d
end

% 3D points are required for this implementation
if(size(m,1) == 2)
    m = [m; zeros(1, size(m,2))];
end
if(size(d,1) == 2)
    d = [d; zeros(1, size(d,2))];
end

% Itteration
D = d;
for k = 1:15
    k 
    
    % For each point in d find closest point in m
    Mdist = zeros(size(D,2), size(m,2));

    for i=1:size(D,2)
        dp = D(:,i);

        for j=1:size(m,2)
            mp = m(:,j); 

            Mdist(i, j) = sum((dp - mp).^2); % d^2 should be fine
        end    
    end
    [V, I] = min(Mdist, [], 2);
    err(k) = sum(sum(Mdist));
    

    % Solve the transform between the point sets
    [ t, q, s ] = minimize(m(:,I), D);
   
    
    % Final Solution
    D = s * quatrotate(q',D')' - repmat(t, 1, size(D,2));
end   
   
    
    
    % Debug Plot ( Input Data)
    figure(1);
    clf
    plot(m(1,:),m(2,:),'.b');
    hold on;
    plot(d(1,:),d(2,:),'.r');
    axis equal
    grid
    title(['Input' num2str(k)]);
    
    % Debug Plot - Result    
    figure(2);
    clf
    plot(m(1,:),m(2,:),'.b');
    hold on;
    plot3(D(1,:),D(2,:),D(3,:),'.r');
    axis equal
    grid
    title(['Output' num2str(k)]);


%end










