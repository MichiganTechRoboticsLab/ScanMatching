function [ t, q, s ] = icp3(m, d)
%ICP Ittertive Closest Point
%   Author: Dereck Wonnacott (2014)
%   
%   I wanted better insight into the errors I was seeing in other ICP
%   implementations, so I made my own implementation.
%
%   

    if 0 
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
        % Find the point corrispondances
        [mi, di] = match(m, D);

        % Solve the transform between the point sets
        [ t, q, s ] = minimize(m(:,mi), D(:,di));

        % Final Solution
        D = s * quatrotate(q',D')' - repmat(t, 1, size(D,2));
    end

    % Final solution
    [ t, q, s ] = minimize(D, d);
    D2 = s * quatrotate(q',d')' - repmat(t, 1, size(d,2));
    
    
    return
    
    % Debug Plot ( Input Data)
    figure(1);
    clf
    plot3(m(1,:),m(2,:),m(3,:),'+b', 'MarkerSize', 7);
    hold on;
    plot3(d(1,:),d(2,:),d(3,:),'.r', 'MarkerSize', 7);
    plot3(D2(1,:),D2(2,:),D2(3,:),'Or');
    axis equal
    grid
    title(['ICP Result k = ' num2str(k)]);
    
    % Plot correspondence
    for i = 1:size(mi,1)
        plot3([m(1,mi(i)) D(1,di(i))], [m(2,mi(i)) D(2,di(i))], [m(3,mi(i)) D(3,di(i))], 'r'); 
        plot3([m(1,mi(i)) d(1,di(i))], [m(2,mi(i)) d(2,di(i))], [m(3,mi(i)) d(3,di(i))], 'b'); 
    end
    legend('Model', 'Data', 'Result');
    
    
    view(0,90)
    pause(1)
end










