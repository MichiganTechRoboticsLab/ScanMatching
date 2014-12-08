function [ t, q, s ] = minimize( m, d )
%MINIMIZE Find the transform between two pointsets
% with known correspondances based on:
% [1] Closed-form solution of absolute orientation using unit quaternions
%
% Inputs:
%   m - 3xN matrix of fixed 3D points
%   d - 3xN matrix of transformed 3D points to be matched to m
%
% Output:
%   t - 3x1 column vector representing the translation
%   q - 4x1 quaterion representing the rotation
%   s - 1x1 scaler representing the scale ratio
%
% Transform d to m:
%   D = s * quatrotate(q',d')' - repmat(t, 1, size(d,2));
%


    % Translation (Centroid)
    um = mean(m,2); 
    ud = mean(d,2);
    Np = size(m,2);

    m1 = m - repmat(um,1,Np);
    d1 = d - repmat(ud,1,Np);

    % Scale
    s = 1/sqrt(sum(d1.^2) / sum(m1.^2));

    d2 = s*d1;


    % Rotation

    % 'pseudo cross correlation matrix' 
    M = zeros(3);
    for i = 1:Np
        M = M + m1(:,i) * d2(:,i)';
    end

    % N Matrix
    N = zeros(4);

    t = M - M';
    a = [t(2,3); t(3,1); t(1,2)]; 

    N(1  , :  ) = [trace(M) a'];
    N(2:4, 1  ) = a;
    N(2:4, 2:4) = M + M' - trace(M)*eye(3);

    % Find eigenvector with max eigenvalue
    [v, e] = eig(N);
    [~, I] = max(max(e));
    q = v(:,I);

    % Find the final translation
    % Remove rotation and scale
    d4 = s*quatrotate(q',d')';

    % calculate the centroid
    t = mean(d4,2) - um;

    

    return
    
    
    %
    % Debug Plots
    %
    
    % Plot
    figure(1)
    clf
    plot3(0,0,0,'+K');
    hold on
    plot3(m(1,:), m(2,:), m(3,:), '.b');
    plot3(d(1,:), d(2,:), d(3,:), '.r');
    axis equal
    grid on
    title('Input Point Sets');
    legend('Origin', 'Model','Data');

    % Plot
    figure(2)
    clf
    plot3(0,0,0,'+K');
    hold on
    plot3(m1(1,:), m1(2,:), m1(3,:), '.b');
    plot3(d1(1,:), d1(2,:), d1(3,:), '.r');
    axis equal
    grid on
    title('Centered Point Sets');
    legend('Origin', 'Model','Data');

    % Plot    
    d3 = quatrotate(Q',d2')';
    
    figure(3)
    clf
    plot3(0,0,0,'+K');
    hold on
    plot3(m1(1,:), m1(2,:), m1(3,:), '.b');
    plot3(d2(1,:), d2(2,:), d2(3,:), '.r');
    axis equal
    grid on
    title('Scaled Point Sets');
    legend('Origin', 'Model','Data');

    % Plot
    figure(4)
    clf
    plot3(0,0,0,'+K');
    hold on
    plot3(m1(1,:), m1(2,:), m1(3,:), '.b');
    plot3(d3(1,:), d3(2,:), d3(3,:), 'Or');
    axis equal
    grid on
    title('Rotated Point Sets');
    legend('Origin', 'Model','Data');

    % Plot
    d5 = d4 - repmat(t, 1, Np);
    
    figure(5)
    clf
    plot3(0,0,0,'+K');
    hold on
    plot3(m(1,:), m(2,:), m(3,:), '.b');
    plot3(d5(1,:), d5(2,:), d5(3,:), 'Or');
    axis equal
    grid on
    title('Rotated Point Sets');
    legend('Origin', 'Model','Data');
end

