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

% Intial registation
tt = zeros(3,1);
tr = eye(3);
    
for k = 1:60
    % move data points to current estimate
    d = tr*d + repmat(tt(:,k),1,size(d,2));
    
    if 0
        % Debug Plot
        figure(1);
        clf
        plot(m(1,:),m(2,:),'.b');
        hold on;
        plot(d(1,:),d(2,:),'.r');
        axis equal
        grid
        
        title(num2str(k));
    end
    
    % For each point in d find closest point in m
    Mdist = zeros(size(d,2), size(m,2));

    for i=1:size(d,2)
        dp = d(:,i);

        for j=1:size(m,2)
            mp = m(:,j); 

            Mdist(i, j) = sum((dp - mp).^2); % d^2 should be fine
        end    
    end
    [V, I] = min(Mdist, [], 2);
    err(k) = sum(sum(Mdist));


    % eq 22 - Notation
    p = d;
    x = m(:,I);
    Np = size(p,2);
    Nx = size(x,2); % Equal to Np, but for the sake of notation....

    % eq 23 - Center of mass
    Up = 1/Np * sum(p,2);
    Ux = 1/Nx * sum(x,2);

    plot(Ux(1),Ux(2),'+b');
    plot(Up(1),Up(2),'+r');

    % eq 24 - Cross-Covariance matrix
    px = zeros(size(p,1));
    for i = 1:Np
        px = px + p(:,i)' * x(:,i);
    end
    Epx = (1/Np * px) - Up'*Ux;

    % eq 25 - Q matrix
    A = Epx - Epx';
    D = [A(2,3); A(3,1); A(1,2)]; 

    Q = zeros(4);
    Q(1,:) = [trace(Epx) D'];
    Q([2:4],1) = D;
    Q([2:4],[2:4]) = Epx + Epx' - trace(Epx)*eye(size(Epx,1));

    [V, D] = eig(Q);
    [~, I] = max(max(D));
    Q = V(:,I);
    Q = [1;0;0;0];
    
    % eq26
    R = [ Q(1)^2 + Q(2)^2 - Q(3)^2 - Q(4)^2, ...
          2*(Q(2)*Q(3) - Q(1)*Q(4)), ...
          2*(Q(2)*Q(4) + Q(1)*Q(3));

          2*(Q(2)*Q(3) + Q(1)*Q(4)), ...
          Q(1)^2 + Q(3)^2 - Q(2)^2 - Q(4)^2, ...
          2*(Q(3)*Q(4) - Q(1)*Q(2));

          2*(Q(2)*Q(4) - Q(1)*Q(3)), ...
          2*(Q(3)*Q(4) + Q(1)*Q(2)), ...
          Q(1)^2 + Q(4)^2 - Q(2)^2 - Q(3)^2;];

    T = Ux - R*Up

    % Final Solution
    tt(:,k+1) = tt(:,k) + T;
    tr = R*tr;
    
    k
end

%end










