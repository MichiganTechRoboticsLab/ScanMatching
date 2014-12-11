clear all
close all
clc
load newdata.mat

%full map
map = [];

%n = size(LidarScan,1);
n = 1000;
est_x = [0;0];
est_dx = [];
est_theta = 0;
est_dtheta = 0;

%minimum turn, this is the error for sensing if the robot is actually
%turning or not.  I know that the robot currently turns at 45 degrees per
%second, so anything less the that should be noted.
error_dtheta = deg2rad(3);

for nScan = 55:2:n
    
    theta = LidarPose(nScan,3);
    a = LidarAngles;
    z = LidarScan(nScan, :);

    %  Remove out of range measurements
    I = (z >= LidarRange);
    a(I) = [];
    z(I) = [];

    [x,y] = pol2cart(a,z);

    raw = [x;y];
    if isempty(map)
        map = [map raw];
    else
 
    dx = 0;
    ii = 0;
    
    last_est_dx=[0;0];
    last_est_dtheta=0;
    if ~isempty(est_dx)
        %make a quick guess based on the estimated dx
        last_est_dx=est_dx;
        TT = repmat(est_x(:,end) + last_est_dx, 1, size(raw,2));
        phi = -est_theta + last_est_dtheta;
        TR = [cos(phi) (-sin(phi)); sin(phi) cos(phi)];
        %TR = eye(2);
        raw = TR*raw + TT;
        
    end
    
    est_dx = [0;0];
    est_dtheta = 0;
    while true
        
        ii = ii + 1;
        temp = killoutliers(map,raw, 0.1);
        [TR,TT] = call_icp1(map,temp);

        TR = TR(1:2,1:2);
        TT = repmat(TT(1:2), 1, size(raw,2));

        raw = TR * raw + TT;
        est_dx = est_dx + TT(:,1);
        
        est_dtheta = est_dtheta + acos(TR(1,1));
        
        %nScan;
        %fprintf('current sub iteration : %d\ncurrent major iteration: %d\n', ii, nScan);
        dx = abs(sqrt(TT(1,1)^2 + TT(2,1)^2));
        if ii > 1
            if abs(dx) < 0.00000001
                new_x = est_x(:,end) + est_dx + last_est_dx;
                est_x = [est_x new_x];
                
                if est_dtheta < error_dtheta;
                    est_dtheta = 0;
                end
                est_theta = real(est_theta + est_dtheta + last_est_dtheta);
                break;
            end
        end
        
    end

    map = [map raw];

    figure(1)
    plot(raw(1,:), raw(2,:), 'r.')
    hold on
    plot(est_x(1,:), est_x(2,:), 'k.');
    end
    
    plot(map(1,:), map(2,:), 'b.')
    hold off
    
    rad2deg(est_theta)
    
    pause(0.1);
end