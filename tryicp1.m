%This is the first attempt at writing an icp algorithm using icp_1
%It takes the approach of 

clear all
close all
clc
load newdata.mat

path=[];

%full map
map = [];

%estimated pose for the robot in the world
est_x = [0;0];
est_dx = [];
est_theta = 0;
est_dtheta = 0;

last_est_dx=[0;0];
last_est_dtheta=0;

%minimum turn, this is the error for sensing if the robot is actually
%turning or not.  I know that the robot currently turns at 45 degrees per
%second, so anything less the that should be noted.
error_dtheta = deg2rad(3);

%I started at 55 because that's a nice place to see a small forward and
%turn going on
for nScan = 55:2:size(LidarScan,1)
    
    %Grab lidar angles and the scan
    a = LidarAngles;
    z = LidarScan(nScan, :);

    %  Remove out of range measurements
    I = (z >= LidarRange);
    a(I) = [];
    z(I) = [];

    [x,y] = pol2cart(a,z);

    %fill the map if it doesn't exist yet.
    raw = [x;y];
    
    %used to keep track of everything
    disp = [0;0];

    if isempty(map)
        map = [map raw];
    else
 
    if ~isempty(est_dx)
        %make a quick guess based on the estimated dx
        last_est_dx=est_dx;
        last_est_dtheta=est_dtheta;
        TT = repmat(est_x(:,end) + last_est_dx, 1, size(raw,2));
        phi = -(est_theta + last_est_dtheta);
        TR = [cos(phi) (-sin(phi)); sin(phi) cos(phi)];
        %TR = eye(2);
        raw = TR*raw + TT;
        disp = TR*disp + TT(:,1);
    end
    
    %sest the estimated pose diff to zero
    est_dx = [0;0];
    est_dtheta = 0;
    
    ii = 0;
    while true
        
        ii = ii + 1;
        
        %kill all the points which are not "close" to the world and run 1
        %iteration of icp on them
        temp = killoutliers(map,raw, 0.1);
        [TR,TT] = call_icp1(map,temp);

        %move the points sample based on the icp output and update the
        %estimated pose
        TR = TR(1:2,1:2);
        TT = repmat(TT(1:2), 1, size(raw,2));

        raw = real(TR * raw + TT);
        disp = real(TR * disp + TT(:,1));

        est_dx = est_dx + TT(:,1);
        est_dtheta = est_dtheta + acos(TR(1,1));
        
        %get the difference in TT so we know when to converge the icp
        %algorithm.  This is a pretty schotty way of convergence, as it
        %doesn't take into account if the value bounced around the setpoint
        %but it works for this application
        dTT = mag(TT(:,1));
        
        %we need to run the loop at least once before we check this
        if ii > 1
            
            %Check for convergence
            converge_metric = 1e-6;
            if abs(dTT) < converge_metric
                
                
                %update our state estimates
                if mag(est_dx + last_est_dx) > 1e-5
                    new_x = est_x(:,end) + est_dx + last_est_dx;
                else
                    new_x = est_x(:,end);
                end
                
                path = [path disp];
                est_x = [est_x new_x];

                %make sure our theta diff isn't just noise.  I still need
                %to do this for the displacement
                if est_dtheta < error_dtheta;
                    est_dtheta = 0;
                end
                est_theta = real(est_theta + est_dtheta + last_est_dtheta);
                break;
            end
        end
        
    end

    %update the world
    map = [map raw];

    
    %plot everything
    figure(1)
    plot(raw(1,:), raw(2,:), 'r.')
    hold on
    plot(est_x(1,:), est_x(2,:), 'k.');
    end
    
    plot(map(1,:), map(2,:), 'b.')
    
    if ~isempty(path)
        plot(path(1,:), path(2,:), 'mo')
    end
    hold off
    
    %pause so we can display things
    pause(0.1);
end