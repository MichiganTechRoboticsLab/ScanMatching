%This is the first attempt at writing an icp algorithm using icp_1
%It takes the approach of 
clear all
close all
clc
load newdata.mat

%add some noise to our data
LidarScan = LidarScan.*normrnd(1,0.01,size(LidarScan,1),size(LidarScan,2));

offset=[0;0;0];
cur=offset;

pose=[0;0;0];
post_dot=[];

%full map
map = [];

converge_metric = 5e-4;
pointer_scale = 0.25;

lidarRange = 4;


%radius to filter out points when running icp
filter_radius = lidarRange*1.2;

for nScan = 1:5:size(LidarScan,1)

    disp = [[0 1];[0 0]];
    
    %Grab lidar angles and the scan
    a = LidarAngles;
    z = LidarScan(nScan, :);

    %  Remove out of range measurements
    I = (z >= LidarRange*0.9);
    a(I) = [];
    z(I) = [];

    [x,y] = pol2cart(a,z);

    %fill the map if it doesn't exist yet.
    raw = [x;y];

    if isempty(map)
        map = [map raw];
    else
 
    if size(pose,2) > 1        
        TT = repmat(pose(1:2,end) + (pose(1:2,end) - pose(1:2,end-1)), 1, size(raw,2));
        phi = pose(3,end) + (pose(3,end)-pose(3,end-1));
        TR = [cos(phi) (-sin(phi)); sin(phi) cos(phi)];
        
        raw = TR*raw + TT;
        disp = TR*disp + TT(:,1:2);
    end
    
    %sest the estimated pose diff to zero
    %est_dtheta = 0;
    
    ii = 0;
    while true
        
        ii = ii + 1;
        
        %temp_map = map;
        temp_map = filterFarPoints(map, cur(1:2,:), filter_radius);

        %kill all the points which are not "close" to the world and run 1
        %iteration of icp on them
        temp = killoutliers(temp_map, raw, 0.1);

        [TR,TT] = call_icp1(temp_map,temp);

        %move the points sample based on the icp output and update the
        %estimated pose
        TR = TR(1:2,1:2);
        TT = repmat(TT(1:2), 1, size(raw,2));

        raw = real(TR * raw + TT);
        disp = real(TR * disp + TT(:,1:2));

        %est_dtheta = est_dtheta + acos(TR(1,1));
        
        %get the difference in TT so we know when to converge the icp
        %algorithm.  This is a pretty schotty way of convergence, as it
        %doesn't take into account if the value bounced around the setpoint
        %but it works for this application
        dTT = mag(TT(:,1))
        
        %we need to run the loop at least once before we check this
        if ii > 1
            %Check for convergence
            if abs(dTT) < converge_metric
                tt = disp(:,2) - disp(:,1);
                [theta, ~] = cart2pol(tt(1), tt(2));
                cur = ([disp(:,1);theta] - offset)
                pose = [pose cur];
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
    %plot(est_x(1,:), est_x(2,:), 'k.');
    end
    
    plot(map(1,:), map(2,:), 'b.')
    
    plot(pose(1,:), pose(2,:), 'mo')
    
    pointer = disp;
    pointer(:,2) = pointer_scale*(pointer(:,2) - pointer(:,1));
    pointer(:,2) = pointer(:,2) + disp(:,1);
    
    plot(pointer(1,:), pointer(2,:), 'k-')
    hold off
    
    %pause so we can display things
    pause(0.1);
end