% This is the first attempt at writing an icp based slam algorithm using icp_1
% Author: Josh M
% 
clear
close all
clc

% Load simulated dataset
load walkStraight.mat

% Add noise to the dataset
%LidarScan = LidarScan + normrnd(0.05,0.01,size(LidarScan,1),size(LidarScan,2));


offset=[0;0;0];
cur=offset;

pose=[0;0;0];
post_dot=[];

% full map
map = [];

%converge_metric = 5e-4;
converge_metric = 5e-3;
pointer_scale = 0.25;

LidarRange = 4;

%radius to filter out points when running icp
filter_radius = LidarRange*1.2;

nScanIndex = unique(Lidar_ScanIndex);

temp_map = [];
temp_scan = [];
for nScan = 1:50:size(nScanIndex)

    disp = [[0 1];[0 0]];
    
    %Grab lidar angles and the scan
    %a = LidarAngles;
    %z = LidarScan(nScan, :);

    % Retrieve each scan's points
    nIndex = nScanIndex(nScan);
    I = nIndex == Lidar_ScanIndex;
    a = Lidar_Angles(I,:)';
    z = Lidar_Ranges(I,:)';
    
    %  Remove out of range measurements
    I = (z >= LidarRange*0.9);
    a(I) = [];
    z(I) = [];

    [x,y] = pol2cart(a,z);

    %fill the map if it doesn't exist yet.
    raw = [x;y];

    if isempty(map)
        % initialize the map
        map = raw;
    else
 
        % Transfrom the current scan to the current estimated pose
        % based on a constantant velocity assumption.
        if size(pose,2) > 1        
            TT = repmat(pose(1:2,end) + (pose(1:2,end) - pose(1:2,end-1)), 1, size(raw,2));
            phi = pose(3,end) + (pose(3,end)-pose(3,end-1));
            TR = [cos(phi) (-sin(phi)); sin(phi) cos(phi)];

            raw = TR*raw + TT;
            disp = TR*disp + TT(:,1:2);
        end

        % ICP Loop for current scan
        ii = 0;
        while true

            ii = ii + 1;

            % Remove all points that do not have a 'near' match
            temp_scan = killoutliers(map, raw, 0.1);
            temp_map = killoutliers(temp_scan, map, 0.1);

            % Execute ICP
            [TR, TT] = call_icp1(temp_map, temp_scan);

            % Transform current scan and pose estimate to new location.
            TR = TR(1:2,1:2); % Cheap way to only select z rotation
            TT = repmat(TT(1:2), 1, size(raw,2)); 

            raw = real(TR * raw + TT);
            disp = real(TR * disp + TT(:,1:2));


            %get the difference in TT so we know when to converge the icp
            %algorithm.  This is a pretty schotty way of convergence, as it
            %doesn't take into account if the value bounced around the setpoint
            %but it works for this application
            dTT = mag(TT(:,1));

            %we need to run the loop at least once before we check this
            if ii > 1
                %Check for convergence
                if abs(dTT) < converge_metric
                    tt = disp(:,2) - disp(:,1);
                    [theta, ~] = cart2pol(tt(1), tt(2));
                    cur = ([disp(:,1);theta] - offset);
                    pose = [pose cur];
                    break;
                end
            end

        end

        % Update the world
        map = [map raw];

    end
    
    
    %
    % plot everything
    %
    
    % Plot Map
    figure(1)
    plot(map(1,:), map(2,:), 'b.');
    hold on
    
    % Plot filtered map for this scan
    %  (No outliers or out of range points)
    if ~isempty(temp_map)
        plot(temp_map(1,:), temp_map(2,:),'g.');
    end
    
    % Plot sensor poses
    plot(pose(1,:), pose(2,:), 'mo')
    
    % Plot current orientation
    pointer = disp;
    pointer(:,2) = pointer_scale*(pointer(:,2) - pointer(:,1));
    pointer(:,2) = pointer(:,2) + disp(:,1);
    plot(pointer(1,:), pointer(2,:), 'k-')
    
    % Plot raw lidar data for current scan
    plot(raw(1,:),raw(2,:), 'r.');
    
    % Plot filted scan
    if ~isempty(temp_scan)
        plot(temp_scan(1,:),temp_scan(2,:), 'm.');
    end
    
    % Plot Stuff
    title('Map');
    legend('Map', 'Filtered map', 'Pose', 'Orientatio', 'Raw Scan', 'Filtered Scan')
    axis equal;
    grid
    hold off
    
    %pause so we can display things
    pause(0.1);
end