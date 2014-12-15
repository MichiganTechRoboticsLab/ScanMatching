% 2D SLAM Demo 
% Author: D5ereck Wonnacott (2014)

% Load dataset
clear
load('../datasets/Sim World 1 - 5Deg - 5Hz.mat');

MatchToMap = 1;

%
% Generate the pose graph
%
%PoseGraph = zeros(4,size(LidarScan,1));
PoseGraph = zeros(4,1);


% init the map with the first scan
if MatchToMap
    Map = scan2cart(LidarAngles, LidarScan(1,:), LidarRange);
    Trajectory = PoseGraph(:,1);
end

% Process all scans
for nScan = 2:size(LidarScan,1)
    % Status
    disp(['Scan ' num2str(nScan) ' of ' num2str(size(LidarScan,1)) '.']);
    
    % New Scan Data
    d = scan2cart(LidarAngles, LidarScan(nScan  ,:), LidarRange);
    
    % Scan to Scan matching
    if MatchToMap

        % Transform the new scan to the previous sensor location
        dm = rotate2d(-Trajectory(3, nScan-1), d);
        dm(1,:) = dm(1,:) + Trajectory(1, nScan-1);
        dm(2,:) = dm(2,:) + Trajectory(2, nScan-1);  
        
        m = Map;
        d = dm;
    else
        m = scan2cart(LidarAngles, LidarScan(nScan-1,:), LidarRange);
    end
        
    %ICP 
    [tr, tt] = call_icp1(m,d);
    %[tr, tt] = call_icp2(m,d);
    %[tr, tt] = call_libicp(m,d);
    %[tr, tt] = call_icp3(m,d);

    % Store the result in the pose graph
    PoseGraph(:, nScan) = [tt(1); tt(2); -acos(tr(1)); 1/LidarHz;];
    
    % Update the map and trajectory
    if MatchToMap
        [ Map, Trajectory ] = MakeMap( PoseGraph, LidarScan, LidarAngles, LidarRange );
    end
end


% Ground Truth PoseGraph
PoseGraphTruth = zeros(4,size(LidarScan,1));
for nScan = 2:size(LidarScan,1)
    p1 = LidarPose(nScan - 1, :);
    p2 = LidarPose(nScan, :);
    PoseGraphTruth(:, nScan) = (p2-p1)';

    % Rotate ground truth to current lidar frame.
    PoseGraphTruth([1,2], nScan) = rotate2d(p1(3), PoseGraphTruth([1,2], nScan));
end


%
% Plot the Pose Graph Error
%
figure(3)
clf;
subplot(2,1,1);
err = sqrt( (PoseGraphTruth(1,:) - PoseGraph(1,:)).^2 + (PoseGraphTruth(2,:) - PoseGraph(2,:)).^2 );
plot(err);
title('Translation Error');

subplot(2,1,2);
err = PoseGraphTruth(3,:) - PoseGraph(3,:);
plot(err);
title('Rotation Error');


%
% Plot the scan matcher results frame by frame
%
if 0
    h = figure(4);
    for nScan = 2:size(LidarScan,1) 
        % read in the inputs
        d = scan2cart(LidarAngles, LidarScan(nScan  ,:),LidarRange); 
        m = scan2cart(LidarAngles, LidarScan(nScan-1,:),LidarRange); 

        % Rotate the new points
        dm = rotate2d(-PoseGraph(3, nScan), d);

        % Translate the new points
        dm(1,:) = dm(1,:) + PoseGraph(1, nScan);
        dm(2,:) = dm(2,:) + PoseGraph(2, nScan);

        % Plot        
        set(0, 'CurrentFigure', 4);
        clf
        plot(m(1,:), m(2,:), '+b');
        hold on
        plot(d(1,:), d(2,:), '.k');
        plot(dm(1,:), dm(2,:), 'Or');
        axis equal
        grid
        title(['Scan Matcher Inputs (' num2str(nScan) ')' ]);
        legend('Model', 'Input', 'Output')
        
        % Save to disk
        %saveas(h, ['../images/' num2str(nScan) '.png'])
        
        pause(1/LidarHz+1);
    end
end

[ Map, Trajectory ] = MakeMap( PoseGraph, LidarScan, LidarAngles, LidarRange );

%
% Plot the map
%
figure(5);
clf;
plot(Trajectory(1,:), Trajectory(2,:), '.-r');
hold on;
grid;
axis equal;
plot(Map(1,:), Map(2,:), '.k')


clear a d dp i I m nScan p p1 p2 scan tr tt x y




