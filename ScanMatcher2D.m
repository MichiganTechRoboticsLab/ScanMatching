% 2D SLAM Demo 
% Author: Dereck Wonnacott (2014)

% Load dataset
clear
load('../datasets/Sim World 1 - 025Deg - 10Hz.mat');

% Add libraries
addpath('./libicp/matlab/');


%
% Generate the pose graph
%
PoseGraph = zeros(4,size(LidarScan,1));


for nScan = 2:size(LidarScan,1)
    % New Scan Data
    d = scan2cart(LidarAngles, LidarScan(nScan  ,:), LidarRange); % Data
    m = scan2cart(LidarAngles, LidarScan(nScan-1,:), LidarRange); % Model
  
   
    % Ground Truth Data
    if 1
        p1 = LidarPose(nScan - 1, :);
        p2 = LidarPose(nScan, :);
        dp = p2-p1;

        % Rotate ground truth to current lidar frame.
        dp([1,2]) = rotate2d(p1(3), dp([1,2])');

        % Rotate the data points to the expected rotation
        dm = rotate2d(-PoseGraph(3, nScan), d([1 2],:));

        % Translate the new points to the expected location
        dm(1,:) = dm(1,:) + PoseGraph(1, nScan);
        dm(2,:) = dm(2,:) + PoseGraph(2, nScan);
    end
    
    % For implementations that require 3D points
    if 0
        d  = [d; zeros(1, size(d,2))];
        m  = [m; zeros(1, size(m,2))];
        dm = [dm; zeros(1, size(dm,2))];
    end

    %ICP1 [tr, tt] = icp(m,dm,15, 'Minimize', 'point');
    %ICP2 [tr, tt] = icp(m,dm);
    Tr_fit = icpMex(m,dm,eye(3),-1,'point_to_plane');
    tt = Tr_fit([1 2], 3);
    tr = Tr_fit([1 2], [1 2]);
    
    dp    = real(tt);
    dp(3) = -acos(tr(1));
    dp(4) = 0; % Timestamp (isn't used yet)
    
    % Store the result in the pose graph
    PoseGraph(:, nScan) = dp;
end


%
% Plot the scan matcher results frame by frame
%
if 0
    h = figure(3);
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
        set(0, 'CurrentFigure', 3);
        clf
        plot(m(1,:), m(2,:), '.b');
        hold on
        plot(d(1,:), d(2,:), 'Ok');
        plot(dm(1,:), dm(2,:), 'Or');
        axis equal
        grid
        title(['Scan Matcher Inputs (' num2str(nScan) ')' ]);
        legend('Model', 'Input', 'Output')
        
        % Save to disk
        %saveas(h, ['../images/' num2str(nScan) '.png'])
        
        pause(1/LidarHz);
    end
end

%
% Generate the trajectory from the pose graph
%
traj = PoseGraph(:,1);
for i = 2:size(PoseGraph,2)
    p  = traj([1, 2], i-1);
    dp = rotate2d(-traj(3, i-1), PoseGraph([1,2],i));

    traj([1, 2], i) = p + dp;
    traj([3, 4], i) = traj([3, 4], i-1) + PoseGraph([3, 4],i);
end


%
% Generate the map
%
Map  = [];
for nScan = 1:size(LidarScan,1)
    scan = LidarScan(nScan,:);
    a    = LidarAngles;
        
    % Remove out of range measurements
    I = (scan >= LidarRange);
    scan(I) = [];
    a(I)    = [];
    
    % Rotation
    a = a + traj(3, nScan);
    [x, y] = pol2cart(a, scan);

    % Translation
    x = x + traj(1, nScan);
    y = y + traj(2, nScan);
    d = [x; y];

    % Add points to map.
    Map = cat(2, Map, d);
end


%
% Plot the map
%
figure(4);
clf;
plot(traj(1,:), traj(2,:), '-r');
hold on;
grid;
axis equal;
plot(Map(1,:), Map(2,:), '.k')


clear a d dp i I m nScan p p1 p2 scan tr tt x y
