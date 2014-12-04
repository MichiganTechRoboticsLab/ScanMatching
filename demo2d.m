% 2D SLAM Demo 
% Author: Dereck Wonnacott (2014)

%
% Generate the pose graph
%

t = [0; 0; 0; 0]; % Starting Point

TT = [0; 0; 0;];
TR = zeros(3);

for nScan = 2:size(LidarScan,1)
    % New Scan Data
    d = scan2cart(LidarAngles, LidarScan(nScan  ,:),LidarRange); % Data
    m = scan2cart(LidarAngles, LidarScan(nScan-1,:),LidarRange); % Model
  
    % Ground Truth Pose Data
    if 1
        p1 = LidarPose(nScan - 1, :);
        p2 = LidarPose(nScan, :);
        dp = p2-p1;

        % Rotate ground truth to current lidar frame.
        dp([1,2]) = rotate2d(p1(3), dp([1,2])');
        
    % ICP
    else
        % This implementation requires 3D points
        d = [d; zeros(1, size(d,2))];
        m = [m; zeros(1, size(m,2))];
        
        [tr, tt] = icp(m,d,30);
        
        TT(:, nScan) = tt;
        TR(:, :, nScan) = tr;
        
        dp    = real(tt);
        dp(3) = -acos(tr(1));
        dp(4) = 0; % Timestamp (isn't used yet)
    end
    
    % Store the result in the pose graph
    t(:, nScan) = dp;
end


%
% Plot the scan matcher results frame by frame
%
if 1
for nScan = 2:size(LidarScan,1) 
    % read in the inputs
    d = scan2cart(LidarAngles, LidarScan(nScan  ,:),LidarRange); 
    m = scan2cart(LidarAngles, LidarScan(nScan-1,:),LidarRange); 

    % Rotate the new points
    dm = rotate2d(-t(3, nScan), d);
    
    % Translate the new points
    dm(1,:) = dm(1,:) + t(1, nScan);
    dm(2,:) = dm(2,:) + t(2, nScan);

    % Plot
    figure(3);
    clf
    plot(m(1,:), m(2,:), '.b');
    hold on
    plot(d(1,:), d(2,:), 'Ok');
    plot(dm(1,:), dm(2,:), 'Or');
    axis equal
    grid
    title('Scan Matcher Inputs');
    legend('Model', 'Input', 'Output')
    pause(1/LidarHz);
end
end

%
% Generate the trajectory from the pose graph
%
traj = t(:,1);
for i = 2:size(t,2)
    p  = traj([1, 2], i-1);
    dp = rotate2d(-traj(3, i-1), t([1,2],i));

    traj([1, 2], i) = p + dp;
    traj([3, 4], i) = traj([3, 4], i-1) + t([3, 4],i);
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



