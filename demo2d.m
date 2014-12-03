% Simulate a 2D robot

%
% Generate the virtual world (Ground Truth Map)
%

% W is an array of objects in the map
% Each object is a polyline stored in a 2xn matrix
% Where n is the number of line endpoints
% the first row is the x coordinates
% the second row is the y coordinates. 
W{1} = [0 0 5 5 0;
        0 5 5 0 0];
    
W{2} = [1 1 2 2 1;
        1 3 3 1 1];
    
W{3} = [3 4 4 3 3;
        3 3 4 4 3];

% Plot the world
figure(1); 
clf;
title('World Map');
for i = 1:length(W)
    line(W{i}(1,:), W{i}(2,:))
end
hold on;
axis equal
grid


%
% Generate Robot Path (Ground Truth)
%

% Robot Trajectory 
% [x, y, Theta]
T = [0.5, 0.5, deg2rad(90);
     0.5, 4.0, deg2rad(90);
     0.5, 4.0, 0;
     2.5, 4.0, 0];

Vp = 0.5;  % Linear Velocity (Units/Second)
Vr = deg2rad(45); % Rotational velocity (Rad/Second)

% Plot Trajectory
figure(1);
plot(T(:,1)', T(:,2)', '-r');


% Generate timestamps for each trajectory vertex
tt = 0;
for i = 1:(length(T)-1)
   dp = sqrt((T(i,1) - T(i+1,1))^2 + (T(i,2) - T(i+1,2))^2);
   tp = dp/Vp;
   
   dr = T(i,3) - T(i+1,3); 
   tr = dr/Vr;   
   
   t  = max(tp, tr); 
   tt = cat(2, tt, t);
end 
T = cat(2, T, tt');


%
% Generate Lidar Data
%

Hz = 5; % Samples Per Second

% lidar poses
LidarPose = []; 

i = 1;
for i = 1:(length(T)-1)
    x  = [0; tt(i+1)];
    xx = x(1):1/Hz:x(2);
   
    y  = [T(i,1); T(i+1,1)];
    px = linterp(x,y,xx);
    
    y  = [T(i,2); T(i+1,2)];
    py = linterp(x,y,xx);
    
    y  = [T(i,3); T(i+1,3)];
    pr = linterp(x,y,xx);
    
    LidarPose = cat(1, LidarPose, [px' py' pr' xx']);
end

plot(LidarPose(:,1),LidarPose(:,2), '.r');



% Generate Lidar Sensor measurements
LidarRange = 4.0;
LidarAngles = deg2rad(-135):deg2rad(5):deg2rad(135);
LidarScan = [];
for n = 1:size(LidarPose,1) % For each pose
    p = LidarPose(n,:); % Current pose
    z = []; % Current scan
    
    for i = 1:length(LidarAngles); % For each beam
        r = LidarRange; % Max Range
        a = LidarAngles(i); % Current Angle

        % find endpoints of this beam
        beam = [p(1), p(1)+r*cos(p(3)+a); p(2), p(2)+r*sin(p(3)+a)];

        % Search if this range measurement intersects any map line
        for j = 1:length(W)
            [xi, yi] = polyxpoly(W{j}(1,:), W{j}(2,:), beam(1,:), beam(2,:)); 
            if ~isempty(xi)
                % Find the closest intersecetion
                for k = 1:length(xi)
                    d = sqrt((p(1) - xi(k)).^2 + (p(2) - yi(k)).^2);
                    if d < r 
                        r = d;
                    end
                end
            end
        end
        
        % Add this measurement to the scan
        z = cat(2, z, r);
    end
    
    LidarScan = cat(1, LidarScan, z);
    
   
    
    % Show the Lidar Measurement
    %  Remove out of range measurements
    a = LidarAngles;
    z = LidarScan(end, :);
    I = (z >= LidarRange);
    a(I) = [];
    z(I) = [];
    
    % Plot
    figure(2)
    clf
    polar(a, z, '.b')
    
    pause(1/Hz);
    
end








