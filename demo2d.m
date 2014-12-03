
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
% Generate Robot Path
%

% Initialize Robot
P = [0.5; 0.5];
R = pi/2;

% Plot a trajectory
T = [0.5, 0.5, R;
     0.5, 4.0, pi/2;
     0.5, 4.0, 0;
     2.5, 4.0, 0];
Vp = 0.5;  % Linear Velocity (Units/Second)
Vr = pi/4; % Rotational velocity (Rad/Second)

figure(1);
plot(T(:,1)', T(:,2)', '--r');

% Generate timestamps for each pose
Tt = [0];
for i = 1:(length(T)-1)
   dp = sqrt((T(i,1) - T(i+1,1))^2 + (T(i,2) - T(i+1,2))^2);
   tp = dp/Vp;
   
   dr = T(i,3) - T(i+1,3); 
   tr = dr/Vr;   
   
   t  = max(tp, tr); 
   Tt = cat(2, Tt, t);
end 


% Genearte Sample Poses
Hz = 5;    % Samples Per Second
LidarPose = []; % Ground Truth Lidar poses

i = 1;
for i = 1:(length(T)-1)
    x  = [0; Tt(i+1)];
    xx = x(1):1/Hz:x(2);
   
    y  = [T(i,1); T(i+1,1)];
    px = linterp(x,y,xx);
    
    y  = [T(i,2); T(i+1,2)];
    py = linterp(x,y,xx);
    
    y  = [T(i,3); T(i+1,3)];
    pr = linterp(x,y,xx);
    
    LidarPose = cat(1, LidarPose, [px' py' pr' xx']);
end



% Plot Robot Location
r = 0.25; % Robot Radius
circle(P(1), P(2), r, '-r');
plot([P(1), P(1)+r*cos(R)],[P(2), P(2)+r*sin(R)], '-r')

% Lidar Sensor
LidarRange = 4.0;
LidarAngles = deg2rad(-135):deg2rad(5):deg2rad(135);
LidarScan = [];
for i = 1:length(LidarAngles);
    r = LidarRange;
    a = LidarAngles(i);
    
    % find endpoints of each beam
    beam = [P(1), P(1)+r*cos(R+a); P(2), P(2)+r*sin(R+a)];
    
    % Find if this range measurement intersects any map line
    for j = 1:length(W)
        [xi, yi] = polyxpoly(W{j}(1,:), W{j}(2,:), beam(1,:), beam(2,:)); 
        if ~isempty(xi)
            
            % Find the first hit
            for k = 1:length(xi)
                d = sqrt((P(1) - xi(k)).^2 + (P(2) - yi(k)).^2);
                if d < r 
                    r = d;
                end
            end
        end
    end
    
    % Show this measurement
    plot([P(1), P(1)+r*cos(R+a)],[P(2), P(2)+r*sin(R+a)], '-g')
    LidarScan = cat(2, LidarScan, r);
end


% Show the Lidar Measurement
%  Remove out of range measurements
I = (LidarScan >= LidarRange);
LidarAngles(I) = [];
LidarScan(I) = [];

figure(2)
clf
polar(LidarAngles, LidarScan, '.b')





