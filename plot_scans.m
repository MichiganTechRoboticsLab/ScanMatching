%This script plots all of the data from the tryicp1 script.  It is not
%included in the tryicp1 script for simplicity

% Plot Map
figure(1)

if 1
    if ~isempty(world)
        plot(world(1,:), world(2,:), 'b.');
    end
else
    plot(map(1,:), map(2,:), 'b.');
end
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