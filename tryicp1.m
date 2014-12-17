% This is the first attempt at writing an icp based slam algorithm using icp_1
% Author: Josh M
%
%clear
clear area
close all
clc

% Load simulated dataset
%load hall_and_room_set.mat

% Add noise to the dataset
%LidarScan = LidarScan + normrnd(0.05,0.01,size(LidarScan,1),size(LidarScan,2));
error = [0;0;0];

pose=[0;0;0];
pose_dot=[];

% This is what I am the last scan to, whether it be the full map or just
% scan-by-scan
map = [];

world = [];

%are we doing a scan-to-scan implementation or a scan-to-map
scanToScan = 1;

converge_metric = 1e-4;
pointer_scale = 0.25;

LidarRange = 30*0.9;

%radius to filter out points when running icp
filter_radius = LidarRange;

nScanIndex = unique(Lidar_ScanIndex);

Hokuyo_freq = 40; %Hz

temp_map = [];
temp_scan = [];

frame_skip = 20;

rejection_setting = 0.1;

scanSizes = [];

theta_offset = deg2rad(0.5);

if ~scanToScan
    warning('Running scan-to-map, this may take a while.')
end

graph.n = 50;
graph.m = 50;
graph.width = 1; %meters

empty_mat = zeros(graph.n, graph.m);
area(graph.n,graph.m) = struct('data', nan(graph.n,graph.m), 'visited', empty_mat, 'n', empty_mat);

visited = 0;

node_cur = [25 25];
last_node = [];

frame = 1;
for nScan = 500:frame_skip:size(nScanIndex)
    
    disp = [[0 1];[0 0]];
    
    % Retrieve each scan's points
    nIndex = nScanIndex(nScan);
    I = nIndex == Lidar_ScanIndex;
    
    a = Lidar_Angles(I,:)' + theta_offset;
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
            TT = repmat(pose(1:2,end) + pose_dot(1:2), 1, size(raw,2));
            %phi = pose(3,end) + (pose(3,end)-pose(3,end-1));
            phi = pose(3,end) + pose_dot(3);
            TR = [cos(phi) (-sin(phi)); sin(phi) cos(phi)];
            
            raw = TR*raw + TT;
            disp = TR*disp + TT(:,1:2);
        end
        
        % ICP Loop for current scan
        ii = 0;
        while true
            
            ii = ii + 1;
            
            % Remove all points that do not have a 'near' match
            [temp_scan, rej_map] = killoutliers(map, raw, rejection_setting);
            [temp_map, rej_raw] = killoutliers(temp_scan, map, rejection_setting);
            
            %calculate union of rejections, may be useful if your
            %calcutating rejection for a scan-to-map, but not really for a
            %scan-to-scan
            %rej_union = rej_map * rej_raw;
            
            % Execute ICP
            [TR, TT] = call_icp1(temp_map, temp_scan);
            
            % Transform current scan and pose estimate to new location.
            TR = TR(1:2,1:2); % Cheap way to only select z rotation
            TT = repmat(TT(1:2), 1, size(raw,2));
            
            theta = acos(TR(1));
            
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
                    cur = ([disp(:,1);theta]);
                    pose = [pose cur];
                    pose_dot = pose(:,end) - pose(:,end-1);
                    break;
                end
            end
        end
        
        if rej_raw > 0.6
            warning('you done fucked up');
        end
        
        %nScan
        %fprintf('rej_map = %.4f, rej_raw = %.4f\n\n', rej_map, rej_raw);
        
        world = [world raw];
        scanSizes = [scanSizes size(world,2)];

        last_node = node_cur;
        
        node_x = floor(pose(1,end)/graph.width) + 25;
        node_y = floor(pose(2,end)/graph.width) + 25;
        node_cur = [node_x, node_y];
        
        tmp = area(node_cur(1), node_cur(2));
        tmp_last = area(last_node(1), last_node(2));

        if ~isequal(last_node, node_cur)
            fprintf('entering node [%d,%d]\n', node_cur(1), node_cur(2));
            area(last_node(1), last_node(2)).visited = 1;
        end
                
        if isempty(tmp.data)
            map = raw;
            fprintf('found empty node!\n');
        else
            map = tmp.data;
        end
        
        if isempty(tmp.visited)
            fprintf('adding data\n');
            area(node_cur(1),node_cur(2)).data = [tmp.data raw];
        end
    end
    
    plot_scans
    %pause so we can display things
    pause(0.1);
    
    frame = frame + 1;
end

%plot_scans

profile viewer
profile off