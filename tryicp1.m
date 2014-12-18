% This is the first attempt at writing an icp based slam algorithm using icp_1
% Author: Josh M
%
%clear
clear map world pose graphStruct
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
rejection_setting = 0.1;

scanSizes = [];

theta_offset = deg2rad(0.5);

if ~scanToScan
    warning('Running scan-to-map, this may take a while.')
end

graph.n = 150;
graph.m = 150;
graph.width = 0.5; %meters

empty_mat = zeros(graph.n, graph.m);
%graph.nodelist(graph.n,graph.m) = struct('data', nan(graph.n,graph.m), 'visited', empty_mat, 'n', empty_mat);
graph.nodelist = repmat(struct('data', [], 'visited', 0, 'n', 0), graph.n, graph.m);

ndx = 1;
ndy = 1;

node_idx_last = [];

frame = 1;
frame_skip = 20;
for nScan = 800:frame_skip:size(nScanIndex)
    
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
        
        [cur, pose, pose_dot, raw, disp] = manelaICP(map, raw, pose, pose_dot, ...
            disp, rejection_setting, converge_metric);
        
        world = [world raw];
        scanSizes = [scanSizes size(world,2)];

        node_idx_last = [ndx ndy]';
        
        % TODO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % give more space to the graph when it needs it.
        % TODO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        ndx = mod(floor(pose(1,end)/graph.width + graph.width/2), graph.n) + 1;
        ndy = mod(floor(pose(2,end)/graph.width + graph.width/2), graph.m) + 1;
        
        node_idx_cur = [ndx ndy]';
        
        node_cur = graph.nodelist(ndx,ndy);
        node_last = graph.nodelist(node_idx_last(1), node_idx_last(2));

        if ~isequal(node_idx_last, node_idx_cur)
            %filter out our last node stuff
            graph.nodelist(node_idx_last(1), node_idx_last(2)).data = node_last.data(:,1:node_last.n:end);
            graph.nodelist(node_idx_last(1), node_idx_last(2)).visited = 1;
            
            fprintf('leaving old node\n\nentering node [%d,%d]\n', ndx, ndy);
        end
                
        if isempty(node_cur.data)
            fprintf('found empty node!\n');
            %when you find an empty node, use the last scan
            map = raw;

            % TODO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %when you find an empty node, search for a node around it that
            %has already been visited (and that's not the last node).  If
            %it can't find a node which isn't the last, just use raw
            % TODO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %{
            surround = [[0 1]' [-1 0]' [1 0]' [0 -1]'];
            
            for ii=1:4
                tmp = surround(:,ii) + node_idx_cur;
                tmp = graphStruct(tmp(1), tmp(2));
                if ~isequal(tmp,node_last) && ~isempty(tmp.visited)
                    map = tmp.data;
                    fprintf('looking at nearby node!\n');
                    break;
                end
            end
            %}
        else
            map = node_cur.data;
        end
        
        if ~node_cur.visited
            fprintf('adding data\n');
            graph.nodelist(ndx,ndy).data = [node_cur.data raw];
            graph.nodelist(ndx,ndy).n = graph.nodelist(ndx,ndy).n+1;
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