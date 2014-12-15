function [ Map, Trajectory ] = MakeMap( PoseGraph, LidarScans, LidarAngles, LidarRange )
%MakeMap - Generate a map from the lidar scans and pose graph
%   INPUTS:
%     PoseGraph   - List of relations between scans
%     LidarScans  - Matrix of lidar scan measurements
%     LidarAngles - list of angles for each lidar measurement
%     LidarRange  - Max sensor distance
%
%   OUTPUTS:
%     Map - List of laser hits in common map frame
%     Trajectory - List of scan locations in common map frame


    % Generate the trajectory from the pose graph
    Trajectory = PoseGraph(:,1);
    for i = 2:size(PoseGraph,2)
        p  = Trajectory([1, 2], i-1);
        dp = rotate2d(-Trajectory(3, i-1), PoseGraph([1,2],i));

        Trajectory([1, 2], i) = p + dp;
        Trajectory([3, 4], i) = Trajectory([3, 4], i-1) + PoseGraph([3, 4],i);
    end


    % Generate the map
    Map  = [];
    for nScan = 1:size(Trajectory,2)
        scan = LidarScans(nScan,:);
        a    = LidarAngles;

        % Remove out of range measurements
        I = (scan >= LidarRange);
        scan(I) = [];
        a(I)    = [];

        % Rotation
        a = a + Trajectory(3, nScan);
        [x, y] = pol2cart(a, scan);

        % Translation
        x = x + Trajectory(1, nScan);
        y = y + Trajectory(2, nScan);
        d = [x; y];

        % Add points to map.
        Map = cat(2, Map, d);
    end

end

