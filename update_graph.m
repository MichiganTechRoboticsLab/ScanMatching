function [ graph, ndx, ndy, map ] = update_graph( graph, raw, ndx, ndy, pose)
%update_graph
%   When a robot explores an area which it has never seen before, it will
%   mark the previous spot as being seen and start saving scans for the
%   current spot.  If it is scans an area it is currently in, it will keep
%   the scan and save it to the spot.  If it explores an area it has
%   already seen, it will not save anything and just localize with it's
%   surruoudings.

%   This is the function which updates the graph and returns the correct
%   map for the icp system to use.  it also returns ndx and ndy represent
%   the current graph indicies the robot is processing.

%get the previous indicies from the graph
ndx_prev = ndx;
ndy_prev = ndy;
node_idx_prev = [ndx ndy]';

%calculate the current indicies for the graph
ndx = mod(floor(pose(1,end)/graph.width + graph.width/2), graph.n) + 1;
ndy = mod(floor(pose(2,end)/graph.width + graph.width/2), graph.m) + 1;
node_idx_cur = [ndx ndy]';

%grab temporary nodes
node_cur = graph.nodemat(ndx,ndy);
node_prev = graph.nodemat(ndx_prev, ndy_prev);

%update the graph and find the right map to compare the next scan
%with

%we have left our current node
if ~isequal(node_idx_cur, node_idx_prev)
    %mark the last node as visited and filter the data.  The data
    %is filterered by taking every n points where n is the number
    %of data sets the node contains.
    if ~node_prev.visited
        fprintf('\tmarking as visited\n', ndx_prev, ndy_prev);
        graph.nodemat(ndx_prev, ndy_prev).visited = 1;
        fprintf('\tfiltering data\n');
        graph.nodemat(ndx_prev, ndy_prev).data = node_prev.data(:,1:node_prev.n:end);
    end
    
    fprintf('\tleaving [%d,%d]\n\n', ndx_prev, ndy_prev);
    
    %if the current node hasn't been visited yet, use the last raw
    %scan as our map.  otherwise we know we're revisting and just
    %use the data from the current node which we already gathered
    if ~node_cur.visited
        fprintf('found node [%d,%d], using previous grid\n', ndx, ndy);
        %map = raw;
        map = node_prev.data;
    else
        fprintf('revisting [%d,%d]\n', ndx, ndy);
        map = node_cur.data;
    end
else
    map = node_cur.data;
end

%if we haven't visited the node yet, add data to the node.
if ~node_cur.visited
    fprintf('\twriting data\n');
    graph.nodemat(ndx,ndy).data = [node_cur.data raw];
    graph.nodemat(ndx,ndy).n = graph.nodemat(ndx,ndy).n+1;
    
    %set the map as our current known points
    map = graph.nodemat(ndx,ndy).data;
else
    %we're already using node_cur.data, so we don't have to set the
    %map.  we're meraly localizing the grid and throwing the scan
    %into the world for later
    fprintf('\tlocalizing\n');
end
end

