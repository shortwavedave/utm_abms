function diameter = LEM_SNM_graph_diameter(obj)
% LEM_SNM_graph_diameter - Spatial Network Measure: graph diameter
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     diameter (float): largest distance (in steps) across graph
% Call:
%     diam = LEM_SNM_graph_diamter(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    diameter = 0;
    return
end

min_path = LEM_SNM_min_path_step(G);
diameter = max(max(min_path));
