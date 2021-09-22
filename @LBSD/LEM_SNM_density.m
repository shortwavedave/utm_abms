function density = LEM_SNM_density(obj)
% LEM_SNM_density - Spatial Network Measure: edge density
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in plane
%       .Edges (mx3 array): vertex indexes for edges; edge weights
% On output:
%     density (float): edge to vertex ratio
% Call:
%     d = LEM_SNM_density(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    degrees = [];
    return
end

V = [G.Nodes.XData,G.Nodes.YData];
%V = table2array(G.Nodes);
%V = V(:,1:2);
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
%E = table2array(G.Edges);
[num_E,dummy] = size(E);

density = num_E/num_V;
