function degrees = LEM_SNM_degree(obj,use_roads)
% LEM_SNM_degree - Spatial Network Measure: node degree
% On input:
%   use_roads: Boolean if true, calculate degree of road graph 
% On output:
%     degrees (nx1 vector): degree of each vertex
% Call:
%     deg = LEM_SNM_degree(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if ~use_roads
    G = obj.lane_graph;
else
    G = obj.road_graph;
end

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
A = adjacency(G);
%A = LEM_SNM_adjacency_matrix(G);

degrees = zeros(num_V,1);
for v = 1:num_V
    degrees(v) = sum(A(v,:));
end
