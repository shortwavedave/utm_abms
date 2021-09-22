function degrees = LEM_SNM_degree(obj)
% LEM_SNM_degree - Spatial Network Measure: node degree
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     degrees (nx1 vector): degree of each vertex
% Call:
%     deg = LEM_SNM_degree(G);
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
A = adjacency(G);
%A = LEM_SNM_adjacency_matrix(G);

degrees = zeros(num_V,1);
for v = 1:num_V
    degrees(v) = sum(A(v,:));
end
