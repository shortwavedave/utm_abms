function A = LEM_SNM_adjacency_matrix(obj)
% LEM_SNM_adjacency_matrix - Spatial Network Measure: adjacency matrix
% On input:
%     G (graph struct): undirected graph info
%       .V (nx2 array): vertex locations in plane
%       .E (mx2 array): vertex indexes for edges
% On output:
%     A (nxn array): 1 if edge beween
% Call:
%     A = LEM_SNM_adjacency_matrix(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    A = [];
    return
end

V = [G.Nodes.XData,G.Nodes.YData];
%V = G.V;
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
%E = G.E;
[num_E,dummy] = size(E);
A = zeros(num_V,num_V);
for e = 1:num_E
    A(E(e,1),E(e,2)) = 1;
    A(E(e,2),E(e,1)) = 1;
end
