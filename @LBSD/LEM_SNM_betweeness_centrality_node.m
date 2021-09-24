function bc = LEM_SNM_betweeness_centrality_node(obj)
% LEM_SNM_betweeness_centrality_node - Spatial Network Measure: 
%                       betweeness centrality per node
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     bc (nx1 vector): betweeness centrality of each vertex
% Call:
%     bc = LEM_SNM_betweeness_centality_node(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    bc = [];
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

bc = zeros(num_V,1);

G1.V = V;
G1.E = E(:,1:2);
G1.W = E(:,3);

tch = 0;
