function coefs = LEM_SNM_clustering_coefs(obj)
% LEM_SNM_clustering_coefs - Spatial Network Measure: clustering coefs
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     coefs (nx1 vector): clustering coefficient for each node
% Call:
%     coefs = LEM_SNM_clustering_coef(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    coefs = [];
    return
end

% number of edges among neighbors
degrees = LEM_SNM_degree(G);
V = [G.Nodes.XData,G.Nodes.YData];
%V = table2array(G.Nodes);
%V = V(:,1:2);
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
%E = table2array(G.Edges);
[num_E,dummy] = size(E);
for v = 1:num_V
    nei = find(E(v,:));
    E_v = sum(degrees(nei));
    coefs(v) = E_v/(degrees(v)*(degrees(v)-1)/2);
end
