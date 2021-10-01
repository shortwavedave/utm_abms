function cc = LEM_SNM_closeness_centrality(obj)
% LEM_SNM_closeness_centrality - Spatial Network Measure: closeness
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     acc (nx1 vector): average accessibility of each vertex
%     avg_acc (float): average accessibility of all vertexes
% Call:
%     cc = LEM_SNM_closeness_centrality(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    acc = [];
    avg_acc = 0;
    return
end

d = LBSD.LEM_SNM_min_path_step(obj);

V = [G.Nodes.XData,G.Nodes.YData];
[N,~] = size(V);
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
[num_E,dummy] = size(E);
cc = zeros(num_V,1);
for v = 1:num_V
    cc(v) = (N-1)/sum(d(v,:));
end
