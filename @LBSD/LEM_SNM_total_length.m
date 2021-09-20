function total_length = LEM_SNM_total_length(obj)
% LEM_SNM_total_length - Spatial Network Measure: total length of G
% On input:
%     G (Matlab undirected graph): graph info
%      .Nodes (nx3 table): vertex locations
%      .Edges (mx3 table): vertex indexes for edges; weight
% On output:
%     total length (float): total length
% Call:
%     tl = LEM_SNM_total_length(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    total_length = 0;
    return
end

V = [G.Nodes.XData,G.Nodes.YData];
%V = table2array(G.Nodes);
%V = V(:,1:2);
[num_V,dummy] = size(V);
[num_V,dummy] = size(V);
total_length = 1.51*num_V^0.49;
