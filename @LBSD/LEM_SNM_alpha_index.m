function alpha_index = LEM_SNM_alpha_index(obj)
% LEM_SNM_alpha_index - Spatial Network Measure: alpha index
%    0 for tree; 1 for clique
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     alpha_index (float): alpha index
% Call:
%     a = LEM_SNM_alpha_index(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    alpha_index = 0;
    return
end

V = [G.Nodes.XData,G.Nodes.YData];
%V = table2array(G.Nodes);
%V = V(:,1:2);
[num_V,dummy] = size(V);
c = LEM_SNM_cyclomatic_num(obj);
alpha_index = c/(2*num_V-5);
