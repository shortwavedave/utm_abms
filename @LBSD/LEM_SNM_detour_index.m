function detour_index = LEM_SNM_detour_index(obj)
% LEM_SNM_degree - Spatial Network Measure: detour index:
%                 node pair straight line dist over path dist
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     detour_index (mx1 vector): detour index of each edge
% Call:
%     d = LEM_SNM_detour_index(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    detour_index = [];
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
detour_index = ones(num_V,num_V);

v_dists = zeros(num_V,num_V);
for v1 = 1:num_V-1
    pt1 = V(v1,1:2);
    for v2 = v1+1:num_V
        pt2 = V(v2,1:2);
        v_dists(v1,v2) = norm(pt1-pt2);
        v_dists(v2,v1) = v_dists(v1,v2);
    end
end

for v1 = 1:num_V-1
    for v2 = v1+1:num_V
        p = shortestpath(G,v1,v2);
        dist = v_dists(v1,v2);
        p_dist = 0;
        for k = 1:length(p)-1
            p_dist = p_dist + v_dists(p(k),p(k+1));
        end
        detour_index(v1,v2) = dist/p_dist;
        detour_index(v2,v1) = detour_index(v1,v2);
    end
end
