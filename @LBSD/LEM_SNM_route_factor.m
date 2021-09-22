function r_factor = LEM_SNM_route_factor(obj)
% LEM_SNM_route_factor - Spatial Network Measure: route factor
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     r_factor (float): route factor
% Call:
%     f = LEM_SNM_route_factor(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    r_factor = 0;
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
ds = LBSD.LEM_SNM_min_path_step(obj);
dd = LBSD.LEM_SNM_min_path_dist(obj);
r_factor = zeros(num_V,num_V);
for v1 = 1:num_V
    for v2 = 1:num_V
        if v1~=v2
            r_factor(v1,v2) = ds(v1,v2)/dd(v1,v2);
            tch = 0;
        end
    end
end
