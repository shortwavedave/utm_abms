function sc = LEM_SNM_straightness_centrality(obj)
% LEM_SNM_straightness_centrality - Spatial Network Measure: straightness
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     sc (nx1 vector): straightness centrality of each vertex
% Call:
%     sc = LEM_SNM_straightness_centrality(G);
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

ds = LBSD.LEM_SNM_min_path_step(obj);
dd = LBSD.LEM_SNM_min_path_dist(obj);

V = [G.Nodes.XData,G.Nodes.YData];
[N,~] = size(V);
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
[num_E,dummy] = size(E);
sc = zeros(num_V,1);
for v = 1:num_V
    for v2 = 1:num_V
        if ds(v,v2)==0
            sc(v) = 0;
        else
            sc(v) = sc(v) + (1/(N-1))/(dd(v,v2)/ds(v,v2));
        end
    end
end
