function sc = LEM_SNM_straightness_centrality(obj,use_roads)
% LEM_SNM_straightness_centrality - Spatial Network Measure: straightness
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
%     use_roads (Boolean): if 1 use roads graph, else lane graph
% On output:
%     sc (nx1 vector): straightness centrality of each vertex
% Call:
%     sc = LEM_SNM_straightness_centrality(G,1);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if ~use_roads
    G = obj.lane_graph;
    if isempty(G)
        sc = [];
        return
    end
else
    G = obj.road_graph;
    if isempty(G)
        sc = [];
        return
    end
end

dd = LEM_SNM_min_path_dist(obj,G);

V = [G.Nodes.XData,G.Nodes.YData,G.Nodes.ZData];
[N,~] = size(V);
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
[num_E,dummy] = size(E);
sc = zeros(num_V,1);
wb = waitbar(0,'Straightness Centrality');
for v = 1:num_V
    waitbar(v/num_V);
    temp = 0;
    for v2 = 1:num_V
        pt1 = V(v,:);
        pt2 = V(v2,:);
        dist = norm(pt2-pt1);
        if v~=v2&dd(v,v2)>0
                temp = temp + (1/(N-1))*dist/dd(v,v2);
            end
    end
    sc(v) = temp;
end
close(wb);
