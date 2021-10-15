function min_path = LEM_SNM_min_path_dist(obj,G)
% LEM_SNM_min_path_dist - Spatial Network Measure: min dist tween nodes
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     min_path (mx1 vector): min path dist between node pairs (l(i,j))
% Call:
%     mps = LEM_SNM_min_path_dist(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

BIG_NUM = 1000000;

%G = obj.lane_graph;
if isempty(G)
    min_path = [];
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

W = G.Edges.Weight;
%W = E(:,3);

Gd = G;
%Gd = obj.lane_graph;
%Gd = LEM_SNM_G2digraph(G);
min_path = zeros(num_V,num_V);
wb = waitbar(0,'min path');
for v1 = 1:num_V
    waitbar(v1/num_V);
    [v1,num_V];
    for v2 = 1:num_V
        if v1~=v2
            path = shortestpath(Gd,v1,v2);
            if isempty(path)
                min_path(v1,v2) = BIG_NUM;
            else
                min_path(v1,v2) = sum(W(path));
            end
        end
    end
end
close(wb);
