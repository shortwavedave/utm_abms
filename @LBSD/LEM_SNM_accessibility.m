function [acc,avg_acc] = LEM_SNM_accessibility(obj)
% LEM_SNM_accesibility - Spatial Network Measure: accessibility
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     acc (nx1 vector): average accessibility of each vertex
%     avg_acc (float): average accessibility of all vertexes
% Call:
%     [a,aa] = LEM_SNM_accessibility(G);
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

V = [G.Nodes.XData,G.Nodes.YData];
%V = table2array(G.Nodes);
%V = V(:,1:2);
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
%E = table2array(G.Edges);
[num_E,dummy] = size(E);
f = LBSD.LEM_SNM_route_factor(obj);
acc = zeros(num_V,1);
for v = 1:num_V
    acc(v) = sum(f(v,:))/(num_V-1);
end
total = 0;
for v1 = 1:num_V
    for v2 = 1:num_V
        if v1~=v2
            total = total + f(v1,v2);
        end
    end
end
avg_acc = total/(num_V*(num_V-1));
