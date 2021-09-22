function r_n = LEM_SNM_r_n(obj)
% LEM_SNM_r_n - Spatial Network Measure: close to 0: organized; else
%               organic
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     r_n (float): r_n
% Call:
%     r_n = LEM_SNM_r_n(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    r_n = 0;
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
degrees = LEM_SNM_degree(G);
indexes1 = find(degrees==1);
indexes3 = find(degrees==3);
indexes_not_2 = find(degrees~=2);
num = length(indexes1) + length(indexes3);
denom = length(indexes_not_2);
r_n = num/denom;
