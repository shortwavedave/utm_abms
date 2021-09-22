function e = LEM_SNM_efficiency(obj)
% LEM_SNM_efficiency - Spatial Network Measure: efficiency
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     e (float): effciency: sum(1/L(i,j)/(N(N-1))   
% Call:
%     e = LEM_SNM_efficency(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    e = 0;
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

costs = LEM_SNM_min_path_step(G);
total = 0;
for v1 = 1:num_V
    for v2 = 1:num_V
        if v1~=v2
            if costs(v1,v2)>0
                total = total + 1/costs(v1,v2);
            end
        end
    end
end
e = total/(num_V*(num_V-1));
