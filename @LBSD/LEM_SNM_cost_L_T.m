function [cost_L_T,cost_L_MST,cost] = LEM_SNM_cost_L_T(obj)
% LEM_SNM_cost_L_T - Spatial Network Measure: sum of all edge lengths
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     cost_L_T (float): total sum of edge lengths
%     cost_L_MST (float): total cost of MST
%     cost (float): ratio of cost_L_T and cost_L_MST
% Call:
%     [c_L_T,c_L_MST,c] = LEM_SNM_cost_L_T(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
cost_L_T = 0;
cost_L_MST = 0;
cost = 0;

if isempty(G)
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

cost_L_T = sum(E(:,3));

MST = minspantree(G);
E_MST = table2array(MST.Edges);
cost_L_MST = sum(E_MST(:,3));

cost = cost_L_T/cost_L_MST;

tch = 0;
