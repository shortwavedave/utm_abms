function gamma_index = LEM_SNM_gamma_index(obj)
% LEM_SNM_gamma_index - Spatial Network Measure: gamma index
%   ratio of num edges over max num edges
% On input:
%     G (graph struct): undirected graph info
%       .V (nx2 array): vertex locations in plane
%       .E (mx2 array): vertex indexes for edges
% On output:
%     gamma_index (float): gamma index
% Call:
%     gamma = LEM_SNM_gamma_index(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

G = obj.lane_graph;
if isempty(G)
    gamma_index = 0;
    return
end

V = [G.Nodes.XData,G.Nodes.YData];
%V = G.V;
[num_V,dummy] = size(V);
E = [str2num(str2mat(G.Edges.EndNodes(:,1))),...
    str2num(str2mat(G.Edges.EndNodes(:,2)))];
%E = G.E;
[num_E,dummy] = size(E);

gamma_index = num_E/(3*num_V-6);
