function cc = LEM_SNM_closeness_centrality(obj,use_roads)
% LEM_SNM_closeness_centrality - Spatial Network Measure: closeness
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
%     use_roads (Boolean): if 1 use roads graph, else lane graph
% On output:
%     acc (nx1 vector): average accessibility of each vertex
%     avg_acc (float): average accessibility of all vertexes
% Call:
%     cc = LEM_SNM_closeness_centrality(G,0);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if ~use_roads
    G = graph(obj.lane_graph.Edges.EndNodes(:,1),...
        obj.lane_graph.Edges.EndNodes(:,2),...
        obj.lane_graph.Edges.Weight);
    W = obj.lane_graph.Edges.Weight+eps;
    if isempty(G)
        cc = [];
        return
    end
else
    G = obj.road_graph;
    W = obj.road_graph.Edges.Weight+eps;
    if isempty(G)
        cc = [];
        return
    end
end

cc = centrality(G,'closeness','Cost',W);
