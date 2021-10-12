function bc = LEM_SNM_betweeness_centrality_node(obj,use_roads)
% LEM_SNM_betweeness_centrality_node - Spatial Network Measure: 
%                       betweeness centrality per node
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
%     use_roads (Boolean): use road graph
% On output:
%     bc (nx1 vector): betweeness centrality of each vertex
% Call:
%     bc = LEM_SNM_betweeness_centality_node(G,1);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if ~use_roads
    G = obj.lane_graph;
    if isempty(G)
        bc = [];
        return
    end
    W = obj.lane_graph.Edges.Weight;
    bc = centrality(obj.lane_graph,'betweenness','Cost',W);
else
    G = obj.road_graph;
    if isempty(G)
        bc = [];
        return
    end
    W = obj.road_graph.Edges.Weight;
    bc = centrality(obj.road_graph,'betweenness','Cost',W);
end
