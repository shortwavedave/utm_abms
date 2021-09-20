function betweenness_centrality(obj)
%

p = plot(obj);
W = obj.lane_graph.Edges.Weight;
ucc = centrality(obj.lane_graph,'betweenness','Cost',W);
p.NodeCData = ucc;
colormap jet
colorbar
