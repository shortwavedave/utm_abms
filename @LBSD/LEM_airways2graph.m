function G = LEM_airways2graph(obj,airways)
% LEM_airway2graph - produce digraph from airways lanes
% On input:
%     airways (airways struct): airways info
% On output:
%    G (digraph): directed graph of 3D lane system
% Call:
%    G = LEM_airways2graph(airway);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

vertexes = airways.lane_vertexes;
edges = airways.lane_edges;
num_edges = length(edges(:,1));

indexes1 = edges(:,1);
indexes2 = edges(:,2);
weights = zeros(num_edges,1);
for e = 1:num_edges
    weights(e) = norm(vertexes(indexes1(e),:)-vertexes(indexes2(e),:));
end
G = digraph(indexes1,indexes2,weights);
G.Nodes = array2table(vertexes);
