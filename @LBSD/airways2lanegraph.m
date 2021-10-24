function lane_graph = airways2lanegraph(airways)
%AIRWAYS2LANEGRAPH convert tch airways to LBSD lane_graph

% Generate a node table with the roundabout vertexes
xs = airways.lane_vertexes(:,1);
ys = airways.lane_vertexes(:,2);
zs = airways.lane_vertexes(:,3);
num_verts = length(xs);
launch_verts = zeros(num_verts,1);
land_verts = zeros(num_verts,1);
launch_verts(airways.launch_lane_vertexes) = 1;
land_verts(airways.land_lane_vertexes) = 1;

node_table = table(xs, ys, zs, ...,
    launch_verts, ...
    land_verts, ...
    'VariableNames', ...
    {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
    'RowNames', string(1:num_verts) );
node_table.Name = node_table.Properties.RowNames;

% Generate an edge table for the roundabout vertexes
num_lanes = size(airways.lane_edges,1);
edge_table = table(airways.lane_edges, ...
    airways.lane_lengths, 'VariableNames', ...
    {'EndNodes','Weight'},'RowNames', string(1:num_lanes));

lane_graph = digraph(edge_table,node_table);
    
end

