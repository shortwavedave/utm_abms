function lbsd = LEM_gen_gis_roads(roads, min_dist, num_nodes)
% LEM_gen_grid_roads - generate roads using grid layout
% On input:
%     xmin (float): min x coord
%     xmax (float): max x coord
%     ymin (float): min y coord
%     ymax (float): max y coord
%     dx (float): dx space between vertexes
%     dy (float): dy space between vertexes
%     min_dist(float): The minimum distance between nodes in roundabouts
% On output:
%     roads (road struct): road info
%       .vertexes (nx3 array): x,y,z coords of endpoints
%       .edges (mx2 array): indexes of vertexes defining lanes
% Call:
%     roadsg = LEM_gen_grid_roads(-20,20,-20,20,5,5);
% Author:
%    T. Henderson
%    UU
%    Fall 2020
%


num_vertexes = size(roads.vertexes,1);
sz_nodes = num_vertexes;
vertexes = roads.vertexes;
edges = roads.edges;
wx = max(vertexes(:,1)) - min(vertexes(:,1));
wy = max(vertexes(:,2)) - min(vertexes(:,2));
mx = wx/2 + min(vertexes(:,1));
my = wy/2 + min(vertexes(:,2));
d = 0.5;


% Generate a node table with the roundabout vertexes
node_table = table(vertexes(:,1), vertexes(:,2), vertexes(:,3), ...,
    zeros(num_vertexes, 1), ...
    zeros(num_vertexes, 1), ...
    'VariableNames', ...
    {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
    'RowNames', string(1:num_vertexes) );

% Generate an edge table for the roundabout vertexes
num_edges = size(edges,1);
edge_lengths = zeros(num_edges, 1);
for i = 1:num_edges
    v1 = edges(i,1);
    v2 = edges(i,2);
    edge_lengths(i) = norm(vertexes(v1,:)-vertexes(v2,:));
end

edge_table = table(edges, ...
    edge_lengths, 'VariableNames', ...
    {'EndNodes','Weight'},'RowNames', string(1:size(edges, 1)));

node_table.Name = node_table.Properties.RowNames;

G = graph(edge_table, node_table);
g = G;

while sz_nodes > num_nodes && d > 0
    xcond = vertexes(:,1) >= (mx - d*wx) & vertexes(:,1) <= (mx + d*wx);
    ycond = vertexes(:,2) >= (my - d*wy) & vertexes(:,2) <= (my + d*wy);
    inds = find(xcond & ycond);
%     vertexes_t = vertexes( inds, : );
%     edges_t = edges(ismember(edges(:,1), inds) & ismember(edges(:,2), inds),:);
    
    g = subgraph(G, inds);
    [bin,binsize] = conncomp(g);
    sz_nodes = max(binsize);
    idx = binsize(bin) == sz_nodes;
    g = subgraph(g, idx);
    
    d = d - 0.0001;
end

% Rename all the verts
new_roads.vertexes = g.Nodes{:,{'XData', 'YData', 'ZData'}};
new_roads.edges = str2double([g.Edges.EndNodes]);
for edge_i = 1:size(new_roads.edges, 1)
    v1 = new_roads.edges(edge_i,1);
    v2 = new_roads.edges(edge_i,2);
    v1_i = find(g.Nodes.Properties.RowNames == string(v1));
    v2_i = find(g.Nodes.Properties.RowNames == string(v2));
    new_roads.edges(edge_i,1) = v1_i;
    new_roads.edges(edge_i,2) = v2_i;
end

num_vertexes = size(new_roads.vertexes,1);
node_table = table(new_roads.vertexes(:,1), new_roads.vertexes(:,2), new_roads.vertexes(:,3), ...,
    zeros(num_vertexes, 1), ...
    zeros(num_vertexes, 1), ...
    'VariableNames', ...
    {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
    'RowNames', string(1:num_vertexes) );

edge_lengths = g.Edges.Weight;
num_edges = length(edge_lengths);
edge_table = table(new_roads.edges, ...
    edge_lengths, 'VariableNames', ...
    {'EndNodes','Weight'},'RowNames', string(1:num_edges));
g = graph(edge_table, node_table);

% Instantiate an LBSD object
lbsd = LBSD();

% Initialize the LBSD object with the generated lane graph
lbsd.road_graph = g;

% Create structure for generating airways
% min_lane_len = 3;
min_lane_len = min_dist;
altitude1 = 142;
altitude2 = 162;
launch_sites = 1:num_vertexes;
land_sites = 1:num_vertexes;


airways = lbsd.LEM_gen_airways(new_roads,launch_sites,land_sites,...
    min_lane_len,altitude1,altitude2);

lane_graph = LBSD.airways2lanegraph(airways);

lbsd.lane_graph = lane_graph;

tch = 0;
