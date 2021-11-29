function lbsd = LEM_gen_grid_roads_del(xmin,xmax,ymin,ymax,dx,dy,min_dist)
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

x_vals = [xmin:dx:xmax]';
y_vals = [ymin:dy:ymax]';
num_x_vals = length(x_vals);
num_y_vals = length(y_vals);
num_vertexes = num_x_vals*num_y_vals;
vertexes = zeros(num_vertexes,3);
count = 0;
for ind1 = 1:num_x_vals
    x = x_vals(ind1);
    for ind2 = 1:num_y_vals
        count = count + 1;
        y = y_vals(ind2);
        vertexes(count,1:2) = [x,y];
    end
end

x = vertexes(:,1);
y = vertexes(:,2);
dedges = delaunay(x,y);
num_dedges = length(dedges(:,1));
nei = zeros(num_vertexes,num_vertexes);
for e = 1:num_dedges
    index1 = dedges(e,1);
    index2 = dedges(e,2);
    index3 = dedges(e,3);
    nei(index1,index2) = 1;
    nei(index1,index3) = 1;
    nei(index2,index3) = 1;
    nei(index2,index1) = 1;
    nei(index3,index1) = 1;
    nei(index3,index2) = 1;
end
edges = [];
edge_lengths = [];
for n1 = 1:num_vertexes-1
    pt1 = vertexes(n1,:);
    for n2 = n1+1:num_vertexes
       pt2 = vertexes(n2,:);
        if nei(n1,n2)==1
            edges = [edges;n1,n2];
            edge_lengths = [edge_lengths;norm(pt1-pt2)]; 
        end
    end
end
num_edges = length(edges(:,1));

% edges = [];
% edge_lengths =[];
% for ind1 = 1:num_vertexes-1
%     pt1 = vertexes(ind1,:);
%     for ind2 = ind1+1:num_vertexes
%         pt2 = vertexes(ind2,:);
%         if norm(pt2-pt1)<1.1*dx|norm(pt2-pt1)<1.1*dy
%             edges = [edges;ind1,ind2];
%             edge_lengths = [edge_lengths;norm(pt1-pt2)]; 
%         end
%     end
% end

% Generate a node table with the roundabout vertexes
node_table = table(vertexes(:,1), vertexes(:,2), vertexes(:,3), ...,
    zeros(num_vertexes, 1), ...
    zeros(num_vertexes, 1), ...
    'VariableNames', ...
    {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
    'RowNames', string(1:num_vertexes) );

% Generate an edge table for the roundabout vertexes
edge_table = table(edges, ...
    edge_lengths, 'VariableNames', ...
    {'EndNodes','Weight'},'RowNames', string(1:size(edges,1)));

node_table.Name = node_table.Properties.RowNames;
% Instantiate an LBSD object
lbsd = LBSD();

% Initialize the LBSD object with the generated lane graph
lbsd.road_graph = graph(edge_table, node_table);

% Create structure for generating airways
roads.vertexes = vertexes;
roads.edges = edges;

% min_lane_len = 3;
min_lane_len = min_dist;
altitude1 = 142;
altitude2 = 162;
launch_sites = 1:num_vertexes;
land_sites = 1:num_vertexes;

airways = lbsd.LEM_gen_airways(roads,launch_sites,land_sites,...
    min_lane_len,altitude1,altitude2);

lane_graph = LBSD.airways2lanegraph(airways);

lbsd.lane_graph = lane_graph;

tch = 0;
