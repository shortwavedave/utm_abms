function lbsd = LEM_gen_Delaunay_roads(xmin,xmax,ymin,ymax,num_vertexes,...
    min_dist, min_rb_dist)
% LEM_gen_Delaunay_roads - generate roads using Delaunay triangles
% On input:
%     xmin (float): min x coord
%     xmax (float): max x coord
%     ymin (float): min y coord
%     ymax (float): max y coord
%     num_vertexes (int): number of lane vertexes
%     min_dist (float): minimal distance between vertexes
% On output:
%     roads (road struct): road info
%       .vertexes (nx3 array): x,y,z coords of endpoints
%       .edges (mx2 array): indexes of vertexes defining lanes
% Call:
%     roads1 = LEM_gen_Delaunay_roads(50,200);
% Author:
%    T. Henderson
%    UU
%    Fall 2020
%

MAX_ITER = 1000;

x = xmin + (xmax-xmin)*rand;
y = ymin + (ymax-ymin)*rand;
pts = [x,y];
done = 0;
count = 0;
% % wb = waitbar(0,'Delaunay');
while done==0
    xn = xmin + (xmax-xmin)*rand;
    yn = ymin + (ymax-ymin)*rand;
    pts = [pts; xn,yn];
    mm = min(pdist(pts));
    if mm<min_dist
        pts = pts(1:end-1,:);
        count= count+1;
%         waitbar(count/MAX_ITER);
    else
        count = 0;
    end
%     length(pts(:,1)); % debug
    if length(pts(:,1))==num_vertexes || count>MAX_ITER
        done = 1;
    end
end
% close(wb);
num_vertexes = length(pts(:,1));
vertexes = [pts,zeros(num_vertexes,1)];
x = pts(:,1);
y = pts(:,2);
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

roads.vertexes = vertexes;
roads.edges = edges;

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
min_lane_len = min_rb_dist;
altitude1 = 142;
altitude2 = 162;
launch_sites = 1:num_vertexes;
land_sites = 1:num_vertexes;

airways = lbsd.LEM_gen_airways(roads,launch_sites,land_sites,...
    min_lane_len,altitude1,altitude2);

lane_graph = LBSD.airways2lanegraph(airways);

lbsd.lane_graph = lane_graph;

tch = 0;
