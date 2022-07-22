function [lbsd, err] = LEM_gen_gis_roads(roads, min_dist, num_nodes, map_sz)
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
err = inf;
num_vertexes = size(roads.vertexes,1);
sz_nodes = num_vertexes;
vertexes = roads.vertexes;
edges = roads.edges;
wx = max(vertexes(:,1)) - min(vertexes(:,1));
wy = max(vertexes(:,2)) - min(vertexes(:,2));
mx = wx/2 + min(vertexes(:,1));
my = wy/2 + min(vertexes(:,2));

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

bad_l = edge_lengths < ;


edge_table = table(edges, ...
    edge_lengths, 'VariableNames', ...
    {'EndNodes','Weight'},'RowNames', string(1:size(edges, 1)));

node_table.Name = node_table.Properties.RowNames;

G = graph(edge_table, node_table);
g = G;

if nargin < 4
    d = 0.5;
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
else
    minx = min(roads.vertexes(:,1));
    maxx = max(roads.vertexes(:,1));
    miny = min(roads.vertexes(:,2));
    maxy = max(roads.vertexes(:,2));
    w = maxx-minx;
    h = maxy-miny;
    m_sz = [min(maxx-minx,map_sz), min(maxy-miny,map_sz)];
    goal = m_sz;
    s_w = w - goal(1);
    s_h = h - goal(2);
    search_bbox = [minx,miny,minx+m_sz(1),miny+m_sz(2)];
    xy_mnmx = search_bbox;
    
    max_iter = 10000;
    done = false;
    best.xy_mnmx = xy_mnmx;
    best.inds = 1:size(vertexes,1);
    best.err = inf;
    best.g = [];
    tol = 5;
    i = 0;
    h = waitbar(i/max_iter, num2str(i));
    while ~done
        vertexes = roads.vertexes;
        xcond = vertexes(:,1) >= xy_mnmx(1) & vertexes(:,1) <= xy_mnmx(3);
        ycond = vertexes(:,2) >= xy_mnmx(2) & vertexes(:,2) <= xy_mnmx(4);
        
        inds = find(xcond & ycond);
%         if ~isempty(inds) && length(inds) > num_nodes
%             inds = inds(randperm(length(inds),num_nodes));
%         end
    %     vertexes_t = vertexes( inds, : );
    %     edges_t = edges(ismember(edges(:,1), inds) & ismember(edges(:,2), inds),:);
        if ~isempty(inds)
            g = subgraph(G, inds);
            [bin,binsize] = conncomp(g);
            sz_nodes = max(binsize);
            idx = binsize(bin) == sz_nodes;
            
            g = subgraph(g, idx);
            [bin, binsize] = conncomp(g);
            sz_nodes = max(binsize);
            idx = binsize(bin) == sz_nodes;
            idx = find(idx);
            if length(idx) > num_nodes*5
                idx = idx(randperm(length(idx),num_nodes*5));
                g = subgraph(g, idx);
                [bin,binsize] = conncomp(g);
                sz_nodes = max(binsize);
                idx = binsize(bin) == sz_nodes;
                g = subgraph(g, idx);
            end
            
            % Evaluate the size of the largest component
            vertexes = g.Nodes{:,{'XData', 'YData', 'ZData'}};
            minx = min(vertexes(:,1));
            maxx = max(vertexes(:,1));
            miny = min(vertexes(:,2));
            maxy = max(vertexes(:,2));
            m_sz = [min(maxx-minx,map_sz), min(maxy-miny,map_sz)];
            err_p = [abs(goal - m_sz), abs(num_nodes - length(find(idx)))];
            err = dot(err_p,err_p);
            if err < best.err
                best.err = err;
                best.xy_mnmx = xy_mnmx;
                best.inds = inds;
                best.g = g;
            end
        else
            err = inf;
        end

        if err <= tol || i > max_iter
            done = true;
        else
            x0 = s_w*rand();
            y0 = s_h*rand();
            xy_mnmx = search_bbox + [x0, y0, x0, y0];
        end
        i = i + 1;
        waitbar(i/max_iter,h,sprintf("%s:%s",num2str(i), num2str(best.err)));
    end
    close(h);
    err = best.err;
    g = best.g;
%     xy_mnmx = best.xy_mnmx;
%     vertexes = roads.vertexes;
%     xcond = vertexes(:,1) >= xy_mnmx(1) & vertexes(:,1) <= xy_mnmx(3);
%     ycond = vertexes(:,2) >= xy_mnmx(2) & vertexes(:,2) <= xy_mnmx(4);
%     inds = find(xcond & ycond);
%     g = subgraph(G, best.inds);
%     [bin,binsize] = conncomp(g);
%     sz_nodes = max(binsize);
%     idx = binsize(bin) == sz_nodes;
%     g = subgraph(g, idx);
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
