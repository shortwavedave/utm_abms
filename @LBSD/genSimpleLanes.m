function lbsd = genSimpleLanes(lane_lengths_m)
    % genSimpleLanes Create a simple lane system
    %   The generated lane system contains a sequence of lanes. The weights 
    %   of the graph represent the length of the lane.
    %   
    %   On Input:
    %       lane_lengths_m - 1xn (float) the lengths of every lane in the 
    %           system in meters. n determines how many lanes there are
    %   On Output:
    %       LBSD - an instance of the LBSD class initialized with
    %       the simple lane network.
    %   Call:
    %       lbsd = LBSD.genSimpleLanes([10, 15])
    
    num_lanes = length(lane_lengths_m);
    num_verts = num_lanes + 1;
    if num_verts < 2
        error("You must provide at least one lane length");
    end
    
    ys = zeros(1,num_verts);
    xs = [0];
    for i = 1:num_lanes
        xs = [xs lane_lengths_m(i)+xs(i)];
    end
    zs = ys;

    % Generate a node table with the roundabout vertexes
    node_table = table(xs', ys', zs', ...,
        [1; zeros(num_verts-1, 1)], ...
        [zeros(num_verts-1, 1); 1], ...
        'VariableNames', ...
        {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
        'RowNames', string(1:num_verts) );

    % Generate an edge table for the roundabout vertexes
    edge_table = table([(1:num_verts-1)', (2:num_verts)'], ...
        lane_lengths_m', 'VariableNames', ...
        {'EndNodes','Weight'},'RowNames', string(1:num_lanes));

    node_table.Name = node_table.Properties.RowNames;
    % Instantiate an LBSD object
    lbsd = LBSD();

    % Initialize the LBSD object with the generated lane graph
    lbsd.lane_graph = digraph(edge_table, node_table);
end