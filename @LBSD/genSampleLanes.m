function lbsd = genSampleLanes(lane_length_m, altitude_m)
    % genSampleLanes Create a sample lane system
    %   The generated lane system contains a single planar
    %   roundabout with 8 sides of length lane_length. This is a
    %   static function that instantiates an LBSD object (it is a
    %   factory for LBSD objects). The weights of the graph
    %   represent the length of the lane.
    %   On Input:
    %       lane_length_m - (float) the length of every lane in the 
    %           system in meters.
    %       altitude_m - (float) the height of the lane system in
    %           meters
    %   On Output:
    %       LBSD - an instance of the LBSD class initialized with
    %       the sample lane network.
    %   Call:
    %       lbsd = LBSD.genSampleLanes(10, 15)

    % Create an octagonal roundabout
    roundabout = nsidedpoly(8,'Center',[0 0],'SideLength',...
        lane_length_m);
    num_verts = size(roundabout.Vertices, 1);

    % Generate a node table with the roundabout vertexes
    node_table = table(roundabout.Vertices(:,1), ...
        roundabout.Vertices(:,2), ...
        altitude_m*ones(num_verts,1), ...,
        zeros(num_verts, 1), ...
        zeros(num_verts, 1), ...
        'VariableNames', ...
        {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
        'RowNames', string(1:num_verts) );

    % Generate an edge table for the roundabout vertexes
    edge_table = table([circshift(1:8,1)', (1:8)'], ...
        lane_length_m*ones(num_verts,1), 'VariableNames', ...
        {'EndNodes','Weights'},'RowNames', string(1:num_verts));

    % Add alternating land an launch sites
    total_verts = num_verts;
    total_edges = size(edge_table,1);
    % Create lookup table for adding wings to the roundabout. I
    % dont know what order matlab creates the nodes of the octagon.
    oct_angles = 22.5:45:337.5;
    oct_angles(oct_angles>180) = oct_angles(oct_angles>180) - 360;
    % Each row in this matrix has this format: [angle, x, y, z, ...
    % launch, land]
    oct_angles = [oct_angles', ...
        [1,0,0,1,0;...
        0,1,0,0,1;...
        0,1,0,1,0;...
        -1,0,0,0,1;...
        -1,0,0,1,0;...
        0,-1,0,0,1;...
        0,-1,0,1,0;...
        1,0,0,0,1]];
    for i = 1:num_verts
        vert = node_table{i,{'XData','YData','ZData'}};
        node_ang = atan2(vert(2),vert(1)) *180/pi;
        wing_ind = find(abs(node_ang-oct_angles(:,1)) < 1000*eps);
        wing_vec = vert + oct_angles(wing_ind,2:4)*lane_length_m;
        ground_vec = [wing_vec(1:2),0];

        % Create the new nodes, one to the side of the roundabout and one
        % on the ground.
        node_a = total_verts + 1; 
        node_b = total_verts + 2; 
        total_verts = total_verts + 2;
        is_launch = oct_angles(wing_ind,5);
        is_land = oct_angles(wing_ind,6);
        new_nodes = num2cell([ wing_vec, 0, 0; ground_vec, is_launch, is_land ]);
        new_nodes = cell2table( new_nodes, ...
            'VariableNames', {'XData', 'YData', 'ZData', 'Launch', 'Land'}, ...
            'RowNames', string([node_a, node_b]) );
        node_table = [node_table; new_nodes];
        % Create the new edges
        if is_launch
            new_edges = { [node_b, node_a], altitude_m; ...
                [node_a, i], lane_length_m };
        else
            new_edges = { [i, node_a], lane_length_m; [node_a, node_b], ...
                altitude_m };
        end
        edge_a = total_edges + 1;
        edge_b = total_edges + 2;
        total_edges = total_edges + 2;
        new_edges = cell2table(new_edges, ...
            'VariableNames', {'EndNodes','Weights'},...
            'RowNames', string([edge_a, edge_b]));
        edge_table = [edge_table; new_edges];
    end
    node_table.Name = node_table.Properties.RowNames;
    % Instantiate an LBSD object
    lbsd = LBSD();

    % Initialize the LBSD object with the generated lane graph
    lbsd.lane_graph = digraph(edge_table, node_table);
end