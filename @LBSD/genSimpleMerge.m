function lbsd = genSimpleMerge(lane_length_m, angle_deg, is_merge)
    % genSimpleMerge Create a simple merging lane system
    %   The generated lane system contains thre lanes total, two merging 
    %   into or two diverging.
    %   
    %   On Input:
    %       lane_length_m - (float) the length of every lane in the 
    %           system in meters.
    %       angle_deg - (float) the angle between the merging or diverging 
    %           lanes. 
    %       is_merge - (bool) (default true) Merging system if True
    %   On Output:
    %       LBSD - an instance of the LBSD class initialized with
    %       the simple lane network.
    %   Call:
    %       lbsd = LBSD.genSimpleMerge(100, 90, true)
    
    num_verts = 4;
    num_lanes = 3;
    if nargin < 3
        is_merge = true;
    end
    
    start = [0 0 0];
    isect = [lane_length_m 0 0];
    y = lane_length_m*sin(angle_deg/2);
    x = lane_length_m*sin(angle_deg/2);
    u_end = isect + [x y 0];
    l_end = isect + [x -y 0];
    
    xs = [start(1) isect(1) u_end(1) l_end(1)]';
    ys = [start(2) isect(2) u_end(2) l_end(2)]';
    zs = [start(3) isect(3) u_end(3) l_end(3)]';
    if is_merge
        launches = [ 0 0 1 1 ]';
        lands = [ 1 0 0 0 ]';
    else
        launches = [ 1 0 0 0 ]';
        lands = [ 0 0 1 1 ]';
    end
    
    % Generate a node table 
    node_table = table(xs, ys, zs, launches, lands, ...
        'VariableNames', ...
        {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
        'RowNames', string(1:num_verts) );

    % Generate an edge table 
    if ~is_merge
        edges = [1 2; 2 3; 2 4];
    else
        edges = [4 2; 3 2; 2 1];
    end
    edge_table = table(edges, ...
        [1 1 1]'*lane_length_m, 'VariableNames', ...
        {'EndNodes','Weight'},'RowNames', string(1:num_lanes));

    node_table.Name = node_table.Properties.RowNames;
    % Instantiate an LBSD object
    lbsd = LBSD();

    % Initialize the LBSD object with the generated lane graph
    lbsd.lane_graph = digraph(edge_table, node_table);
end