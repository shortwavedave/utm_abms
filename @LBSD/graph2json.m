function graph2json(obj, filename)
%GRAPH2JSON Summary of this function goes here
%   Detailed explanation goes here
    s.nodes = table2struct(obj.lane_graph.Nodes);
    s.edges = table2struct(obj.lane_graph.Edges);
    json = jsonencode(s);
    fid = fopen(filename,'w');
    fprintf(fid, json);
    fclose('all');
end

