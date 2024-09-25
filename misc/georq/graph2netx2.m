function [shp] = graph2netx(graph)
%NETX2ROADS Summary of this function goes here
%   Detailed explanation goes here
    num_verts = size(graph.Nodes,1);
    num_edges = size(graph.Edges,1);
    data = struct([]);
    for i = 1:num_edges
        data(i).Geometry = 'PolyLine';
        data(i).X = t(i)  ;  % latitude 
        data(i).Y = x(i) ;  % longitude 
        data(i).Name = 'sine' ;   % some random attribute/ name 
    end
    shapewrite(Data, 'myfile.shp')
