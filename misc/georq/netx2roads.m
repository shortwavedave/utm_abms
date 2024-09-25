function [roads] = netx2roads(netx)
%NETX2ROADS Summary of this function goes here
%   Detailed explanation goes here
    num_verts = size(netx.nodes,1);
    vertexes = zeros(num_verts, size(netx.nodes(1).id',2));
    for i = 1:num_verts
        vertexes(i,:) = netx.nodes(i).id';
    end

    num_links = size(netx.links,1);
    edges = zeros(num_links, 2);
    for i = 1:num_links
        s = getNodeIdx(netx.links(i).source', vertexes);
        t = getNodeIdx(netx.links(i).target', vertexes);
        edges(i,:) = [s,t];
    end

    roads.vertexes = vertexes;
    roads.edges = edges;
end

function idx = getNodeIdx(id, vertexes)
    [~,idx] = min(sum(abs(vertexes - id),2));
end