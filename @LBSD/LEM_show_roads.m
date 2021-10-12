function LEM_show_roads(roads,b)
% LEM_show_roads - show road layout
% On input:
%     roads (road struct): road info
%       .vertexes (nx3 array): x,y,z coords
%       .edges (mx2 array): vertex indexes of edges
%     b (Boolean): 1 means clf; 0 otherwise
% On output:
%     <displays figure of roads>
% Call:
%     LEM_show_roads(roads_d);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

vertexes = roads.vertexes;
edges = roads.edges;
num_edges = length(edges(:,1));

xmin = min(vertexes(:,1));
xmax = max(vertexes(:,1));
ymin = min(vertexes(:,2));
ymax = max(vertexes(:,2));
xspread = xmax - xmin;
yspread = ymax - ymin;
xbor = xspread*0.1;
ybor = yspread*0.1;

if b==1
    figure(1);
    clf
end
plot(xmin-xbor,ymin-ybor,'w.');
hold on
plot(xmax+xbor,ymax+ybor,'w.');

if max(vertexes(:,3))==0
    plot(vertexes(:,1),vertexes(:,2),'ko');
    for e = 1:num_edges
        vertex1 = edges(e,1);
        vertex2 = edges(e,2);
        plot([vertexes(vertex1,1),vertexes(vertex2,1)],...
            [vertexes(vertex1,2),vertexes(vertex2,2)],'k');
    end
else
    plot3(vertexes(:,1),vertexes(:,2),vertexes(:,3),'ko');
    for e = 1:num_edges
        vertex1 = edges(e,1);
        vertex2 = edges(e,2);
        plot3([vertexes(vertex1,1),vertexes(vertex2,1)],...
            [vertexes(vertex1,2),vertexes(vertex2,2)],...
            [vertexes(vertex1,3),vertexes(vertex2,3)],'k');
    end
end
