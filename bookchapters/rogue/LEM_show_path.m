function LEM_show_path(airways,path)
%

pts = airways.vertexes;
xmin = min(pts(:,1));
xmax = max(pts(:,1));
ymin = min(pts(:,2));
ymax = max(pts(:,2));
zmin = min(pts(:,3));
zmax = max(pts(:,3));
plot3(xmin,ymin,zmin,'w.');
hold on
plot3(xmax,ymax,zmax,'w.');

plot3(pts(:,1),pts(:,2),pts(:,3),'k.');

a_vertexes = airways.lane_vertexes;
num_path = length(path);
for k = 1:num_path-1
    p1 = a_vertexes(path(k),:);
    p2 = a_vertexes(path(k+1),:);
    plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'r','LineWidth',4);
end
tch = 0;
