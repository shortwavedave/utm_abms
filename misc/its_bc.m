function its_bc()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

xmin = 0;
xmax = 5000;
ymin = 0;
ymax = 5000;
dx = 500;
dy = 500;
min_dist = 50;
grid_lbsd = LBSD.LEM_gen_grid_roads(xmin,xmax,ymin,ymax,dx,dy,min_dist);
grid_lbsd.name = "Grid Network";

xmin = 0;
xmax = 5000;
ymin = 0;
ymax = 5000;
num_vertexes = 100;
min_dist = 50;
min_rb_dist = 30;
delaunay_lbsd = LBSD.LEM_gen_Delaunay_roads(xmin, xmax, ymin, ymax,...
    num_vertexes, min_dist, min_rb_dist);
delaunay_lbsd.name = "Delaunay Network";

xmin = 0;
xmax = 5000;
ymin = 0;
ymax = 5000;
dx = 500;
dy = 500;
min_dist = 50;
g_del_lbsd = LBSD.LEM_gen_grid_roads_del(xmin,xmax,ymin,ymax,dx,dy,min_dist);
g_del_lbsd.name = "Grid-Delaunay Network";

l = load("gis_lbsd_100N.mat");
gis_lbsd = l.lbsd;
gis_lbsd.name = "GIS Network";

bcs = [];
for lbsd = [grid_lbsd delaunay_lbsd g_del_lbsd gis_lbsd]
    bc_roads = lbsd.LEM_SNM_betweenness_centrality_node(true);
    bc_airways = lbsd.LEM_SNM_betweenness_centrality_node(false);
    xyz_r = [lbsd.road_graph.Nodes{:,["XData","YData"]}, bc_roads];
    xyz_a = [lbsd.lane_graph.Nodes{:,["XData","YData"]}, bc_airways];
    tri_r = delaunay(xyz_r(:,1), xyz_r(:,2));
    tri_a = delaunay(xyz_a(:,1), xyz_a(:,2));
    figure;
    trimesh(tri_r, xyz_r(:,1), xyz_r(:,2), bc_roads);
    title(sprintf("%s Roads", lbsd.name));
    figure;
    trimesh(tri_a, xyz_a(:,1), xyz_a(:,2), bc_airways);
    title(sprintf("%s Airways", lbsd.name));
end



