function lbsd = LEM_run_lbsd
%

lbsd = LBSD.LEM_gen_grid_roads(0,100,0,100,10,10);
%lbsd = LBSD.LEM_gen_grid_roads(0,100,0,100,50,50)
n1 = lbsd.getClosestLaunchVerts([0,0]);
n2 = lbsd.getClosestLandVerts([100,100]);
[lane_ids, vert_ids, dist] = lbsd.getShortestPath(n1,n2);
h = lbsd.plot;
lbsd.highlight(h,lane_ids,'EdgeColor','r');

% Betweenness centrality
figure(2);
clf
p = lbsd.plot_roads;
bc = lbsd.LEM_SNM_betweeness_centrality_node(1);
p.NodeCData = bc;
p.MarkerSize = 6;
colorbar
print -djpeg lanes_bc

% Closeness centrality
cc = lbsd.LEM_SNM_closeness_centrality(1);
p = lbsd.plot_roads;
p.NodeCData = cc;
p.MarkerSize = 6;
colorbar
print -djpeg lanes_cc

% Straightness centrality
sc = lbsd.LEM_SNM_straightness_centrality(1);
p = lbsd.plot_roads;
p.MarkerSize = 6;
p.NodeCData =sc;
colorbar
print -djpeg lanes_sc

tch = 0;
