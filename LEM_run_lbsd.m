function lbsd = LEM_run_lbsd
%

lbsd = LBSD.LEM_gen_grid_roads(0,100,0,100,10,10);
%lbsd = LBSD.LEM_gen_grid_roads(0,100,0,100,50,50)
n1 = lbsd.getClosestLaunchVerts([0,0]);
n2 = lbsd.getClosestLandVerts([100,100]);
[lane_ids, vert_ids, dist] = lbsd.getShortestPath(n1,n2);
h = lbsd.plot;
lbsd.highlight(h,lane_ids,'EdgeColor','r');
figure(2);
clf
lbsd.plot_roads;
bc = lbsd.LEM_SNM_betweeness_centrality_node(1);
p.NodeCData = bc;
colorbar
