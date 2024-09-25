xmin = 0;
xmax = 5000;
ymin = 0;
ymax = 5000;
dx = 500;
dy = 500;
min_dist = 50;
lbsd = LBSD.LEM_gen_grid_roads(xmin,xmax,ymin,ymax,dx,dy,min_dist);
lbsd.lane_graph
size(lbsd.lane_graph.Nodes,1)

table2struct(lbsd.lane_graph.Nodes)
lbsd.graph2json("grid.json");
sim = Sim();
lbsd.setPreallocations(1000000);
sim.lbsd = lbsd;

uas_config = UASConfig();
density = 100;
launch_rate = density/60/60;
sim_time = 1*60*60;
uas_config.num_uas = round(launch_rate*sim_time);
uas_config.num_uas

speed = 5;
trials.headways = 5;
trials.speeds = 5;
trials.flexes = 1800;
h_d = 5;
flex = 1800;
uas_config.setSpeedMix('CONSTANT',speed);
uas_config.setClimbRateMix('CONSTANT',speed);
uas_config.setHeadwayMix('CONSTANT',h_d);
uas_config.setFlexMix('CONSTANT',flex);
sim.uas_config = uas_config;
sim_time
sim_config = SimConfig();
sim_config.t0 = 0;
sim_config.tf = sim_time;
sim_config.fit_traj = true;
sim.sim_config = sim_config;
sim.en_init_radar = false;
sim.en_init_atoc = false;
sim.initialize(false);

s = trajDump(sim, "grid_uas2.json")