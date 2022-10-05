xmin = 0;
xmax = 5000;
ymin = 0;
ymax = 5000;
num_vertexes = 100;
min_dist = 50;
min_rb_dist = 30;
lbsd = LBSD.LEM_gen_Delaunay_roads(xmin, xmax, ymin, ymax,...
num_vertexes, min_dist, min_rb_dist);
lbsd.graph2json("delaunay.json");
density = 100;
sim = Sim();
lbsd.setPreallocations(1000000);
sim.lbsd = lbsd;
uas_config = UASConfig();
launch_rate = density/60/60;
sim_time = 4*60*60;
uas_config.num_uas = round(launch_rate*sim_time);
uas_config.num_uas
speed = 5
h_d = 5
trials.speeds = 5
flex = 1800
uas_config.setSpeedMix('CONSTANT',speed);
uas_config.setClimbRateMix('CONSTANT',speed);
uas_config.setHeadwayMix('CONSTANT',h_d);
uas_config.setFlexMix('CONSTANT',flex);
sim.uas_config = uas_config;
sim_config = SimConfig();
sim_config.t0 = 0;
sim_config.tf = sim_time;
sim_config.fit_traj = true;
sim.en_init_radar = false;
sim.en_init_atoc = false;
sim.initialize(false);
figure
plot(toa_s)
toa_s
sim.initialize(false);
[s] = trajDump(sim, "delaunay_uas.json")