function res = LEM_driver
%

rng('default');

% set up airway
lbsd = LBSD.genSampleLanes(10, 15);   % Need to change this
roads = lbsd.LEM_gen_grid_roads(0,50,0,50,10,10);
airways = lbsd.LEM_gen_airways(roads,[1:6],[31:36],2,40,50);
LBSD.LEM_show_airways3D(lbsd,airways,[]);

% run simple simulation
num_flights = 100;
t_min = 0;
t_max = 50;
launch_time_spread = 10;
b = 1;
res = LBSD.LEM_sim1_LBSD_51x51(lbsd,num_flights,airways,t_min,t_max,...
    launch_time_spread,b);
