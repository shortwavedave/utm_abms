function sim = dddas_sandbox(en_morph, K)
workingpath = erase(mfilename('fullpath'), mfilename());

simulate = 1;
if nargin < 1
    en_morph = false;
end

mps2fpm = 196.9; % fpm per mps
m2ft = 3.281; % ft per m
fpm2mps = 1/mps2fpm;
ft2m = 1/m2ft;
knots2mps = 1/1.944;
% Create a simulator object
sim = Sim();
% Disable Radar initialization, since this example uses
% a single lane on the ground
sim.en_init_radar = false;


% Define a single-lane LBSD object
lane_length_m = 1600*ft2m;
% num_lanes = 1;
% lbsd = LBSD.genSimpleLanes(lane_length_m*ones(1,num_lanes));
% lane_length_m = 10;
altitude_m = 1000*ft2m;
lbsd = LBSD.genSampleLanes2(lane_length_m, altitude_m);

f = figure;
plot(lbsd);

adsb_traj = traj_plot(false);
adsb_traj(:,4) = adsb_traj(:,4)*ft2m;

hold on;
plot3(adsb_traj(:,2),adsb_traj(:,3),adsb_traj(:,4),"r--","Linewidth",2);
hold off;

title("Experiment Setup");
legend("Lane System", "ADSB Path", 'Location', 'south', 'NumColumns', 2);

folder = strcat(workingpath, "../../data/DDDAS/plots");
exportgraphics(f,strcat(folder, '/conflict',num2str(K),'.png'),'Resolution',300);
exportgraphics(f,strcat(folder, '/conflict',num2str(K),'.eps'),'Resolution',300);


positions = lbsd.getVertPositions("1");
t = linspace(0,1500,500);
mapxy = zeros(500, 4);
mapxy(:,1) = t';
mapxy(:,2:4) = mapxy(:,2:4) + positions;

[xy,distance,t_a] = distance2curve(adsb_traj,mapxy,'pchip');

f_conflict = figure;
plot(t, distance, "Linewidth",2);
title("Seperation Between Lane System (Node 1) and ADSB Trajectory");
xlabel("time(s)");
ylabel("distance(m)");

folder = strcat(workingpath, "../../data/DDDAS/plots");
exportgraphics(f_conflict,strcat(folder, '/conflict_dist',num2str(K),'.png'),'Resolution',300);
exportgraphics(f_conflict,strcat(folder, '/conflict_dist',num2str(K),'.eps'),'Resolution',300);

if simulate
    sim.lbsd = lbsd;

    % Initialize the uas configuration
    uas_config = UASConfig();
    uas_config.num_uas = 300;
    uas_config.setSpeedMix('CONSTANT',30*knots2mps)
    uas_config.setHeadwayMix('CONSTANT',30*ft2m)
    uas_config.setFlexMix('CONSTANT',100)

    sim.uas_config = uas_config;

    % initialize the simulation configuration
    sim_config = SimConfig();
    sim_config.t0 = 0;
    sim_config.tf = 1500;
    sim_config.en_morph = en_morph;

    basic_morph = BasicMorph(lbsd);
    basic_morph.toa = sim_config.tf;
    basic_morph.nodes = [1:8, 9 11 13 15 17 19 21 23];
    basic_morph.record_node = "1";
    basic_morph.K = .01;
    % basic_morph.endpoints = 10 + 10*(rand(1,8)-.5);
    % basic_morph.nodes = [1, 2];
    basic_morph.endpoints = altitude_m - 600*ft2m;
    sim_config.morph = basic_morph;


    sim.sim_config = sim_config;


    sim.initialize(true);

    sim.run_sim()
end

disp("Processing closest point of approach");
[~,morph_distance,~] = distance2curve(adsb_traj,basic_morph.node_positions,'pchip');
f_conflict = figure;
plot(t, distance, "Linewidth",2);
hold on
plot(basic_morph.node_positions(:,1), morph_distance, "Linewidth",2);
hold off
legend("Static Spatial Network", "Dynamic Spatial Network");
title("Closest Point of Approach");
xlabel("time(s)");
ylabel("distance(m)");

folder = strcat(workingpath, "../../data/DDDAS/plots");
exportgraphics(f_conflict,strcat(folder, '/morph_conflict_dist',num2str(K),'.png'),'Resolution',300);
exportgraphics(f_conflict,strcat(folder, '/morph_conflict_dist',num2str(K),'.eps'),'Resolution',300);

disp("Plotting Climb Rates");
[~, ~, rate_f] = plot_sim_rates(sim);
exportgraphics(rate_f,strcat(folder, '/rates',num2str(K),'.png'),'Resolution',300);
exportgraphics(rate_f,strcat(folder, '/rates',num2str(K),'.eps'),'Resolution',300);
tch = 0;
end

