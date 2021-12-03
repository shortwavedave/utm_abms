function metrics = run_comb_test(trials, run_parallel, show_waitbar)
%RUN_RENYI_TEST Summary of this function goes here
%   Detailed explanation goes here
densities  = trials.densities;
speeds = trials.speeds;
headways = trials.headways;
flexes = trials.flexes;
structs = trials.structs;

if nargin < 2
    run_parallel = true;
end
if nargin < 3
    show_waitbar = false;
end

parallel_wait = run_parallel && show_waitbar;

num_trials = length(densities)*length(speeds)*length(headways)*...
    length(flexes)*length(structs);
metrics(num_trials) = SimMetrics;
if parallel_wait
    %WaitMessage = parfor_wait(num_trials, 'Waitbar', true,'ReportInterval',1);
    WaitMessage = parfor_wait(num_trials, 'FileName','screen',...
      'Waitbar', false,'ReportInterval',1);
end

% densities  = [1000,1000,100,100];
% speeds = [5,10,15];
% headways = [5,10];


test_configs = [];
for density = densities
  for speed = speeds
    for headway = headways
        for flex = flexes
            for net_struct = structs
              test_conf.net_struct = net_struct;
              test_conf.h_d = headway;
              test_conf.speed = speed;
              test_conf.graph_density = 50;
              test_conf.flex = flex;
              test_conf.sim_time = 4*60*60;
              test_conf.density = density;
              test_conf.launch_rate = density/60/60;
              test_configs = [test_configs test_conf];
            end
        end
    end
  end
end

num_tests = length(test_configs)

test_start = datetime('now')
posix_start = posixtime(test_start);
tic;
if run_parallel
    q = parallel.pool.DataQueue;
    afterEach(q, @save_metric)
    parfor i = 1:length(test_configs)
        metric = to_eval(test_configs(i));
        metric.count = i;
        metric.posix_seconds = posix_start;
        metrics(i) = metric;
        send(q, metric); 
        if show_waitbar
            WaitMessage.Send;
        end
    end
else
    for i = 1:length(test_configs)
        metrics(i) = to_eval(test_configs(i));
    end
end

toc
test_end = datetime('now')
if parallel_wait
    WaitMessage.Destroy
end

end

function save_metric(metric)
    s = sprintf("%d", round(metric.posix_seconds));
    s2 = sprintf("%s_%s_%s", "output/metrics_", s, num2str(metric.count));
    save(s2, 'metric');
end

function metric = to_eval(test_conf)
    % Create a simulator object
    sim = Sim();
    sim_time = test_conf.sim_time;
    
    % Disable Radar initialization, since this example uses
    % a single lane on the ground
    sim.en_init_radar = false;
    sim.en_init_atoc = false;
    
    % Define a LBSD object
    if test_conf.net_struct == "grid"
        xmin = 0;
        xmax = 5000;
        ymin = 0;
        ymax = 5000;
        dx = 500;
        dy = 500;
        min_dist = 50;
        lbsd = LBSD.LEM_gen_grid_roads(xmin,xmax,ymin,ymax,dx,dy,min_dist);
    elseif test_conf.net_struct == "delaunay"
        xmin = 0;
        xmax = 5000;
        ymin = 0;
        ymax = 5000;
        num_vertexes = 100;
        min_dist = 50;
        min_rb_dist = 30;
        lbsd = LBSD.LEM_gen_Delaunay_roads(xmin, xmax, ymin, ymax,...
            num_vertexes, min_dist, min_rb_dist);
    elseif test_conf.net_struct == "g_delaunay"
        xmin = 0;
        xmax = 5000;
        ymin = 0;
        ymax = 5000;
        dx = 500;
        dy = 500;
        min_dist = 50;
        lbsd = LBSD.LEM_gen_grid_roads_del(xmin,xmax,ymin,ymax,dx,dy,min_dist);
    elseif test_conf.net_struct == "gis"
        l = load("gis_lbsd_100N.mat");
        lbsd = l.lbsd;
    else
        error("Unknown lane network structure: %s", ...
            test_conf.net_struct);
    end
    
    lbsd.setPreallocations(1000000);
    sim.lbsd = lbsd;
    
    % Initialize the uas configuration
    uas_config = UASConfig();
    uas_config.num_uas = round(test_conf.launch_rate*test_conf.sim_time);
    uas_config.setSpeedMix('CONSTANT',test_conf.speed);
    uas_config.setClimbRateMix('CONSTANT',test_conf.speed);
    uas_config.setHeadwayMix('CONSTANT',test_conf.h_d);
    uas_config.setFlexMix('CONSTANT',test_conf.flex);
    
    sim.uas_config = uas_config;
    
    % initialize the simulation configuration
    sim_config = SimConfig();
    sim_config.t0 = 0;
    sim_config.tf = sim_time;
    sim_config.fit_traj = false;
    
    sim.sim_config = sim_config;
    
    sim.initialize(false);
    metric = sim.sim_metrics;
    metric.test_config = test_conf;
%     sim.sim_metrics.h_sim = sim;
end
