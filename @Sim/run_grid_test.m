function metrics = run_grid_test(num_trials, run_parallel, show_waitbar)
%RUN_RENYI_TEST Summary of this function goes here
%   Detailed explanation goes here
if nargin < 1
    num_trials = 100;
end
if nargin < 2
    run_parallel = true;
end
if nargin < 3
    show_waitbar = true;
end

parallel_wait = run_parallel && show_waitbar;

metrics(num_trials) = SimMetrics;
if parallel_wait
    WaitMessage = parfor_wait(num_trials, 'Waitbar', true,'ReportInterval',1);
end

tic;
if num_trials > 1 && run_parallel
    parfor i = 1:num_trials
        metrics(i) = to_eval();
        if show_waitbar
            WaitMessage.Send;
        end
    end
else
    for i = 1:num_trials
        metrics(i) = to_eval();
    end
end

toc
if parallel_wait
    WaitMessage.Destroy
end

if length(metrics) > 1
    m = [metrics.lane_occs];
    occs = [m.occ];
else 
    m = metrics.lane_occs;
    occs = m.occ;
end
plot(occs);
ylabel('Occupancy')
xlabel('Trial')
disp(["Mean occupancy: " num2str(mean(occs))])

end

function metric = to_eval()
    % Create a simulator object
    sim = Sim();
    sim_time = 0.5*60*60;   
    ft2m = 1/3.281;
    % Disable Radar initialization, since this example uses
    % a single lane on the ground
    sim.en_init_radar = false;
    
    % Define a single-lane LBSD object
    xmin = 0;
    xmax = 1000;
    ymin = 0;
    ymax = 1000;
    dx = 50;
    dy = 50;
    lbsd = LBSD.LEM_gen_grid_roads(xmin,xmax,ymin,ymax,dx,dy);
    sim.lbsd = lbsd;
    
    % Initialize the uas configuration
    uas_config = UASConfig();
    uas_config.num_uas = 500;
    uas_config.setSpeedMix('CONSTANT',30*ft2m)
    uas_config.setHeadwayMix('CONSTANT',115)
    uas_config.setFlexMix('CONSTANT',sim_time)
    
    sim.uas_config = uas_config;
    
    % initialize the simulation configuration
    sim_config = SimConfig();
    sim_config.t0 = 0;
    sim_config.tf = sim_time;
%     sim_config.fit_traj = false;
%     sim_config.single_lane = "1";
    
    sim.sim_config = sim_config;
    
    sim.initialize(false);
    metric = sim.sim_metrics;
    sim.sim_metrics.h_sim = sim;
end
