function metrics = run_comb_test(num_trials, run_parallel, show_waitbar)
%RUN_RENYI_TEST Summary of this function goes here
%   Detailed explanation goes here
if nargin < 1
    num_trials = 100;
end
if nargin < 2
    run_parallel = true;
end
if nargin < 3
    show_waitbar = false;
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

% 1. Network Structure (roads, grid, Delaunay, mono?)
% 2. Headway distance (low, medium, high)
% 3. Speed (Fixed s in {L,M,H} or Variable s in [L,H])
% 4. Graph density (|E|/(3|N|-6) in {low,medium,high})
% 5. Job characteristics
%      a. flex  in {small,large}
%      b. launch rate (avg flights/site/time) in {low,high}
%      c. path in {shortest, sample shortest set, random}
%      d. total simulation time in {short,long}
% 
% Element:  1  2  3  4  5a 5b 5c 5d
% Combos:  3x3x4x3x 2x 2x 3x 2     = 864
% 
% If 500/40min, then about 70 min for 1 trial each, so 10 trials each is
% 700min total (~12 hours)


function metric = to_eval()
    test_conf.net_struct= "grid";
    test_conf.h_d = 5;
    test_conf.speed = 5;
    test_conf.graph_density = 50;
    test_conf.flex = 5*60;
    test_conf.sim_time = 4*60*60;
    test_conf.launch_rate = 1200/60/60;

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
        lbsd = LBSD.LEM_gen_grid_roads(xmin,xmax,ymin,ymax,dx,dy);
    end
    
    sim.lbsd = lbsd;
    
    % Initialize the uas configuration
    uas_config = UASConfig();
    uas_config.num_uas = round(test_conf.launch_rate*test_conf.sim_time);
%     uas_config.num_uas = 100;
    uas_config.setSpeedMix('CONSTANT',test_conf.speed)
    uas_config.setHeadwayMix('CONSTANT',test_conf.h_d)
    uas_config.setFlexMix('CONSTANT',test_conf.flex)
    
    sim.uas_config = uas_config;
    
    % initialize the simulation configuration
    sim_config = SimConfig();
    sim_config.t0 = 0;
    sim_config.tf = sim_time;
%     sim_config.fit_traj = true;
    sim_config.fit_traj = false;
%     sim_config.single_lane = "1";
    
    sim.sim_config = sim_config;
    
    sim.initialize(false);
    metric = sim.sim_metrics;
%     sim.sim_metrics.h_sim = sim;
end
