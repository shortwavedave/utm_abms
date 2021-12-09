classdef Sim < handle
    %SIM An instance of this class represents a simulation
    
    properties
        tick_del_t
        uas_list
        radar_list = []
        atoc
        lbsd
        % radar_config (radar config struct): contains
        %     .range (float): the maximum range the radar field can have
        %     .noise (3x3): noise that each radar will have
        %     .angle (float): the radian angle from middle of radar 
        %       field to the edge.
        radar_config
        % uas_config (object of UASConfig class)
        uas_config
        % sim_config (object of SimConfig class)
        sim_config
        % sim_metrics (SimMetrics object) includes all metrics collected
        % during initialization and running of this simulation
        sim_metrics
        % If True, enable initialization of radar objects
        en_init_radar = true
        % If True, enable initialization of radar objects
        en_init_atoc = true
        % The frequency of the simulation steps in hertz
        step_rate_hz = 1
    end
    
    properties (Access = private)
        tick_listeners = []
        
    end
    
    events
        Tick
    end
    
    methods(Static)
        radars = LEM_radars_placement_coverage(lbsd, range, noise, angle);
        [coverage, pts] = LEM_monte_carlo(lbsd, radars, num_samples)
        aplha = LEM_posori(theta)
    end
    
    methods
        function obj = Sim()
            %SIM Construct an instance of this class
            obj.setDefaultRadarConfig();
            obj.setDefaultUASConfig();
            obj.setDefaultSimConfig();
            obj.sim_metrics = SimMetrics();
        end
        
        function ok = initialize(obj, en_disp)
            % initialize - intialize the simulation object
            % On Input:
            %   obj (Sim object): Instance of the Sim class
            %   en_disp (optional boolean): If true, print initialization 
            %       steps. Default is false
            %
            % On Output:
            %   ok (boolean): true if initialization was successful
            
            ok = false;
            if nargin < 2
                en_disp = false;
            end
            % Setup the LBSD
            if en_disp;disp("Checking if LBSD is Instantiated");end
            if ~isa(obj.lbsd, 'LBSD')
                error("Please Initialize the lbsd Property of this Object")
            end
            
            % Setup the ATOC
            if obj.en_init_atoc
                if en_disp;disp("Initializing ATOC");end
                obj.timeFunction(@obj.initializeATOC,"init_time_atoc_s");
            end
            
            % Setup Radar
            if obj.en_init_radar
                if en_disp;disp("Initializing Radar");end
                obj.timeFunction(@obj.initializeRadar,"init_time_radar_s");
            end
            
            % Setup some UAS
            if en_disp;disp("Initializing UAS Dataset");end
            obj.timeFunction(@obj.initializeUAS,"init_time_uas_s");
            
            % Generate trajectories and reservations
            if en_disp;disp("Initializing UAS Trajectories");end
            obj.timeFunction(@obj.initializeUASTraj,"init_time_traj_s");
            
            if en_disp;disp("Updating Metrics");end
            obj.updateMetrics();
            
            if en_disp;disp("Done");end
            ok = true;
        end
        
        function reset(obj)
            num_uas = length(obj.uas_list);
            for i = 1:num_uas
                obj.uas_list(i).reset();
            end
        end
        
        function run_sim(obj)
            disp("Simulation Started");
            res = obj.lbsd.getReservations();
            minTime = min(res.entry_time_s);
            maxTime = max(res.exit_time_s);
            num_steps = floor((maxTime - minTime)/obj.step_rate_hz);
            
            num_uas = length(obj.uas_list);

            % Set up timer
            cdata  = jet(num_uas);
            del_t = 1/obj.step_rate_hz;
            obj.atoc.time = minTime;
            for numradar = 1:length(obj.radar_list)
                obj.radar_list(numradar).time = minTime;
            end
            f = figure;
            obj.lbsd.plot();
            hold on;
            axis square;
            title("Lane Simulation");
            for i = 1:num_steps
                for j = 1:num_uas
                    uas = obj.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            if uas_step == 1
                                uas.h = plot3(f.CurrentAxes, ...
                                    traj(:,1),traj(:,2),traj(:,3),...
                                    '-','Color',cdata(j,:));
                            end
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                        end
                    end
                end
                obj.step(del_t);
                %                 p = plot3(pos_x(1:i,:),pos_y(1:i,:),pos_z(1:i,:),...
                %                     'b-o','MarkerFaceColor','b');numel(xxUnq)
%                 h = plot3(f.CurrentAxes, pos_x(1:i,:),pos_y(1:i,:),pos_z(1:i,:),'-o');
%                 set(h,{'color'},num2cell(cdata,2));
                drawnow limitrate;
%                 pause(.03);
            end
            hold off;
            disp("Simulation Complete");
        end
        
        function subscribe_to_tick(obj, subscriber)
            %Set an event listener to trigger when a tick event occurs
            % On input
            %   obj - an instance of the UAS class
            %   subscriber - a function handle to trigger
            % Example:
            %   sim.subscribe_to_tick(@uas.step);
            lh = obj.addlistener('Tick', subscriber);
            obj.tick_listeners = [obj.tick_listeners, lh];
        end
        
        function step(obj, del_t)
            %STEP Step the simulation
            % On Input:
            %   obj - An instalnce of the Sim class
            %   del_t - the simulation time represented by the tick
            obj.tick_del_t = del_t;
            notify(obj, 'Tick');
        end
        
        function handle_tick(obj, src, event)
            % HANDLE_TICK handle a Tick event
            if event.EventName == "Tick"
                del_t = src.tick_del_t;
                obj.step(del_t);
            end
        end
        
        function uas_list = getSuccessfulUAS(obj)
            uas_list = obj.uas_list(~[obj.uas_list.failed_to_schedule]);
        end
        
        function plotTrajsX(obj)
            uas_list = obj.uas_list(~[obj.uas_list.failed_to_schedule]);
            num_uas = length(uas_list);
            cdata = jet(num_uas);
            hold on;
            ts = zeros(1,num_uas);
            ps{num_uas} = [];
            min_hd = 0;
            for i = 1:num_uas
                uas = uas_list(i); 
                if ~uas.failed_to_schedule
                    t0 = uas.toa_s(1);
                    tf = uas.toa_s(end);
                    s = uas.nominal_speed;
                    p = uas.exec_traj(:,1);
                    ps{i} = p;
                    t = linspace(t0,tf,length(p));
                    linear_pos = (t-t0)*s;
                    max_dist = max(abs(linear_pos'-p));
                    if max_dist > min_hd
                        min_hd = max_dist;
                    end
                    ts(i) = t(1);
                    plot(t,p,"Color",cdata(i,:));
                end
            end

            obj.lbsd.plotLaneDiagram("1");
            hold off;
        end
        
        function max_dev = getMaxXDeviation(obj)
            uas_list = obj.uas_list(~[obj.uas_list.failed_to_schedule]);
            num_uas = length(uas_list);
            min_hd = 0;
            for i = 1:num_uas
                uas = uas_list(i); 
                if ~uas.failed_to_schedule
                    t0 = uas.toa_s(1);
                    tf = uas.toa_s(end);
                    s = uas.nominal_speed;
                    p = uas.exec_traj(:,1);
                    t = linspace(t0,tf,length(p));
                    linear_pos = (t-t0)*s;
                    max_dist = max(abs(linear_pos'-p));
                    if max_dist > min_hd
                        min_hd = max_dist;
                    end
                end
            end
            max_dev =  min_hd;
        end
    end
    
    methods (Access = protected)
        function updateMetrics(obj)
            lane_ids = obj.lbsd.getLaneIds;
            res = obj.lbsd.getReservations();
            % TODO the start and end time that define where occupancy is 
            % measured needs to be updated. Currently using simulation time
            % which doesn't make sense for lanes in the middle of the
            % system
            t0 = obj.sim_config.t0;
            tf = obj.sim_config.tf;
            for i = 1:length(lane_ids)
                lane_id = lane_ids(i);
                lane_res = res(res.lane_id == lane_id, :);
%                 t0 = min(lane_res.entry_time_s);
%                 tf = max(lane_res.entry_time_s);
                [d, n] = obj.lbsd.getLaneOccupancy(lane_id, t0, tf);
                lane_occ.lane_id = lane_id;
                lane_occ.num_uas = n;
                lane_occ.occ = d;
                obj.sim_metrics.lane_occs = ...
                    [obj.sim_metrics.lane_occs, lane_occ];
            end
        end
        
        function timeFunction(obj, fn, metric_name)
            tStart = tic;
            fn();
            t = toc(tStart);
            set(obj.sim_metrics,metric_name,t);
        end
        
        function setDefaultRadarConfig(obj)
            obj.radar_config.range = 110; 
            obj.radar_config.noise = eye(3);
            obj.radar_config.angle = pi/4;
        end
        
        function setDefaultUASConfig(obj)
            obj.uas_config = UASConfig();
        end
        
        function setDefaultSimConfig(obj)
            obj.sim_config = SimConfig();
        end
        
        function radars = initializeRadar(obj)
            % initializeRadar - intialize the radar system
            range = obj.radar_config.range; 
            noise = obj.radar_config.noise;
            angle = obj.radar_config.angle;
            radars = Sim.LEM_radars_placement_coverage(obj.lbsd, ...
                range, noise, angle);
            for row = 1:size(radars, 2)
                id = radars(row).id;
                pos = [radars(row).x, radars(row).y, radars(row).z];
                range = radars(row).max_range;
                apexAngle = radars(row).phi;
                dir = [radars(row).dx, radars(row).dy, radars(row).dz];
                radar = RADAR(pos, range, apexAngle, dir, id, obj.lbsd);
                radar.time = 0;
                obj.subscribe_to_tick(@radar.handle_events);
                obj.radar_list = [obj.radar_list; radar];
            end
        end
        
        function initializeATOC(obj)
            % intializeATOC = intitialize the ATOC object
            obj.atoc = ATOC(obj.lbsd);
            atoc2 = obj.atoc;
            obj.subscribe_to_tick(@atoc2.handle_events)
        end
        
        function initializeUAS(obj)
            num_uas = obj.uas_config.num_uas;
            % Matlab idiosyncrasy, have to create the object array in a
            % different structure than the one we ultimately assign it to
            uass(num_uas) = UAS;
            obj.uas_list = uass;
            for i = 1:num_uas
                uas = obj.uas_list(i);
                % Assign a simple id to the UAS 
                uas.id = string(i);
                % Give this UAS a reference to the LBSD
                uas.lbsd = obj.lbsd;
                obj.uas_config.configureUAS(uas);
            end
        end
        
        function initializeUASTraj(obj)
            num_uas = obj.uas_config.num_uas;
            t0 = obj.sim_config.t0;
            tf = obj.sim_config.tf;
            % Get the extent of this lane system
            [minx, miny, maxx, maxy] = obj.lbsd.getEnvelope();
            failed_i = zeros(num_uas,1);
            num_fail = 0;
            num_success = 0;
            success_i = zeros(num_uas,1);
            res_times = zeros(num_uas,1);
            res_delays = zeros(num_uas,1);
            for i = 1:num_uas
                uas_i = obj.uas_list(i);
                if isempty(obj.sim_config.single_lane)
                    % Create random start and end points
                    x0 = [minx+(maxx-minx)*rand(),miny+(maxy-miny)*rand()];
                    xf = [minx+(maxx-minx)*rand(),miny+(maxy-miny)*rand()];
                    % Create a trajectory based on this UAS capabilities
                    [traj, lane_ids, vert_ids, toa_s] = ...
                        uas_i.createTrajectory(x0, xf, obj.sim_config.fit_traj);
                else
                    l = obj.sim_config.single_lane;
                    [traj, lane_ids, vert_ids, toa_s] = ...
                        uas_i.createTrajectoryLane(l, obj.sim_config.fit_traj);
                end
                % Generate a random request time
                r = t0+(tf-t0)*rand();
                uas_i.r_s = r;
                % Move the time-of-arrival of the trajectory to this time
                toa_s = toa_s+r;
                % The earliest and latest possible release
                % For renyi set to the request time
                r_t0 = max(r-uas_i.flex, t0);
                r_tf = min(r+uas_i.flex, tf);
                
                % Reserve the trajectory
                timerVal = tic;
                [ok, res_ids, res_toa_s] = ...
                    obj.lbsd.reserveTrajectory(lane_ids, uas_i.id, toa_s, ...
                    uas_i.h_d, r_t0, r_tf);
                res_times(i) = toc(timerVal);
                if ok
                    % The trajectory was scheduled successfully
                    res_delays(i) = abs(res_toa_s(1)-r);
                    uas_i.res_ids = res_ids;
                    uas_i.traj = traj;
                    uas_i.toa_s = res_toa_s;
                    step_cnt = ceil(...
                        (res_toa_s(end)-res_toa_s(1))*obj.step_rate_hz...
                        )+1;
                    uas_i.exec_traj = zeros(step_cnt,3);
                    uas_i.exec_orient = quaternion(zeros(step_cnt,4));
                    uas_i.exec_vel = zeros(step_cnt,3);
                    % Executed trajectory acceleration (nx3)
                    uas_i.exec_accel = zeros(step_cnt,3);
                    % Executed trajectory angular velocity (nx3)
                    uas_i.exec_ang_vel = zeros(step_cnt,3);
                    
                    uas_i.set_point_hz = obj.step_rate_hz;
                    num_success = num_success + 1;
                    success_i(num_success) = i;
                else
                    num_fail = num_fail + 1;
                    failed_i(num_fail) = i;
                    uas_i.failed_to_schedule = true;
                end
            end
            if num_fail < num_uas
                failed_i(num_fail+1:end) = [];
            end
            if num_success < num_uas
                success_i(num_success+1:end) = [];
            end
            res_mean = mean(res_times);
            res_var = var(res_times);
            res_median = median(res_times);
            res_max = max(res_times);
            res_min = min(res_times);
            
            delays = res_delays(success_i);
            delay_mean = mean(delays);
            delay_var = var(delays);
            delay_median = median(delays);
            delay_max = max(delays);
            delay_min = min(delays);
            
            obj.sim_metrics.delay_time.max = delay_max;
            obj.sim_metrics.delay_time.min = delay_min;
            obj.sim_metrics.delay_time.mean = delay_mean;
            obj.sim_metrics.delay_time.median = delay_median;
            obj.sim_metrics.delay_time.var = delay_var;
            
            obj.sim_metrics.reservation_time.max = res_max;
            obj.sim_metrics.reservation_time.min = res_min;
            obj.sim_metrics.reservation_time.mean = res_mean;
            obj.sim_metrics.reservation_time.median = res_median;
            obj.sim_metrics.reservation_time.var = res_var;
%             obj.sim_metrics.failed_flights_ids = failed_i;
            obj.sim_metrics.num_failed_flights = num_fail;
%             obj.sim_metrics.success_flights_ids = success_i;
            obj.sim_metrics.num_success_flights = num_success;
        end
    end
    
    methods (Static)
        metrics = run_renyi_test(num_trials, run_parallel, show_waitbar)
        metrics = run_flex_test(num_trials, run_parallel, show_waitbar)
        metrics = run_grid_test(num_trials, run_parallel, show_waitbar)
        metrics = run_comb_test(num_trials, run_parallel, show_waitbar)
    end
end

