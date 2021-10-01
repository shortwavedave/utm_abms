classdef Sim < handle
    %SIM An instance of this class represents a simulation
    
    properties
        tick_del_t
        uas_list = []
        radar_list = []
        atoc
        lbsd
    end
    
    properties (Access = private)
       tick_listeners = [] 
       
    end
    
    events
        Tick
    end
    
    methods
        function obj = Sim()
            %SIM Construct an instance of this class
            
        end
        
        function initialize(obj)
            %% Setup the LBSD
            disp("Initializing LBSD");
            lane_length_m = 50;
%             altitude_m = 100;
%             obj.lbsd = LBSD.genSampleLanes(lane_length_m, altitude_m);
            obj.lbsd = LBSD.genSimpleLanes(lane_length_m*ones(1,3));
            
            %% Setup the ATOC
            disp("Initializing ATOC");
            obj.atoc = ATOC(obj.lbsd);
            
            %% Setup some UAS
            disp("Instantiating UAS");
            num_uas = 5;
            for i = 1:num_uas
                % Create a new UAS with a unique identifier
                new_uas = UAS(string(i));
                % Give this UAS a reference to the LBSD
                new_uas.lbsd = obj.lbsd;
                % Store a reference to this UAS in this simulation object
                obj.uas_list = [obj.uas_list, new_uas];
            end
            
            %% Generate trajectories and reservations
            disp("Generating Random Trajectories for UAS");
            t0 = 0;
            tf = 100;
            h_d = 10;
            % Get the extent of this lane system
            [minx, miny, maxx, maxy] = obj.lbsd.getEnvelope();
            failed_i = [];
            for i = 1:num_uas
                uas_i = obj.uas_list(i);
                % Create random start and end points
                x0 = [minx+(maxx-minx)*rand(),miny+(maxy-miny)*rand()];
                xf = [minx+(maxx-minx)*rand(),miny+(maxy-miny)*rand()];
                % Create a trajectory based on this UAS capabilities
                [traj, lane_ids, vert_ids, toa_s] = ...
                    uas_i.createTrajectory(x0, xf);
                % Generate a random request time
                r = t0+(tf-t0)*rand();
                % Move the time-of-arrival of the trajectory to this time
                toa_s = toa_s+r;
                % Reserve the trajectory
                [ok, res_ids, res_toa_s] = ...
                    obj.lbsd.reserveLBSDTrajectory(lane_ids, toa_s, ...
                    h_d, t0, tf);
                if ok
                    % The trajectory was scheduled successfully
                    uas_i.res_ids = res_ids;
                    uas_i.traj = traj;
                    uas_i.toa_s = res_toa_s;
                else
                    disp("There was an error scheduling one of the flights")
                    failed_i = [failed_i i];
                end
            
            end
            obj.uas_list(failed_i) = [];
            disp("Initialization Complete");
        end
        
        function reset(obj)
            num_uas = length(obj.uas_list);
            for i = 1:num_uas
                obj.uas_list(i).reset();
            end
        end
        
        function run_sim(obj)
            disp("Simulation Started");
            num_steps = 1200;
            num_uas = length(obj.uas_list);
            pos_x = zeros(num_steps, num_uas);
            pos_y = zeros(num_steps, num_uas);
            pos_z = zeros(num_steps, num_uas);
            obj.lbsd.plot();
            hold on;
            for i = 1:num_steps
                for j = 1:num_uas
                    uas = obj.uas_list(j);
                    uas.stepTrajectory();
                    pos = uas.exec_traj;
                   
                    pos_x(1:i, j) = pos(:,1);
                    pos_y(1:i, j) = pos(:,2);
                    pos_z(1:i, j) = pos(:,3);
                end
%                 p = plot3(pos_x(1:i,:),pos_y(1:i,:),pos_z(1:i,:),...
%                     'b-o','MarkerFaceColor','b');
                p = plot3(pos_x(1:i,:),pos_y(1:i,:),pos_z(1:i,:),'-o');
                drawnow;
                pause(.03);
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
    end
end

