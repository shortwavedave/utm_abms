classdef UAS < handle
    % UAS An instance of this class represents a UAS agent
    %   Detailed explanation goes here
    
    properties
        % The UAS agent's unique identifier (string)
        id
        % The UAS agent's GPS sensor, an instance of the GPS class
        gps
        % The UAS agent's knowledge base, an instance of the KB class
        kb
        % An instance of the LBSD class
        lbsd
        % The nominal climb rate in m/s
        climb_rate = 100.0
        % The nominal ground speed during flight in m/s
        nominal_speed = 1.0
        % The sample rate of set points along a planned trajectory in Hz
        set_point_hz = 1.0
        % Reservation IDs
        res_ids = []
        % Telemetry Listeners
        telemetry_listeners = []
        % Planned trajectory
        traj = []
        % True if this UAS desired plan could not be scheduled
        failed_to_schedule = false
        % Current step in trajectory execution
        traj_step_i = 0;
        % Request time (the original request time)
        r_s = 0;
        % Planned arrivals
        toa_s = []
        % Current time
        t_s = 0
        % True if the UAS is executing a flight
        active = false
        % This UAS required headway distance
        h_d = 10.0
        % This value defines the the interval around which a UAS would
        % accept a launch time
        flex = 0
        % plot handle associated with this uas
        h
    end
    
    properties (Dependent = true)
        % Executed trajectory - The trajectory positions that have been
        % executed (nx3)
        exec_traj
        % Executed trajectory orientation (n quaternion)
        exec_orient
        % Executed trajectory velocity (nx3)
        exec_vel
        % Executed trajectory acceleration (nx3)
        exec_accel
        % Executed trajectory angular velocity (nx3)
        exec_ang_vel   
    end
    
    properties (Access = protected)
        % Executed trajectory - The trajectory positions that have been
        % executed (nx3)
        m_exec_traj = []
        % Executed trajectory orientation (n quaternion)
        m_exec_orient = []
        % Executed trajectory velocity (nx3)
        m_exec_vel = []
        % Executed trajectory acceleration (nx3)
        m_exec_accel = []
        % Executed trajectory angular velocity (nx3)
        m_exec_ang_vel = []
    end
    
    events
        telemetry
    end
       
    methods
        function obj = UAS(id)
            %UAS Construct a UAS agent
            %   On Input:
            %       id (optional) - string A unique identifier for this 
            %       agent
            if nargin < 1
                id = java.util.UUID.randomUUID.toString;
            end
            if ~isa(id, 'string')
                id = string(id);
            end
            obj.id = id;
            obj.initialize();
        end
        
        function traj = get.exec_traj(obj)
           traj = [];
           if obj.traj_step_i > 0
               traj = obj.m_exec_traj(1:obj.traj_step_i, :);
           end
        end
        
        function set.exec_traj(obj, traj)
            obj.m_exec_traj = traj;
        end
        
        function traj = get.exec_orient(obj)
           traj = [];
           if obj.traj_step_i > 0
               traj = obj.m_exec_orient(1:obj.traj_step_i, :);
           end
        end
        
        function set.exec_orient(obj, traj)
            obj.m_exec_orient = traj;
        end
        
        function traj = get.exec_vel(obj)
           traj = [];
           if obj.traj_step_i > 0
               traj = obj.m_exec_vel(1:obj.traj_step_i, :);
           end
        end
        
        function set.exec_vel(obj, traj)
            obj.m_exec_vel = traj;
        end
        
        function traj = get.exec_accel(obj)
           traj = [];
           if obj.traj_step_i > 0
               traj = obj.m_exec_accel(1:obj.traj_step_i, :);
           end
        end
        
        function set.exec_accel(obj, traj)
            obj.m_exec_accel = traj;
        end
        
        function traj = get.exec_ang_vel(obj)
           traj = [];
           if obj.traj_step_i > 0
               traj = obj.m_exec_ang_vel(1:obj.traj_step_i, :);
           end
        end
        
        function set.exec_ang_vel(obj, traj)
            obj.m_exec_ang_vel = traj;
        end
        
        function reset(obj)
            % reset Reinitialize this object;
            if ~isempty(obj.traj)
                obj.traj.reset();
                obj.t_s = 0;
                obj.traj_step_i = 0;
            end
        end
        
        function step = stepTrajectory(obj)
            obj.t_s = obj.t_s + 1/obj.set_point_hz;
            if ~isempty(obj.traj) ...
                    && obj.t_s >= obj.toa_s(1) ...
                    && obj.t_s <= obj.toa_s(end)
                obj.active = true;
                obj.traj_step_i = obj.traj_step_i + 1;
                [pos,ori,vel,acc,ang_vel] = obj.traj.step();
                obj.m_exec_traj(obj.traj_step_i,:) = pos;
                obj.m_exec_orient(obj.traj_step_i,:) = ori;
                obj.m_exec_vel(obj.traj_step_i,:) = vel;
                obj.m_exec_accel(obj.traj_step_i,:) = acc;
                obj.m_exec_ang_vel(obj.traj_step_i,:) = ang_vel;
            else
                obj.active = false;
            end
            step = obj.traj_step_i;
        end
        
        function subscribeToTelemetry(obj, subscriber)
            % subscribeToTelemetry Set an event listener to trigger when 
            %   a telemetry change occurs.
            % On input
            %   obj - an instance of the UAS class
            %   subscriber - a function handle to trigger
            % Example:
            %   uas.subscribeToTelemetry(@atoc.handleEvent);
            lh = obj.addlistener('telemetry', subscriber);
            obj.telemetry_listeners = [obj.telemetry_listeners, lh];
        end
        
        function [traj, lane_ids, vert_ids, toa_s, ok] = ...
                createTrajectoryRandVerts(obj, fit_traj)
            % createTrajectory Construct a trajectory between two locations.
            %	The locations may be any 2D locations and this method will
            %   find the closest land and launch vertexes in the lane
            %   system. This method reserves trajectories using the LBSD.
            %   On Input:-
            %       x0 - 1x2 position in meters [x,y]
            %       xf - 1x2 position in meters [x,y]
            %   On Output:
            %       traj - Trajectory object
            %       lane_ids - [nx1] string of lane identifiers
            %       vert_ids - [(n+1)x1] string of vertex identifiers
            %       toa_s - [(n+1)x1] float seconds arrival at each vertex
            %       The first arrival time is always zero
            traj = [];
            lane_ids = [];
            vert_ids = [];
            toa_s = [];
            ok = false;
            ok = true;
            if nargin < 2
                fit_traj = true;
            end
            f_hz = obj.set_point_hz;
            
%             launch_vert = obj.lbsd.getClosestLaunchVerts(x0);
%             launch_vert = launch_vert(1);
            launch_vert = obj.lbsd.getRandLaunchVert();
            land_vert = obj.lbsd.getRandLandVert();
            
%             land_vert = obj.lbsd.getClosestLandVerts(xf);
%             land_vert = land_vert(1);
            
            [lane_ids, vert_ids, ~] = obj.lbsd.getShortestPath(...
                launch_vert, land_vert);
            waypoints_m = obj.lbsd.getVertPositions(vert_ids);

            num_wps = size(waypoints_m,1);
            if num_wps < 2
                ok = false;
                return;
%                 error("Number of Waypoints must be at least 2");
            end
                        
            
            [toa_s, ground_speed_ms, climb_rate_ms] = calc_toas(obj, waypoints_m);

            new_waypts = [];
            num_interp = 100;
            for i = 2:num_wps
               p1 = waypoints_m(i-1,:);
               p2 = waypoints_m(i,:);
               xi = linspace(p1(1),p2(1),num_interp);
               yi = linspace(p1(2),p2(2),num_interp);
               zi = linspace(p1(3),p2(3),num_interp);
               new_waypts = [new_waypts; xi(2:end)' yi(2:end)' zi(2:end)'];
            end
            t_waypoints_m = new_waypts;
%             num_wps = size(waypoints_m,1);
            [t_toa_s, t_ground_speed_ms, t_climb_rate_ms] = calc_toas(obj, t_waypoints_m);
            
            if fit_traj
                try
                    traj = Trajectory(f_hz, t_waypoints_m, t_toa_s, ...
                        t_ground_speed_ms, t_climb_rate_ms);
                catch e
                    warning("Failed to create traj");
                    warning(e.message);
                    traj = [];
                end
                
            else
                traj = [];
            end
            toa_s = toa_s';
        end
        
        function [toa_s, ground_speed_ms, climb_rate_ms] = calc_toas(obj, waypoints_m)
            dista = waypoints_m(1:end-1,:);
            distb = waypoints_m(2:end,:);
            dist = distb - dista;
            planar_dists = sqrt(dist(:,1).^2 + dist(:,2).^2);
            climb_dists = abs(dist(:,3));
            num_wps = size(waypoints_m,1);
            toa_s(1) = 0;
            toa_s(num_wps) = 0;
            ground_speed_ms(1) = 0;
            ground_speed_ms(num_wps) = 0;
            climb_rate_ms(1) = 0;
            climb_rate_ms(num_wps) = 0;
            for i = 2:num_wps
                dist_i = i-1;
                planar_toa = planar_dists(dist_i)/obj.nominal_speed;
                climb_toa = climb_dists(dist_i)/obj.climb_rate;
                toa_s(i) = toa_s(i-1)+max(planar_toa, climb_toa);
                if i == num_wps || i == 2
                    ground_speed_ms(i) = 0;
                    climb_rate_ms(i) = 0;
                else
                    climb_rate_ms(i) = 0;
                    ground_speed_ms(i) = obj.nominal_speed;
                end
            end
        end
        
        function [traj, lane_ids, vert_ids, toa_s, ok] = ...
                createTrajectory(obj, x0, xf, fit_traj)
            % createTrajectory Construct a trajectory between two locations.
            %	The locations may be any 2D locations and this method will
            %   find the closest land and launch vertexes in the lane
            %   system. This method reserves trajectories using the LBSD.
            %   On Input:-
            %       x0 - 1x2 position in meters [x,y]
            %       xf - 1x2 position in meters [x,y]
            %   On Output:
            %       traj - Trajectory object
            %       lane_ids - [nx1] string of lane identifiers
            %       vert_ids - [(n+1)x1] string of vertex identifiers
            %       toa_s - [(n+1)x1] float seconds arrival at each vertex
            %       The first arrival time is always zero
            traj = [];
            lane_ids = [];
            vert_ids = [];
            toa_s = [];
            ok = false;
            ok = true;
            if nargin < 4
                fit_traj = true;
            end
            f_hz = obj.set_point_hz;
            launch_vert = obj.lbsd.getClosestLaunchVerts(x0);
            launch_vert = launch_vert(1);
            land_vert = obj.lbsd.getClosestLandVerts(xf);
            land_vert = land_vert(1);
            [lane_ids, vert_ids, ~] = obj.lbsd.getShortestPath(...
                launch_vert, land_vert);
            waypoints_m = obj.lbsd.getVertPositions(vert_ids);
            
            num_wps = size(waypoints_m,1);
            if num_wps < 2
                ok = false;
                return;
%                 error("Number of Waypoints must be at least 2");
            end
            
            dista = waypoints_m(1:end-1,:);
            distb = waypoints_m(2:end,:);
            dist = distb - dista;
            planar_dists = sqrt(dist(:,1).^2 + dist(:,2).^2);
            climb_dists = abs(dist(:,3));
            
            toa_s(1) = 0;
            toa_s(num_wps) = 0;
            ground_speed_ms(1) = 0;
            ground_speed_ms(num_wps) = 0;
            climb_rate_ms(1) = 0;
            climb_rate_ms(num_wps) = 0;
            for i = 2:num_wps
                dist_i = i-1;
                planar_toa = planar_dists(dist_i)/obj.nominal_speed;
                climb_toa = climb_dists(dist_i)/obj.climb_rate;
                toa_s(i) = toa_s(i-1)+max(planar_toa, climb_toa);
                if i == num_wps
                    ground_speed_ms(i) = 0;
                    climb_rate_ms(i) = 0;
                else
                    climb_rate_ms(i) = 0;
                    ground_speed_ms(i) = obj.nominal_speed;
                end
            end
            
            if fit_traj
                try
                    traj = Trajectory(f_hz, waypoints_m, toa_s, ...
                        ground_speed_ms, climb_rate_ms);
                catch e
                    warning("Failed to create traj");
                    warning(e);
                    traj = [];
                end
            else
                traj = [];
            end
            toa_s = toa_s';
        end
        
        function [traj, lane_ids, vert_ids, toa_s] = ...
                createTrajectoryLane(obj, lane_id, fit_traj)
            % createTrajectoryInds Construct a trajectory on a lane.
            %   On Input:
            %       lane_id - string id of lane
            %   On Output:
            %       traj - Trajectory object
            %       lane_ids - [nx1] string of lane identifiers
            %       vert_ids - [(n+1)x1] string of vertex identifiers
            %       toa_s - [(n+1)x1] float seconds arrival at each vertex
            %       The first arrival time is always zero
            if nargin < 3
                fit_traj = true;
            end
            f_hz = obj.set_point_hz;
            lane_ids = [lane_id];
            vert_ids = obj.lbsd.getLaneVertexes(lane_id);
            waypoints_m = obj.lbsd.getVertPositions(vert_ids);
            
            num_wps = size(waypoints_m,1);
            if num_wps < 2
                error("Number of Waypoints must be at least 2");
            end
            
            dista = waypoints_m(1:end-1,:);
            distb = waypoints_m(2:end,:);
            dist = distb - dista;
            planar_dists = sqrt(dist(:,1).^2 + dist(:,2).^2);
            climb_dists = dist(:,3);
            
            toa_s(1) = 0;
            toa_s(num_wps) = 0;
            ground_speed_ms(1) = 0;
            ground_speed_ms(num_wps) = 0;
            climb_rate_ms(1) = 0;
            climb_rate_ms(num_wps) = 0;
            for i = 2:num_wps
                dist_i = i-1;
                planar_toa = planar_dists(dist_i)/obj.nominal_speed;
                climb_toa = climb_dists(dist_i)/obj.climb_rate;
                toa_s(i) = max(planar_toa, climb_toa);
                if i == num_wps
                    ground_speed_ms(i) = 0;
                    climb_rate_ms(i) = 0;
                else
                    climb_rate_ms(i) = 0;
                    ground_speed_ms(i) = obj.nominal_speed;
                end
            end
            
            if fit_traj
                traj = Trajectory(f_hz, waypoints_m, toa_s, ...
                    ground_speed_ms, climb_rate_ms);
            else
                traj = [];
            end
            toa_s = toa_s';
        end
    end
    
    methods (Access = private)
       function initialize(obj)
            % initialize Initialize the internal class datastructures
            obj.gps = GPS();
            obj.gps.subscribeToStateChange(@obj.handleGPS);
%             obj.kb = KB();    
        end
        
        function handleGPS(obj, ~, ~)
            % handleGPS handle a GPS state update event. The updated gps
            % information is contained in the gps property of this object.
            % Call obj.gps to access the gps data.
%             disp('GPS Received');
            obj.notifyTelemetryChange();
        end
        
        function notifyTelemetryChange(obj)
            % notifyStateChange Trigger the StateChange event and notify
            % all subscribers
            notify(obj, 'telemetry');
        end
        
    end
end