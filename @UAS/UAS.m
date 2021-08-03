classdef UAS < handle
    % UAS An instance of this class represents a UAS agent
    %   Detailed explanation goes here
    
    properties
        id
        trajectories
        gps
        kb
        lbsd
        climb_rate = 1
        nominal_speed = 1
        set_point_hz = 1
    end
    
    methods
        function obj = UAS(id, lbsd)
            %UAS Construct a UAS agent
            %   On Input:
            %       id - string A unique identifier for this agent
            %       lbsd - a reference to the LBSD supplemental data
            %       service provider (SDSP).
            obj.id = id;
            obj.initialize();
            obj.lbsd = lbsd;
        end
        
        function initialize(obj)
            obj.gps = GPS();
            obj.gps.subscribeToStateChange(@obj.handleGPS);
            obj.kb = KB();
        end
        
        function reset(obj)
            obj.initialize();
        end
        
        %% Plans
        function traj = planTrajectory(obj, x0, xf)
            %planTrajectory Construct a trajectory between two locations.
            %	The locations may be any 2D locations and this method will
            %   find the closest land and launch vertexes in the lane
            %   system.
            %   On Input:
            %       x0 - 1x2 position in meters [x,y]
            %       xf - 1x2 position in meters [x,y]
            %   On Output:
            %       traj - Trajectory object
            
            f_hz = obj.set_point_hz;
%             ground_speed_ms = obj.nominal_speed;
%             climb_rate_ms = obj.climb_rate;
            
            launch_vert = obj.lbsd.getClosestLaunchVerts(x0);
            launch_vert = launch_vert(1);
            land_vert = obj.lbsd.getClosestLandVerts(xf);
            land_vert = land_vert(1);
            [lane_ids, vert_ids, ~] = obj.lbsd.getShortestPath(...
                launch_vert, land_vert);
            waypoints_m = obj.lbsd.getVertPositions(vert_ids);
            
            dista = waypoints_m(1:end-1,:);
            distb = waypoints_m(2:end,:);
            dist = distb - dista;
            dist = sqrt(dist(:,1).^2 + dist(:,2).^2 + + dist(:,3).^2);
            dist = [0; dist];
            toa_s(1) = 0;
            toa_s(2) = dist(2) / obj.climb_rate;
            
            toa_s = [ toa_s, (dist(3:end-1) ./ obj.nominal_speed)'];
            toa_s(end+1) = dist(end) / obj.climb_rate;
            for i = 2:length(toa_s)
                toa_s(i) = toa_s(i) + toa_s(i-1);
            end
            ground_speed_ms = [0, ...
                obj.nominal_speed*ones(1,length(toa_s)-2), 0];
            climb_rate_ms = zeros(1,length(toa_s));
            
            traj = Trajectory(f_hz, waypoints_m, toa_s, ...
                ground_speed_ms, climb_rate_ms);
        end
       
        %% Event Handlers
        function handleGPS(obj, ~, ~)
            % handleGPS handle a GPS state update event. The updated gps
            % information is contained in the gps property of this object.
            % Call obj.gps to access the gps data.
            disp('GPS Received');
        end
    end
end

