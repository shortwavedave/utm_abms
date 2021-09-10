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
        climb_rate = 1.0
        % The nominal ground speed during flight in m/s
        nominal_speed = 1.0
        % The sample rate of set points along a planned trajectory in Hz
        set_point_hz = 1.0
        % Reservation IDs
        res_ids = []
    end
    
    events
        telemetry
    end
    
    methods
        function obj = UAS(id)
            %UAS Construct a UAS agent
            %   On Input:
            %       id - string A unique identifier for this agent
            if nargin < 1
                id = java.util.UUID.randomUUID.toString;
            end
            if ~isa(id, 'string')
                id = string(id);
            end
            obj.id = id;
            obj.initialize();
        end
        
        function reset(obj)
            % reset Reinitialize this object
            obj.initialize();
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
        
        %% Plans
        function traj = reserveLBSDTrajectory(obj, x0, xf)
            %planTrajectory Construct a trajectory between two locations.
            %	The locations may be any 2D locations and this method will
            %   find the closest land and launch vertexes in the lane
            %   system. This method reserves trajectories using the LBSD.
            %   On Input:
            %       x0 - 1x2 position in meters [x,y]
            %       xf - 1x2 position in meters [x,y]
            %   On Output:
            %       traj - Trajectory object
            
            if isempty(obj.lbsd)
                error(['This UAS has not been initialized with a LBSD ',...
                    'instance. Please set the lbsd property of this agent']);
            end
            f_hz = obj.set_point_hz;
            
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
            
            % Reserve lanes
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

