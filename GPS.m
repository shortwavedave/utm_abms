classdef GPS < handle
    %GPS An instance of this class represents a GPS sensor
    %   Unlike the PX4 simulator, this GPS sensor represents the filtered
    %   global position (normally a fused measurement of accelerometers and
    %   raw GPS data).
    
    properties
        % unsigned integer ms timestamp representing the time since this
        % sensor was powered on.
        time_ms = 0
        % float degrees WGS84 Latitude
        lat = 0.0  
        % float degrees WGS84 Longitude
        lon = 0.0
        % float m Altitude (MSL)
        alt = 0.0
        % float	m Altitude above ground
        relative_alt = 0.0
        % float	m/s Ground X Speed (Latitude, positive north)
        vx = 0.0
        % float	m/s Ground Y Speed (Longitude, positive east)
        vy = 0.0
        % float m/s Ground Z Speed (Altitude, positive down)
        vz = 0.0
        % float	degrees Vehicle heading (yaw angle), 0.0..359.99 degrees.
        % 0 represents true North, 90 represents true East.
        hdg = 0.0
    end
    
    properties (Access = private)
       state_listeners = [] 
    end
    
    events
        StateChange
    end
    
    methods
        function obj = GPS()
            %GPS Construct an instance of this class
        end
        
        function notifyStateChange(obj)
            % notifyStateChange Trigger the StateChange event and notify
            % all subscribers
            notify(obj, 'StateChange');
        end
        
        function subscribeToStateChange(obj, subscriber)
            % subscribeToStateChange Set an event listener to trigger when 
            %   a state change occurs.
            % On input
            %   obj - an instance of the GPS class
            %   subscriber - a function handle to trigger
            % Example:
            %   gps.subscribeToStateChange(@uas.updateState);
            lh = obj.addlistener('StateChange', subscriber);
            obj.state_listeners = [obj.state_listeners, lh];
        end
    end
end

