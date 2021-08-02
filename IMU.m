classdef IMU < handle
    %IMU An object of this class describes an Intertial Measurement Unit
    %   All units are specified in the North-East-Down (NED) coordinate
    %   frame.
    
    properties
        % unsigned integer us timestamp representing the time since this
        % sensor was powered on.
        time_usec = 0
        % float m/s/s X Acceleration
        xacc = 0.0  
        % float m/s/s Y Acceleration
        yacc = 0.0
        % float m/s/s Z Acceleration
        zacc = 0.0
        % float	rad/s Angular speed around X axis in body frame
        xgyro = 0.0
        % float	rad/s Angular speed around Y axis in body frame
        ygyro = 0.0
        % float	rad/s Angular speed around Z axis in body frame
        zgyro = 0.0
        % float	gauss X Magnetic field
        xmag = 0.0
        % float	gauss Y Magnetic field
        ymag = 0.0
        % float	gauss Z Magnetic field
        zmag = 0.0
        % float	hPa	Absolute pressure
        abs_pressure = 0.0
        % float	hPa	Differential pressure (airspeed)
        diff_pressure = 0.0
        % float	Altitude calculated from pressure
        pressure_alt = 0.0
        % float	degC Temperature
        temperature	= 0.0
    end
    
    events
        StateChange
    end
    
    properties (Access = private)
       state_listeners = [] 
    end
    
    methods
        function obj = IMU()
            %IMU Construct an instance of this class
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
            %   obj - an instance of the IMU class
            %   subscriber - a function handle to trigger
            % Example:
            %   sim.subscribeToStateChange(@uas.updateState);
            lh = obj.addlistener('StateChange', subscriber);
            obj.state_listeners = [obj.state_listeners, lh];
        end
    end
end

