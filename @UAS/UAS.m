classdef UAS < handle
    % UAS An instance of this class represents a UAS agent
    %   Detailed explanation goes here
    
    properties
        id
        trajectory
        gps
        kb
        lbsd
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
        function planDelivery(obj, x0, xf)
            
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

