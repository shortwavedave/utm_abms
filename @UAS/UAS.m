classdef UAS < handle
    % UAS An instance of this class represents a UAS agent
    %   Detailed explanation goes here
    
    properties
        id
        a
        v
        x
    end
    
    methods
        function obj = UAS(id)
            %UAS Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = id;
            obj.initialize();
        end
        
        function initialize(obj)
            obj.a = [0 0 0]';
            obj.v = [1 1 1]';
            obj.x = [0 0 0]';
        end
        
        function reset(obj)
            obj.initialize();
        end
        
        function step(obj, del_t)
            %STEP Summary of this method goes here
            %   Detailed explanation goes here
            obj.x = obj.x + obj.v*del_t + obj.a*(del_t^2);
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

