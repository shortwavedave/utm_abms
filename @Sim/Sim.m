classdef Sim < handle
    %SIM An instance of this class represents a simulation
    
    properties
        tick_del_t
        uas_list = []
        radar_list = []
        uas = []
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
            lane_length_m = 50;
            altitude_m = 100;
            obj.lbsd = LBSD.genSampleLanes(lane_length_m, altitude_m);
            
            %% Setup the ATOC
            obj.atoc = ATOC(obj.lbsd);
            
            %% Setup some UAS
            num_uas = 10;
            for i = 1:num_uas
                % Create a new UAS with a unique identifier
                new_uas = UAS(string(i));
                % Give this UAS a reference to the LBSD
                new_uas.lbsd = obj.lbsd;
                % Store a reference to this UAS in this simulation object
                obj.uas = [obj.uas, new_uas];
            end
            
%             %% Generate trajectories
%             [minx, miny, maxx, maxy] = lbsd.getEnvelope();
%             for i = 1:num_uas
%                 x0 = rand(
%             end
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

