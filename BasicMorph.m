classdef BasicMorph < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lbsd
        endpoints
        nodes
        toa = 10
        t = 0
    end
    
    methods
        function obj = BasicMorph(lbsd)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.lbsd = lbsd;
        end
        
        function done = morph(obj, step_t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if step_t == 0
                obj.lbsd.lane_graph.Nodes.ZData(obj.nodes) = obj.endpoints;
                done = true;
            else
                z = obj.lbsd.lane_graph.Nodes.ZData(obj.nodes);
                t_left = obj.toa - obj.t;
                s = abs(z - obj.endpoints) / t_left;
                %TODO account for step_t > obj.toa
                z = z + s*step_t;
                obj.lbsd.lane_graph.Nodes.ZData(obj.nodes) = z;
                d = abs(z - obj.endpoints);
                if all(d < 1e-7)
                    done = true;
                end
            end
        end
    end
end

