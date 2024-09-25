classdef BasicMorph < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lbsd
        endpoints
        nodes
        toa = 10
        t = 0
        K = 0.004
        max_rate = 0
        min_rate = 0
        record_node
        node_positions = []
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
            done = false;
            z = obj.lbsd.lane_graph.Nodes.ZData(obj.nodes);
            d = norm(z - obj.endpoints);
            if all(d < 1e-7)
                done = true;
            end
            
            if step_t == 0
                obj.lbsd.lane_graph.Nodes.ZData(obj.nodes) = obj.endpoints;
                done = true;
            elseif ~done
                obj.t = obj.t + step_t;
%                 t_left = obj.toa - obj.t;
                s = (obj.endpoints' - z) * obj.K;
                rates = s/step_t;
                min_rate = min(rates);
                max_rate = max(rates);
                if min_rate < obj.min_rate
                    obj.min_rate = min_rate;
                end
                if max_rate > obj.max_rate
                    obj.max_rate = max_rate;
                end
                
                %TODO account for step_t > obj.toa
                z = z + s;
                obj.lbsd.lane_graph.Nodes.ZData(obj.nodes) = z;
                
                if ~isempty(obj.record_node)
                    positions = obj.lbsd.getVertPositions(obj.record_node);
                    obj.node_positions = [obj.node_positions; obj.t, positions];
                end
            end
        end
    end
end

