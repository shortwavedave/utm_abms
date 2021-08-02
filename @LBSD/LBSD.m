classdef LBSD < handle
    %LBSD An instance of this class represents a Lane Based Strategic
    %   Deconfliction Supplemental Data Service Provider (SDSP).
    %   Detailed explanation goes here
    
    properties
        % A digraph representing the lane network
        lane_graph
    end
    
    methods
        function obj = LBSD()
            %LBSD Construct an instance of this class
            %   Detailed explanation goes here
        end

        function h = plot(obj)
            xdata = obj.lane_graph.Nodes.XData;
            ydata = obj.lane_graph.Nodes.YData;
            zdata = obj.lane_graph.Nodes.ZData;
            h = plot(obj.lane_graph,'XData',xdata,'YData',ydata, ...
                'ZData',zdata);
        end
    end
    
    methods (Static)
        lbsd = genSampleLanes(lane_length_m, altitude_m)
    end
end

