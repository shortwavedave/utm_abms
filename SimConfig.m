classdef SimConfig
    %SIMCONFIG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Seconds. The start of the simulation day
        t0 = 0.0
        % Seconds. The last time that a flight can be scheduled
        tf = 100.0
    end
    
    methods
        function obj = SimConfig()
            %SIMCONFIG Construct an instance of this class
            %   Detailed explanation goes here
        end
    end
end

