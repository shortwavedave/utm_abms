classdef SimConfig
    %SIMCONFIG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Seconds. The start of the simulation day
        t0 = 0.0
        % Seconds. The last time that a flight can be scheduled
        tf = 100.0
        % Fit a clothoid trajectory
        fit_traj = true;
        % Empty will enable random sampling of start and end vertexes. A
        % string indicates a single lane id that the simulation with happen
        % on
        single_lane
        en_morph = false
        morph
    end
    
    methods
        function obj = SimConfig()
            %SIMCONFIG Construct an instance of this class
            %   Detailed explanation goes here
        end
    end
end

