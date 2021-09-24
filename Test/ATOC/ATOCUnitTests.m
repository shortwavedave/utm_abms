classdef ATOCUnitTests < matlab.unittest.TestCase
    %ATOCUNITTESTS - This is a unit testing class that is supposed to test
    %   the functions of the ATOC Class
    
    properties
        testToDisplay = "Testing :";
    end
    
    %% Test Setup and Take Down
    % This section is used to set up all the tests and breakdown all the
    % tests that are needed for the unit testing
    % 
    % Setup includes:
    %   Creation of the simulation object
    %   Creation of the lane system objects
    methods (TestClassSetup) % Setups before each method tests
        % Set up the simulation
        sim = Sim();
        
        % Set up the Lane System
        lbsd = LBSD.genSampleLanes(10, 15);
        start_time = 0;
        end_time = 100;
        lane_ids = ["1","2","3"];
        num_res = 50;
        speed = 1;
        headway = 5;
        lbsd.genRandReservations(start_time, end_time, ...
            num_res, lane_ids, speed, headway);
    end
    
    %% Create Data Structure Tests
    % This section is used to ensure that the data structures are being
    % created in the way they are meant to be designed.
    %
    % Data Structures to Test
    %   1. laneData - lane specific telemetry and sensory data
    %   2. sensoryData - stores sensory information from the simulation
    %   3. telemetryData - stores telemetry information from the simulation
    %   4. allDen - stores the overall density vs time throughout the 
    %       simulation
    %   5. denFig - figure handle to display density data
    %   6. p - plot handle to plot overall density versus time
    methods(Test)
    end
    %% Update Data Structures Tests
    %
    %
    methods(Test)
    end
    %% findClustering Function Tests
    methods(Test)
    end
    %% Projection Function Tests
    methods(Test)
    end
    %% Calculate Speed and Distance Function Tests
    methods(Test)
    end
    %% Rogue Detection Tests
    methods(Test)
    end
end

