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
        
    end
    methods (Static)
        function lbsd = LBSDSetup()
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
        function sim = SIMSetup()
            sim = Sim();
        end
        function uas = UASSetup(pos, id)
            uas = UAS(id);
            uas.gps.lat = pos(1);
            uas.gps.lon = pos(2);
            uas.gps.alt = pos(3);
        end
        
        function radar = RADARSetup(pos, range, angle, dir, ID, lbsd)
            radar = RADAR(pos, range, angle, dir, ID, lbsd);
        end
    end
    
    %% Create Data Structure Tests
    % This section is used to ensure that the data structures are being
    % created in the way they are meant to be designed.
    %
    % Data Structures to Test
    %   1. laneData - lane specific telemetry and sensory data
    %   2. sensoryData - stores sensory information from the simulation
    %   3. telemetryData - stores telemetry information from the simulation
    %   4. overallDensity - Stores the information about the overall
    %       density of the simulation, as well as figure and plot handles.
    
    methods(Test)
        function LaneDataSetUp(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_id = lbsd.getLaneIds;
            atoc = ATOC(lbsd);
            for lane = 1:size(lane_id, 1)
                testCase.verifyNotEmpty(atoc.laneData(lane_id(lane)));
            end
        end
        
        function SensorySetUp(testCase)
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            testCase.verifyNotEmpty(atoc.radars);
        end
        
        function TelemeterySetUp(testCase)
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            testCase.verifyNotEmpty(atoc.telemetry);
        end
        
        function OverallDensityStruct(testCase)
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            testCase.verifyNotEmpty(atoc.overallDensity);
        end
    end
    
    %% Update Data Structures Tests
    % This section is used to ensure that all the data structures are being
    % updated in the way they are intended to.
    %
    % Data Structures to Test
    %   1. laneData - lane specific telemetry and sensory data
    %   2. sensoryData - stores sensory information from the simulation
    %   3. telemetryData - stores telemetry information from the simulation
    %   4. overallDensity - Stores the information about the overall
    %       density of the simulation, as well as figure and plot handles.
    %   5. Time - keeps track of the time during the simulation
    %   6. lbsd - the Lane Base System handle
    methods(Test)
        function LaneDataUpdate(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            
            
        end
        
        function SensoryDataUpdate(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            radar = ATOCUnitTests.RADARSetup([0,0,0], 20, ...
                pi/4, [0,0,1], "1", lbsd);
            radar.time = 0;
            radar.subscribe_to_detection(@atoc.handle_events);
            atoc.time = 0;
            uas = ATOCUnitTests.UASSetup([30,30,15], "1");
            radar.scan(uas);
            numRows = height(atoc.radars);
            testCase.verifyEqual(1, numRows);
            uas.gps.lon = 0;
            uas.gps.lat = 0;
            uas.gps.alt = 10;
            radar.scan(uas);            
            numRows = height(atoc.radars);
            testCase.verifyEqual(2, numRows);
        end
        
        function TelemetryDataUpdate(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup([0,0,15], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.res_ids = "5";
            uas.gps.commit();
            testCase.verifyEqual("1", atoc.telemetry.ID(end));
            testCase.verifyEqual([0,0,15], atoc.telemetry.pos(end, :));
            testCase.verifyTrue(isequaln([2,4], size(atoc.telemetry)));
        end
        
        function DensityDataUpdate(testCase)
            sim = ATOCUnitTests.SIMSetup();
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            sim.subscribe_to_tick(@atoc.handle_events);
            sz = size(atoc.overallDensity.data);
            testCase.verifyEqual([1, 2], sz);
            sim.step(.3);
            sz = size(atoc.overallDensity.data);
            testCase.verifyEqual([2, 2], sz);
            sim.step(.1);
            sz = size(atoc.overallDensity.data);
            testCase.verifyEqual([3, 2], sz);
        end
        
        function TimingUpdate(testCase)
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            sim.step(.1);
            testCase.verifyEqual(.1, atoc.time);
            sim.step(1);
            testCase.verifyEqual(1.1, atoc.time);
        end
        
        function NewReservationUpdate(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            ok = false;
            while ~ok
                [ok, ~] = lbsd.makeReservation("2", ...
                randi(10), randi(100) + 50, 1, 5);
            end
            testCase.verifyEqual(atoc.lbsd.getNumReservations, ...
                lbsd.getNumReservations);
        end
        
        function ClearReservationsUpdate(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            lbsd.clearReservations();
            testCase.verifyEmpty(atoc.lbsd.getReservations);
        end
    end
    %% findClustering Function Tests
    % This section is used to test to ensure that the clustering function
    % is correctly clustering sensory data to telemetry data to ensure that
    % the correct number of UAS are currently flying.
    methods(Test)
    end
    %% Projection Function Tests
    % This section is used to test that the projection function is
    % correctly calculating the projection value from the planned flight
    % informaiton and the UAS telemetry information
    methods(Test)
    end
    %% Calculate Speed and Distance Function Tests
    % This section is used to test that the calculations for deviation in
    % speed and distance are correct
    %
    methods(Test)
    end
    %% Rogue Detection Tests
    % This section is used to test
    %
    methods(Test)
    end
end

