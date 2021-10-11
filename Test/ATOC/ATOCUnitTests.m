classdef ATOCUnitTests < matlab.unittest.TestCase
    %ATOCUNITTESTS - This is a unit testing class that is supposed to test
    %   the functions of the ATOC Class
     
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
            % Sets up a sim object
            sim = Sim();
        end
        function uas = UASSetup(pos, id)
            % Sets up the UAS Object
            uas = UAS(id);
            uas.gps.lat = pos(1);
            uas.gps.lon = pos(2);
            uas.gps.alt = pos(3);
        end
        function radar = RADARSetup(pos, range, angle, dir, ID, lbsd)
            % Sets up a RADAR object
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
        % LaneDataSetUp - checks to ensure that the lane data structure is
        %   set up correctly when ATOC object is created
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_id = lbsd.getLaneIds;
            atoc = ATOC(lbsd);
            for lane = 1:size(lane_id, 1)
                testCase.verifyNotEmpty(atoc.laneData(lane_id(lane)));
            end
        end
        function SensorySetUp(testCase)
        % SensorySetUP - Checks to Ensure that the sensory data structure
        %    is setup correctly when the ATOC Object is created
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            testCase.verifyNotEmpty(atoc.radars);
        end
        function TelemeterySetUp(testCase)
        % TelemeterySetUp - Checks to ensure that the telemetry data
        % structure is setup correctly when ATOC Object was created.
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            testCase.verifyNotEmpty(atoc.telemetry);
        end
        function OverallDensityStruct(testCase)
        % OverallDensityStruct - Checks to ensure that the overall density
        %   data structure is setup correctly when the ATOC object was
        %   created
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
        % LaneDataUpdate - Checks to see if the lane data structure is
        % updating when the telemetry data is being transmitting
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(2));
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_event);
            uas.res_ids = "3";
            uas.gps.commit();
            id = atoc.laneData(ids(2)).telemetry.ID;
            uasPos = atoc.laneData(ids(2)).telemetry.pos;
            testCase.verifyEqual(pos(1:3), uasPos);
            testCase.verifyEqual(id, "1");
            pos = [pos(1) + rand, pos(2) + rand, pos(3) + rand];
            uas.gps.lat = pos(1);
            uas.gps.lon = pos(2);
            uas.gps.alt = pos(3);
            uas.gps.commit();
            id = atoc.laneData(ids(2)).telemetry.ID;
            uasPos = atoc.laneData(ids(2)).telemetry.pos;
            testCase.verifyEqual(pos(1:3), uasPos);
            testCase.verifyEqual(id, "1");
        end
        function SensoryDataUpdate(testCase)
        % SensoryDataUpdate - Checks to see if the sensory information is
        %   updating correctly when the detection event is celled.
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
        % TelemetryDataUpdate - Checks to see if the telemetry data
        %   structure is updating correctly when telemetry data is sent
        %   through
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
        % DensityDatUpdate - Checks to ensure that the density data is
        % updating correctly when the tick function runs without any UAS
        % flying
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
        % TimingUpdate - Ensures that the time is changing with every step
        %   in the sim object.
            atoc = ATOC(ATOCUnitTests.LBSDSetup());
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            sim.step(.1);
            testCase.verifyEqual(.1, atoc.time);
            sim.step(1);
            testCase.verifyEqual(1.1, atoc.time);
        end
        function NewReservationUpdate(testCase)
        % NewReservationUpdate - Ensuring that the ATOC object's lbsd is
        %   updating when a new reservation is being made.
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            lbsd.subscribeToNewReservation(@atoc.handle_event)
            ok = false;
            while ~ok
                [ok, ~] = lbsd.makeReservation("2", ...
                    randi(10), randi(100) + 50, 1, 5);
            end
            testCase.verifyEqual(atoc.lbsd.getNumReservations, ...
                lbsd.getNumReservations);
        end
        function ClearReservationsUpdate(testCase)
        % ClearReservationsUpdate - Checks to see if the ATOC Object's lbsd
        %   is cleared when the reseverations are cleared in the LBSD Object.
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            lbsd.subscribeToNewReservation(@atoc.handle_event);
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
    
    %% Event Handling
    % This section of unit tests is to ensure that the event handling is
    %   working properly for a simulation
    
    methods(Test)
        function tickTimeHandling(testCase)
        % tickTimeHandling - ensures that the time is incrementing with
        % each step
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_event);
            atoc.time = 0;
            testCase.verifyEqual(0, atoc.time);
            sim.step(.1);
            testCase.verifyEqual(.1, atoc.time);
            sim.step(.3);
            testCase.verifyEqual(.4, atoc.time);
            sim.step(10);
            testCase.verifyEqual(1.4, atoc.time);
        end
        function tickClusteringHandling(testCase)
        % tickClusteringHandling - checks to see if the findClusters method
        %   is entered when the tick handling happens.
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_event);
            atoc.time = 0;
            sz = size(atoc.overallDensity.data);
            numUas = atoc.overallDensity.data(1);
            testCase.verifyEqual([1,2], sz);
            testCase.verifyEqual(0, numUas);
            sim.step(.1);
            sz = size(atoc.overallDensity.data);
            numUas = atoc.overallDensity.data(2);
            testCase.verifyEqual([2,2], sz);
            testCase.verifyEqual(0, numUas);
        end
        function telemetryInformationHandling(testCase)
        % telemetryHandling - Checks to ensure that telemetry data is
        % updating when the uas updates its position
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_event);
            uas.gps.commit();
            numRow = height(atoc.telemetry);
            testCase.verifyEqual(2, numRow);
            uas.gps.lat = pos(1) + rand();
            uas.gps.lon = pos(2) + rand();
            uas.gps.alt = pos(3) + rand();
            uas.gps.commit();
            numRow = height(atoc.telemetry);
            testCase.verifyEqual(3, numRow);
        end
        function telemteryLaneDataHandling(testCase)
        % telemetryLaneDataHandling - Checks to ensure that the Lane Data
        % is updating when the uas updates its positions
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_event);
            uas.res_ids = "3";
            uas.gps.commit();
            data = atoc.laneData(ids(1)).telemetry;
            sz = height(data);
            testCase.verifyEqual(2, sz(1));
            uas.gps.commit();
            data = atoc.laneData(ids(1)).telemetry;
            sz = height(data);
            testCase.verifyEqual(3, sz(1));
        end
    end
end

