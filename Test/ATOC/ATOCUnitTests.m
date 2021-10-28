classdef ATOCUnitTests < matlab.unittest.TestCase
    %ATOCUNITTESTS - This is a unit testing class that is supposed to test
    %   the functions of the ATOC Class
    
    %% Helper Methods
    % This section uses static methods to help with the creatation of
    % different objects to assist in the testing below. These objects
    % include the simulation, lbsd, radar, and uas objects.
    
    methods (Static)
        function lbsd = LBSDSetup()
            % Set up the Lane System
            lbsd = LBSD.genSampleLanes(10, 15);
        end
        function sim = SIMSetup()
            % Sets up a sim object
            sim = Sim();
        end
        function uas = UASSetup(pos, id)
            % Sets up the UAS Object
            uas = UAS(id);
            uas.gps.lon = pos(1);
            uas.gps.lat = pos(2);
            uas.gps.alt = pos(3);
        end
        function radar = RADARSetup(pos, range, angle, dir, ID, lbsd)
            % Sets up a RADAR object
            radar = RADAR(pos, range, angle, dir, ID, lbsd);
        end
        function lbsd = SpecificLBSDReservationSetup(lbsd, lane_id, entry_time, ...
                exit_time, speed, hd, uas_id)
            % SpecificLBSDReservationSetup - Creates an LBSD object with a
            %       specifc reseveration for unit tests
            lbsd.makeReservation(lane_id, entry_time, exit_time, speed, ...
                hd, uas_id);
        end
        function proj = ProjectionCalculation(Actual, Planned)
            % ProjectionCalculation - A Helper method that calculates the
            %    projection of the Actual onto the Planned vector.
            dotProduct = dot(Actual, Planned);
            normPlanned = norm(Planned)^2;
            proj = (dotProduct/normPlanned)*Planned;
            proj = norm(Planned - proj);
            if(isnan(proj))
                proj = 0;
            end
        end
        function dis = CalculateDistanceDifference(UASpos, Plannedpos)
            % CalculateDistanceDifference - A helper method that calculates the
            % change in distance from actual versus the planned distance.
            dis = norm(Plannedpos - UASpos);
        end
        function speed = CalculateSpeedDifference(prevUasPos, curUasPos,...
                prevPlannedPos, curPlannedPos, del_time)
            % CalculateSpeedDifference - A helper method that calculates the
            % speed difference from the planned speed and the actual speed.
            speedUAS = norm(curUasPos - prevUasPos)/del_time;
            speedPlan = norm(curPlannedPos - prevPlannedPos)/del_time;
            speed = speedUAS - speedPlan;
        end
        function lane_ids = FindLargestDistance(lbsd)
            ids = lbsd.getLaneIds();
            lane_ids = [ids(1), ids(2)];
            start1 = lbsd.getVertPositions(lbsd.getLaneVertexes(ids(1)));
            end1 = lbsd.getVertPositions(lbsd.getLaneVertexes(ids(2)));
            dis = norm(start1(1, 1:3) - end1(1,1:3));
            for id = 3:length(ids)
                pos = lbsd.getVertPositions(lbsd.getLaneVertexes(ids(id)));
                dis2 = norm(start1(1, 1:3) - pos(1, 1:3));
                if(dis < dis2)
                    dis = dis2;
                    lane_ids(2) = ids(id);
                end
            end
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
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            ids = res.lane_id;
            vertid = lbsd.getLaneVertexes(ids);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.res_ids = res.id;
            uas.gps.commit();
            id = atoc.laneData(ids).telemetry.ID;
            uasPos = atoc.laneData(ids).telemetry.pos(end, :);
            testCase.verifyEqual(pos(1, 1:3), uasPos, "AbsTol",1);
            testCase.verifyEqual(id(end), "1");
            pos = [pos(1,1) + rand, pos(1,2) + rand, pos(1,3) + rand];
            uas.gps.lon = pos(1);
            uas.gps.lat = pos(2);
            uas.gps.alt = pos(3);
            uas.gps.commit();
            id = atoc.laneData(ids).telemetry.ID;
            uasPos = atoc.laneData(ids).telemetry.pos(end, :);
            testCase.verifyEqual(pos(1, 1:3), uasPos, "AbsTol",1);
            testCase.verifyEqual(id(end), "1");
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
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, "1", ...
                0, 10, 1, 5, "1");
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup([0,0,15], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);
            res = lbsd.getLatestRes();
            uas.res_ids = res.id;
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
            lbsd.subscribeToNewReservation(@atoc.handle_events)
            ok = false;
            while ~ok
                [ok, ~] = lbsd.makeReservation("2", ...
                    randi(10), randi(100) + 50, 1, 5, "1");
            end
            testCase.verifyEqual(atoc.lbsd.getNumReservations, ...
                lbsd.getNumReservations);
        end
        function ClearReservationsUpdate(testCase)
            % ClearReservationsUpdate - Checks to see if the ATOC Object's lbsd
            %   is cleared when the reseverations are cleared in the LBSD Object.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, "1", ...
                0, 10, 1, 5, "1");
            atoc = ATOC(lbsd);
            lbsd.subscribeToNewReservation(@atoc.handle_events);
            lbsd.clearReservations();
            testCase.verifyEmpty(atoc.lbsd.getReservations);
        end
    end
    %% findClustering Function Tests
    % This section is used to test to ensure that the clustering function
    % is correctly clustering sensory data to telemetry data to ensure that
    % the correct number of UAS are currently flying.
    %
    % Clustering Data is stored in the overallDensity data structure in
    % ATOC.
    
    methods(Test) % Small/no Amount UAS No Radar Tests
        function NoUASOneStepClusterTest(testCase)
            % NoUASClusterTest - Test to ensure that No clusters
            %   are found in one step.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            sim.step(1);
            density = atoc.overallDensity.data;
            testCase.verifyEqual(0, density(end, 1));
            testCase.verifyEqual(0, density(end, 2));
        end
        function NoUASMultipleSmallStepsClusterTest(testCase)
            % NoUASMultipleStepsClusterTests - Tests to ensure that no uas is
            %   detected with multiple steps as time is incrementing.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            counter = 0;
            while counter < 100
                sim.step(.1);
                density = atoc.overallDensity.data;
                testCase.verifyEqual(0, density(end, 2));
                counter = counter + 1;
            end
        end
        function NoUASMultipleLargeStepsClusterTest(testCase)
            % NoUASMultipleLargeStepsClusterTest - Tests to see if no uas is
            % detected over larger time steps.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            counter = 0;
            while counter < 100
                sim.step(.1);
                density = atoc.overallDensity.data;
                testCase.verifyEqual(0, density(end, 2));
                counter = counter + 1;
            end
        end
        function SingleUASOneStepFlight(testCase)
            % SingleUASOneStepFlight - Tests the clustering method only picks
            % up a single uas after one step.
            % Setting up LBSD
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc.time = res.entry_time_s;
            
            % Setting up UAS
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            sim.step(.1);
            
            % Check that it should be 1
            density = atoc.overallDensity.data;
            testCase.verifyEqual(1, density(end, 2));
        end
        function SingleStepUASMultipleTime(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            sim.step(.1);
            
            % Setting up Reservation
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc.time = res.entry_time_s;
            
            % Setting up UAS
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            
            sim.step(.3);
            
            % Check that it should be 1
            density = atoc.overallDensity.data;
            testCase.verifyEqual(1, density(end, 2));
        end
        function TwoStepUASMultipleTime(testCase)
            % TwoStepUASMultipleTime - Tests whether two steps with a single
            % uas in flight to see if the clustering method only picks up one
            % UAS.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc.time = res.entry_time_s;
            
            % Setting up UAS
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            
            sim.step(.1);
            
            % Check that it should be 1
            density = atoc.overallDensity.data;
            testCase.verifyEqual(1, density(end, 2));
            
            % Step One More Time
            uas.gps.lon = uas.gps.lon + rand();
            uas.gps.lat = uas.gps.lat + rand();
            uas.gps.alt = uas.gps.alt + rand();
            uas.gps.commit();
            
            % Take a step
            sim.step(.3);
            
            % Still should be 1
            density = atoc.overallDensity.data;
            testCase.verifyEqual(1, density(end, 2));
        end
        function MultipleStepUASVaryingTime(testCase)
            % MultipleStepUASVaryingTime - Tests to see that the clustering
            % method only picks up a single uas through varying time steps
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservations
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc.time = res.entry_time_s;
            
            % Setting up UAS
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            sim.step(abs(rand()) + .01);
            
            counter = 0;
            while counter < 10
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.alt = uas.gps.alt + rand();
                uas.gps.commit();
                
                sim.step(abs(rand()) + .01);
                density = atoc.overallDensity.data;
                testCase.verifyEqual(1, density(end, 2));
                counter = counter + 1;
            end
        end
        function TwoUASNoStepFarAway(testCase)
            % TwoUASNoStepFarAway - Tests the clustering method for
            %   Two UAS placed in the airway without taking a step.
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = ATOCUnitTests.FindLargestDistance(lbsd);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            res1 = lbsd.getLatestRes();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(2), 0, 10, 1, 5, "2");
            res2 = lbsd.getLatestRes();
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            
            atoc.time = res1.entry_time_s;
            
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            % Call A time
            sim.step(1);
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoUASSingleStepFarAway(testCase)
            % TwoUASSingleStepFarAway - Tests to see if the clustering method
            % correctly indicates two clusters far away.
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = ATOCUnitTests.FindLargestDistance(lbsd);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(2), 0, 10, 1, 5, "2");
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            res2 = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            % Call A time
            sim.step(1);
            
            uas1.gps.lon = uas1.gps.lon + rand();
            uas1.gps.lat = uas1.gps.lat + rand();
            uas1.gps.alt = uas1.gps.alt + rand();
            uas1.gps.commit();
            
            uas2.gps.lon = uas2.gps.lon + rand();
            uas2.gps.lat = uas2.gps.lat + rand();
            uas2.gps.alt = uas2.gps.alt + rand();
            uas2.gps.commit();
            
            sim.step(1);
            
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoUASTwoStepsFarAway(testCase)
            % TwoUASTwoStepsFarAway - Tests that the clustering method works to
            % idenitfy two individual clusters when two uas steps twice.
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = ATOCUnitTests.FindLargestDistance(lbsd);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(2), 0, 10, 1, 5, "2");
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            res2 = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            % Call A time
            sim.step(1);
            
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
            
            uas1.gps.lon = uas1.gps.lon + rand();
            uas1.gps.lat = uas1.gps.lat + rand();
            uas1.gps.alt = uas1.gps.alt + rand();
            uas1.gps.commit();
            
            uas2.gps.lon = uas2.gps.lon + rand();
            uas2.gps.lat = uas2.gps.lat + rand();
            uas2.gps.alt = uas2.gps.alt + rand();
            uas2.gps.commit();
            
            sim.step(1);
            
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
            
            uas1.gps.lon = uas1.gps.lon + rand();
            uas1.gps.lat = uas1.gps.lat + rand();
            uas1.gps.alt = uas1.gps.alt + rand();
            uas1.gps.commit();
            
            uas2.gps.lon = uas2.gps.lon + rand();
            uas2.gps.lat = uas2.gps.lat + rand();
            uas2.gps.alt = uas2.gps.alt + rand();
            uas2.gps.commit();
            
            sim.step(1);
            
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoUASMultipleStepsFarAway(testCase)
            % TwoUASMultipleStepsFarAway - Tests that the clustering methods
            % over multiple steps only pick up two uas
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = ATOCUnitTests.FindLargestDistance(lbsd);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(2), 0, 10, 1, 5, "2");
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            res2 = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            sim.step(.1)
            
            counter = 0;
            while counter < 10
                uas1.gps.lon = uas1.gps.lon + rand();
                uas1.gps.lat = uas1.gps.lat + rand();
                uas1.gps.alt = uas1.gps.alt + rand();
                uas1.gps.commit();
                
                uas2.gps.lon = uas2.gps.lon + rand();
                uas2.gps.lat = uas2.gps.lat + rand();
                uas2.gps.alt = uas2.gps.alt + rand();
                uas2.gps.commit();
                
                sim.step(abs(rand()) + .001);
                
                density = atoc.overallDensity.data;
                testCase.verifyEqual(2, density(end, 2));
                
                counter = counter + 1;
            end
        end
        function TwoMiddleNoStepsClusterSteps(testCase)
            % TwoMiddleNoStepsClusterSteps - Tests the clustering method when
            % two uas are in differing lanes, but are some distance apart from
            % one another.
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = lbsd.getLaneIds();
            sizeId = length(lane_ids);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(sizeId/2), 0, 10, 1, 5, "2");
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            res2 = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            sim.step(.1)
            
            % Testing
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoMiddleSingleStepsClusterSteps(testCase)
            % TwoMiddleSingleStepsClusterSteps - Tests the cluster method when
            % two uas are moving through the lane system only 1 single step
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = lbsd.getLaneIds();
            sizeId = length(lane_ids);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(sizeId/2), 0, 10, 1, 5, "2");
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            res2 = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            sim.step(.1)
            
            uas1.gps.lon = uas1.gps.lon + rand();
            uas1.gps.lat = uas1.gps.lat + rand();
            uas1.gps.alt = uas1.gps.alt + rand();
            uas1.gps.commit();
            
            uas2.gps.lon = uas2.gps.lon + rand();
            uas2.gps.lat = uas2.gps.lat + rand();
            uas2.gps.alt = uas2.gps.alt + rand();
            uas2.gps.commit();
            
            sim.step(.2)
            
            % Testing
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoMiddleTwoStepsClusteringSteps(testCase)
            % TwoMiddleTwoStepsClusteringSteps - Tests that the clustering
            % method only picks up two uas flying in the lane system only two
            % steps taken
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = lbsd.getLaneIds();
            sizeId = length(lane_ids);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(sizeId/2), 0, 10, 1, 5, "2");
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            res2 = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            sim.step(.1)
            
            uas1.gps.lon = uas1.gps.lon + rand();
            uas1.gps.lat = uas1.gps.lat + rand();
            uas1.gps.alt = uas1.gps.alt + rand();
            uas1.gps.commit();
            
            uas2.gps.lon = uas2.gps.lon + rand();
            uas2.gps.lat = uas2.gps.lat + rand();
            uas2.gps.alt = uas2.gps.alt + rand();
            uas2.gps.commit();
            
            sim.step(.1)
            
            % Testing
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoMiddleStepsMultipleSteps(testCase)
            % TwoMiddleStepsMultipleSteps - Tests that the clustering method
            % only detects 2 uas over multiple steps.
            lbsd = ATOCUnitTests.LBSDSetup();
            lane_ids = lbsd.getLaneIds();
            sizeId = length(lane_ids);
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(1), 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                lane_ids(sizeId/2), 0, 10, 1, 5, "2");
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            res2 = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res2.lane_id);
            pos2 = lbsd.getVertPositions(vertid);
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos2(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            sim.step(.001);
            counter = 0;
            
            while counter < 10
                uas1.gps.lon = uas1.gps.lon + rand();
                uas1.gps.lat = uas1.gps.lat + rand();
                uas1.gps.alt = uas1.gps.alt + rand();
                uas1.gps.commit();
                
                uas2.gps.lon = uas2.gps.lon + rand();
                uas2.gps.lat = uas2.gps.lat + rand();
                uas2.gps.alt = uas2.gps.alt + rand();
                uas2.gps.commit();
                
                sim.step(abs(rand()) + .001);
                
                % Testing
                density = atoc.overallDensity.data;
                testCase.verifyEqual(2, density(end, 2));
                counter = counter + 1;
            end
        end
        function TwoUASCloseSingleTogetherSteps(testCase)
            % TwoUASCloseSingleTogetherSteps - tests the clustering methods
            % that it indicates two groups/one group for very close uas groups.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "2");
            
            % Setting Up ATOC
            res2 = lbsd.getLatestRes();
            
            start1 = pos(1, 1:3) + [2, 0, 0];
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(start1, res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            % Call A time
            sim.step(1);
            
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoUASCloseTwoStepsClusterTogether(testCase)
            % TwoUASCloseTwoStepsClusteringTogether - Tests the clustering
            % method having two uas flying in the same lane moving two steps.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "2");
            res2 = lbsd.getLatestRes();
            
            start1 = pos(1, 1:3) + [randi(5), 0, 0];
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(start1, res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            % Call A time
            sim.step(1);
            
            uas1.gps.lon = uas1.gps.lon + rand();
            uas1.gps.lat = uas1.gps.lat + rand();
            uas1.gps.alt = uas1.gps.alt + rand();
            uas1.gps.commit();
            
            uas2.gps.lon = uas2.gps.lon + rand();
            uas2.gps.lat = uas2.gps.lat + rand();
            uas2.gps.alt = uas2.gps.alt + rand();
            uas2.gps.commit();
            
            sim.step(1);
            
            density = atoc.overallDensity.data;
            testCase.verifyEqual(2, density(end, 2));
        end
        function TwoUASCloseMultipleStepsClusteringTogether(testCase)
            % TwoUASCloseMultipleStepsClusteringTogether - Test the clustering
            % method that the two uas close together over multiple different
            % steps.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            
            % Setting Up ATOC
            atoc = ATOC(lbsd);
            
            % Setting Up Sim
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Setting up Reservation
            res1 = lbsd.getLatestRes();
            atoc.time = res1.entry_time_s;
            vertid = lbsd.getLaneVertexes(res1.lane_id);
            pos = lbsd.getVertPositions(vertid);
            
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "2");
            res2 = lbsd.getLatestRes();
            
            start1 = pos(1, 1:3) + [randi(5), 0, 0];
            
            % UAS - 1
            uas1 = ATOCUnitTests.UASSetup(start1, res1.uas_id);
            uas1.res_ids = res1.id;
            uas1.subscribeToTelemetry(@atoc.handle_events);
            uas1.gps.commit();
            
            % UAS - 2
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), res2.uas_id);
            uas2.res_ids = res2.id;
            uas2.subscribeToTelemetry(@atoc.handle_events);
            uas2.gps.commit();
            
            % Call A time
            sim.step(1);
            
            counter = 0;
            while counter < 10
                uas1.gps.lon = uas1.gps.lon + rand();
                uas1.gps.lat = uas1.gps.lat + rand();
                uas1.gps.alt = uas1.gps.alt + rand();
                uas1.gps.commit();
                
                uas2.gps.lon = uas2.gps.lon + rand();
                uas2.gps.lat = uas2.gps.lat + rand();
                uas2.gps.alt = uas2.gps.alt + rand();
                uas2.gps.commit();
                
                sim.step(abs(rand()) + .001);
                
                density = atoc.overallDensity.data;
                testCase.verifyEqual(2, density(end, 2));
                
                counter = counter + 1;
            end
        end
    end
    methods(Test) % Multiple UAS No Radar Tests - Stress Tests
        function EnteringIntoOneLaneMultipleClusters(testCase)
            % EnteringINtoOneLaneMultipleClusters - stress tests the cluster
            % method that continually adds uas into the same lane over a varity
            % of time.
            
            % Creation of Sim/atoc/lbsd/uas
            sim = ATOCUnitTests.SIMSetup();
            lbsd = ATOCUnitTests.LBSDSetup();
            numUas = 100;
            list_uas = [];
            for plan = 0:numUas
                startTime = plan*2;
                endTime = startTime + 10;
                lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", startTime, endTime, 1, 5, num2str(plan));
                pos = lbsd.getVertPositions(lbsd.getLaneVertexes("1"));
                uas = ATOCUnitTests.UASSetup(pos(1, 1:3), num2str(plan));
                uas.res_ids = "1";
                s = struct("UAS", uas,...
                    "StartTime", startTime, "EndTime", endTime);
                list_uas = [list_uas; s];
            end
            atoc = ATOC(lbsd);
            sim.subscribe_to_tick(@atoc.handle_events);
           
            % Get the position starting point of the uas
            
            % Subscribe uas to atoc
            atoc.time = 0;
            for index = 1:numUas
                uas = list_uas(index).UAS;
                uas.subscribeToTelemetry(@atoc.handle_events);
            end
            
            % Actual testing class
            
            endTime = list_uas(numUas).EndTime;
            while (atoc.time < endTime) % Run the simulation
                inFlight = 0;
                removeIndex = [];
                for drone = 1:size(list_uas)
                    flight = list_uas(drone);
                    % Done flying - remove
                    if(flight.EndTime < atoc.time)
                        removeIndex = [removeIndex; drone];
                    % Start flight - continue flight
                    elseif(flight.StartTime <= atoc.time)
                        inFlight = inFlight + 1;
                        uas = flight.UAS;
                        uas.gps.lon = uas.gps.lon + abs(randi(3));
                        uas.gps.lat = uas.gps.lat + abs(randi(3));
                        uas.gps.alt = uas.gps.alt + rand();
                        uas.gps.commit();
                    end
                end
                list_uas(removeIndex) = [];
                sim.step(2);
                density = atoc.overallDensity.data;
                if inFlight < 0
                    inFlight = 0;
                end
                testCase.verifyEqual(inFlight, density(end, 2));
            end

        end
        function EnteringDifferentLanesMultipleClusters(testCase)
            % EnteringDifferentLanesMultipleClusters - Stress tests the cluster
            % method that continually adds uas into multiple different lanes
            % over a varity of time.
            % Set up ATOC/SIM/LBSD/UAS array/num Array
            
            % while uas [] not empty
            % If UAS pos is 2 m away add new uas into the same lane
            % If numUAS < total UAS
            % Create New Res (random lane)
            % Add to uas struct
            % Loop through UAS
            % If time > end time || cur pos ~= end pos
            % Remove from uas struct
            % else
            % Increment position
            % Sim.step
            % Test Case
            
        end
    end
    
    methods(Test) % Single UAS With Radar Tests
    end
    
    methods(Test) % Multiple UAS With Radar Tests
    end
    %% Projection Function Tests
    % This section is used to test that the projection function is
    % correctly calculating the projection value from the planned flight
    % informaiton and the UAS telemetry information
    methods(Test)
        function startingProjectTest(testCase)
            % startingProjectTest - Testing that when the UAS is on track at
            % the beginning of the lane the projection is zero.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            telemetry = atoc.laneData(res.lane_id).telemetry;
            projection = telemetry.projection(end);
            testCase.verifyEqual(projection, 0);
        end
        function noDeviationProjectionTest(testCase)
            % noDeviationProjectionTest - Testing that if the UAS is following
            %   the desirable path - the projection should be zero or close to
            %   zero.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            while atoc.time < endTime
                % Submit the new position
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                
                % Testing if the projection is equal to zero
                testCase.verifyEqual(telemetry.projection(end), 0, ...
                    "AbsTol",0.001);
                
                % Calculate the new position
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
            end
        end
        function OneSetXDeviationProjectionTest(testCase)
            % OneSetXDeviationProjectionTest - Checks one step deviation in the
            %   x direction.
            % SetUp Lane Base System/UAS
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ro = [0,0,0] + (atoc.time - startTime)*dirVector;
            ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lon = ri(1) + 1;
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            laneToUas = [uas.gps.lon, uas.gps.lat, uas.gps.alt] - pos(1, 1:3);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation(laneToUas, ro), ...
                "AbsTol", .001);
            
        end
        function MultipleXResetEachStepSetsDeviationProjectionTest(testCase)
            % MultipleXResetEachStepSetsDeviationProjectionTest - Checks over a longer
            %   period of time of moving x direction deviation of some randi
            % 	distribution
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                proj = ATOCUnitTests.ProjectionCalculation(uasDif, ro);
                testCase.verifyEqual(telemetry.projection(end), ...
                    proj, "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1) + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
            end
        end
        function ContinualMultipleXStepDeviationProjectionTest(testCase)
            % ContinualMultipleXStepDeviationProjectionTest - Checks to see if
            %   the projection is correct as the x direction deviation with each
            %   step without reseting it back to the planned route with each
            %   step.
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = uas.gps.lat + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
            end
        end
        function OneSetYDeviationProjectionTest(testCase)
            % OneSetYDeviationProjectionTest - Checks to see if the projection
            %   helper function is working when one step deviation in the Y
            %   direction
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + rand();
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            ro = [0,0,0] + (atoc.time - startTime)*dirVector;
            uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                - pos(1, 1:3);
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                "AbsTol", .001);
        end
        function MultipleYResetStepDeviationProjectionTest(testCase)
            % MultipleXResetEachStepSetsDeviationProjectionTest - Checks over a longer
            %   period of time of moving x direction deviation of some randi
            % 	distribution
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2) + rand();
                uas.gps.alt = ri(3);
            end
        end
        function MultipleYNoResetStepDeviationProjectionTest(testCase)
            % MultipleYNoResetStepDeviationProjectionTest - Checks to see if
            %   the helper projection method calculates the project when the UAS
            %   is deviating from the path in the y direction.
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1);
                uas.gps.lat = uas.gps.lon + rand();
                uas.gps.alt = ri(3);
            end
        end
        function OneSetZDeviationProjectionProjectionTest(testCase)
            % OneSetZDeviationProjectionProjectionTest - Checks to see if one
            %   step in the z direction that is deviated from the planned path is
            %   calculated correctly in the helper projection method.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            ro = [0,0,0] + (atoc.time - startTime)*dirVector;
            uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                - pos(1, 1:3);
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                "AbsTol", .001);
        end
        function MultipleZResetStepDeviationProjectionTest(testCase)
            % MultipleZResetStepDeviationProjectionTest - Checks a longer trial
            %   run of reseting the z direction based on the planned path and
            %   then add some noise deviation.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleZNoResetStepDeviationProjectionTest(testCase)
            % MultipleZNoResetStepDeviationProjectionTest - Checks to see if
            %   the projection helper method correctly calculates the deviation
            %   when the Z direction continues to further separate from the
            %   planned path.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
        function OneStepXYResetStepDeviationProjectionTest(testCase)
            % OneStepXYResetStepDeviationProjectionTest - Checks to see if the
            % projection method works when calculating the projection of actual
            % onto planned when x & y direction has deviations, reseting with
            % each step.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lon = ri(1)  + rand();
            uas.gps.lat = ri(2)  + rand();
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            ro = [0,0,0] + (atoc.time - startTime)*dirVector;
            uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                - pos(1, 1:3);
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                "AbsTol", .001);
        end
        function MultipleXYResetStepDeviationProjectionTest(testCase)
            % MultipleXYResetStepDeviationProjectionTest - Checks to see if the
            %   projection helper method correctly projects the acutal onto the
            %   planned when continually deviating in the XY direction while
            %   resting with each step.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1) + rand();
                uas.gps.lat = ri(2) + rand();
                uas.gps.alt = ri(3);
            end
        end
        function MultipleXYNoResetStepDeviationProjectionTest(testCase)
            % MultipleXYNoResetStepDeviationProjectionTest - Checks to see if
            % the projection method works when calculating the projection of
            % actual onto planned when x and y direction continually deviate.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = uas.gps.lat + rand();
                uas.gps.lat = uas.gps.lon + rand();
                uas.gps.alt = ri(3);
            end
        end
        function OneStepXZDeviationProjectionTest(testCase)
            % OneStepXZResetStepDeviationProjectionTest - Checks to see fi the
            % projection method works with the x and z deviates from the
            % planned path after one step
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lon = ri(1) + rand();
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            ro = [0,0,0] + (atoc.time - startTime)*dirVector;
            uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                - pos(1, 1:3);
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                "AbsTol", .001);
        end
        function MultipleXZRestStepDeviationProjectionTest(testCase)
            % MultipleXZRestStepDeviationProjectionTest - Checks to see fi the
            % projection methods works when calculating the projection of the
            % acutal onto planned in the xz direction over a longer period of
            % time, resting with each step.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1) + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleXZNoResetDeviationProjectionTest(testCase)
            % MultipleXZNoResetDeviationProjectionTest - Checks to see if the
            % projection methods works when calculating the projection fo the
            % actual onto the planned with continually deviations in the x and
            % z direction over a longer period of time.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = uas.gps.lat + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
        function OneStepYZDeviationProjectionTest(testCase)
            % OneStepYZDeviationProjectionTest - Checks to see if the
            % projection method works when calculating the projection of the
            % acutal onto the planned with one step deviation in the y and z
            % direction.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + rand();
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            ro = [0,0,0] + (atoc.time - startTime)*dirVector;
            uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                - pos(1, 1:3);
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                "AbsTol", .001);
        end
        function MultipleYZResetStepDeviationProjectionTest(testCase)
            % MultipleYZResetStepDeviationProjectionTest - Checks to see fi the
            % projection methods works when calculating the projection of the
            % actual onto the planned with deviations of Y and Z direction,
            % resting after each step.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2) + rand();
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleYZNoResetStepDeviationProjectionTest(testCase)
            % MultipleYZNoResetStepDeviationProjectionTest - Checks to see if
            % the projection methods work when calculating the projection of
            % the actual onto the planned with deviations of the Y and Z
            % continually deviating form the planned flight.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1);
                uas.gps.lat = uas.gps.lon + rand();
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
        function OneStepAllDeviationProjectionTest(testCase)
            % OneStepAllDeviationProjectionTest - checks to see if the
            %   projection helper works when the deviation from the planned path
            %   is in all directions.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lon = ri(1) + rand();
            uas.gps.lat = ri(2) + rand();
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            ro = [0,0,0] + (atoc.time - startTime)*dirVector;
            uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                - pos(1, 1:3);
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                "AbsTol", .001);
        end
        function MultipleStepAllResetDeviationProjection(testCase)
            % MultipleStepAllResetDeviationProjection - Checks to see if the
            %   projection methods works when the deviation in all directions
            %   over a longer period of time works, with each step reseting back
            %   to the planned path and then added deviation.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lon = ri(1) + rand();
                uas.gps.lat = ri(2) + rand();
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleStepAllNoResetDeviationProjection(testCase)
            % MultipleStepAllNoResetDeviationProjection - Checks to see if the
            %   projection helper methods correctly projects the actual onto the
            %   planned when all the directions are deviating from the planned
            %   with each step, while not reseting the position.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function
                uas.gps.commit();
                telemetry = atoc.laneData(res.lane_id).telemetry;
                ro = [0,0,0] + (atoc.time - startTime)*dirVector;
                uasDif = [uas.gps.lon, uas.gps.lat, uas.gps.alt] ...
                    - pos(1, 1:3);
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation(uasDif, ro), ...
                    "AbsTol", .001);
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                uas.gps.lon = uas.gps.lat + rand();
                uas.gps.lat = uas.gps.lon + rand();
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
    end
    %% Calculate Speed and Distance Function Tests
    % This section is used to test that the calculations for deviation in
    % speed and distance are correct
    %
    
    methods(Test) % Distance Tests
        function NoStepNotAddedInDistanceTest(testCase)
            % NoStepNotAddedInDistanceTest - Tests that there is no deviation
            %   in distance from planned.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            testCase.verifyEqual(dis, 0,  "AbsTol",0.001);
        end
        function NoStepDeviationInDistanceTest(testCase)
            % NoStepDeviationInDistanceTest - Tests if there is deviation at
            % the starting of the line.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.lon = pos(1, 1) + rand();
            uas.gps.lat = pos(1, 2) + rand();
            uas.gps.alt = pos(1, 3) + rand();
            uas.gps.commit();
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], pos(1, 1:3));
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            testCase.verifyEqual(dis, del_dis, "AbsTol",0.01);
        end
        function SmallStepXDeviationInDistanceTest(testCase)
            % Set up Objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1) + 1;
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], pos(1, 1:3));
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function LargeStepXDeviationInDistanceTest(testCase)
            % Set up
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            
            % Calculate difference distance
            atoc.time = atoc.time + .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            uas.gps.lon = ri(1) + 100;
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            expected = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected, actual, "AbsTol",0.001);
        end
        function StressTestOfXDeviationResting(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1) + randi(10);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
            end
        end
        function StressTestOfXDeviationNoResetting(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dirV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            del_time = .2;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = uas.gps.lon + 10;
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
            end
        end
        function SmallStepYDeviationInDistanceTest(testCase)
            % Set up Objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + rand();
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function LargeStepYDeviationInDistanceTest(testCase)
            % Set up Objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + randi(10);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.1);
        end
        function StressTestYDeviationDistanceRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            uas.gps.commit();
            del_time = .2;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2)  + randi(10);
                uas.gps.alt = ri(3);
                uas.gps.commit();
            end
        end
        function StressTestYDeviationDistanceNoRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,"AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1);
                uas.gps.lat = uas.gps.lat  + randi(10);
                uas.gps.alt = ri(3);
                uas.gps.commit();
            end
        end
        function SmallStepZDeviationInDistanceTest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            
            atoc.time = atoc.time + del_time;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            expected = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
        end
        function LargeStepZDeviationInDistanceTest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            
            atoc.time = atoc.time + del_time;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + randi(10);
            uas.gps.commit();
            
            expected = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
        end
        function StressTestZDeviationDistanceRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3) + randi(10);
                uas.gps.commit();
            end
        end
        function StressTestZDeviationDistanceNoRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = uas.gps.alt + randi(10);
                uas.gps.commit();
            end
        end
        function SmallStepDeviationSameDirectionInsentive(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            
            atoc.time = atoc.time + del_time;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            uas.gps.lon = ri(1) + 1;
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            expected1 = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected1, actual,  "AbsTol",0.001);
            
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + 1;
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            expected2 = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected2, actual);
            
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + 1;
            uas.gps.commit();
            
            expected3 = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            
            testCase.verifyEqual(expected3, actual);
            testCase.verifyEqual(expected1, expected2);
            testCase.verifyEqual(expected1, expected3);
            testCase.verifyEqual(expected2, expected3);
            
        end
        function SmallStepXYDeviationInDistanceTest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1) + rand();
            uas.gps.lat = ri(2) + rand();
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function LargeStepXYDeviationInDistanceTest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1) + randi(100);
            uas.gps.lat = ri(2) + randi(100);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function StressTestXYDeviationDistanceRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1) + 10;
                uas.gps.lat = ri(2) + 12;
                uas.gps.alt = ri(3);
                uas.gps.commit();
            end
        end
        function StressTestXYDeviationDistanceNoRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = uas.gps.lon + randi(10);
                uas.gps.lat = uas.gps.lat + randi(10);
                uas.gps.alt = ri(3);
                uas.gps.commit();
            end
        end
        function SmallStepXZDeviationInDistanceTest(testCase)
            % Set up Objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1) + rand();
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function LargeStepXZDeviationInDistanceTest(testCase)
            % Set up Objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1) + randi(100);
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + randi(100);
            uas.gps.commit();
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function StressTestXZDeviationDistanceRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1) + randi(10);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3) + randi(10);
                uas.gps.commit();
            end
        end
        function StressTestXZDeviationDistanceNoRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = uas.gps.lon + randi(10);
                uas.gps.lat = ri(2);
                uas.gps.alt = uas.gps.alt + randi(10);
                uas.gps.commit();
            end
        end
        function SmallStepYZDeviationInDistanceTest(testCase)
            % Set up Objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + rand();
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function LargeStepYZDeviationInDistanceTest(testCase)
            % Set up Objects
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            % Move one step
            atoc.time = atoc.time + .2;
            % Calculate Direction Vector
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            % Update UAS Position with noise
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + randi(10);
            uas.gps.alt = ri(3) + randi(10);
            uas.gps.commit();
            % What the distance difference should be
            del_dis = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            % Grab the atoc calculated distance difference
            telemetry = atoc.laneData(res.lane_id).telemetry;
            dis = telemetry.del_dis(end);
            % Make the test
            testCase.verifyEqual(dis, del_dis,  "AbsTol",0.001);
        end
        function StressTestYZDeviationDistanceRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2) + randi(10);
                uas.gps.alt = ri(3) + randi(10);
                uas.gps.commit();
            end
        end
        function StressTestYZDeviationDistanceNeRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1);
                uas.gps.lat = uas.gps.lat + randi(10);
                uas.gps.alt = uas.gps.alt + randi(10);
                uas.gps.commit();
            end
        end
        function TwoDeviationDirectionInsentivieTest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            uas.gps.lon = ri(1) + 10;
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3) + 10;
            uas.gps.commit();
            
            expected1 = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected1, actual,  "AbsTol",0.001);
            
            uas.gps.lon = ri(1) + 10;
            uas.gps.lat = ri(2) + 10;
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            expected2 = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected2, actual, "AbsTol",0.001);
            
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2) + 10;
            uas.gps.alt = ri(3) + 10;
            uas.gps.commit();
            
            expected3 = ATOCUnitTests.CalculateDistanceDifference(...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
            
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_dis(end);
            testCase.verifyEqual(expected3, actual, "AbsTol",0.001);
            
            testCase.verifyEqual(expected3, expected2);
            testCase.verifyEqual(expected3, expected1);
            testCase.verifyEqual(expected1, expected2);
        end
        function StressTestNoDeviationInDistance(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
            end
        end
        function StressTestSmallDeviationInDistance(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual, "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1) + rand();
                uas.gps.lat = ri(2) + rand();
                uas.gps.alt = ri(3) + rand();
                uas.gps.commit();
            end
        end
        function StressTestLargeDeivationInDistance(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = ri(1) + randi(10);
                uas.gps.lat = ri(2) + randi(10);
                uas.gps.alt = ri(3) + randi(10);
                uas.gps.commit();
            end
        end
        function StressTestAllDeviationNoRest(testCase)
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .2;
            dirV = pos(2, 1:3) - pos(1, 1:3);
            % Find where it should be
            ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
            
            while atoc.time < res.exit_time_s - del_time
                expected = ATOCUnitTests.CalculateDistanceDifference(...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], ri);
                
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_dis(end);
                testCase.verifyEqual(expected, actual,  "AbsTol",0.001);
                
                atoc.time = atoc.time + del_time;
                dirV = pos(2, 1:3) - pos(1, 1:3);
                ri = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dirV;
                uas.gps.lon = uas.gps.lon + dirV(1)*randi(10);
                uas.gps.lat = uas.gps.lat + dirV(2)*randi(10);
                uas.gps.alt = uas.gps.alt + dirV(3)*randi(10);
                uas.gps.commit();
            end
        end
    end
    
    methods(Test) % Speed Tests
        function NoStepNotAddedInSpeedTest(testCase)
            % NoStepNotAddedInSpeedTest - Ensures that the del_speed is zero
            % when the drone barely enters into the lane.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            telemetry = atoc.laneData(res.lane_id).telemetry;
            speed = telemetry.del_speed(end);
            testCase.verifyEqual(speed, 0);
        end
        function OneStepNoSpeedDifference(testCase)
            % OneStepNoSpeedDifference - Checks that the change in speed
            % between the planned and actual is zero if on the same path.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            atoc.time = atoc.time + .1;
            dV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
            uas.gps.lon = ri(1);
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            expected = ATOCUnitTests.CalculateSpeedDifference(pos(1, 1:3), ...
                ri, pos(1, 1:3), ri, .1);
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_speed(end);
            testCase.verifyEqual(actual, expected, "AbsTol",0.01);
        end
        function OneStepSmallSpeedDifference(testCase)
            % OneStepSmallSpeedDIfference - Checks that the del_speed is
            % correct when the difference is very small.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            atoc.time = atoc.time + .1;
            dV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
            uas.gps.lon = ri(1) + rand();
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            expected = ATOCUnitTests.CalculateSpeedDifference(pos(1, 1:3), ...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], pos(1, 1:3), ri, .1);
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_speed(end);
            testCase.verifyEqual(actual, expected, "AbsTol",0.01);
        end
        function OneStepLargeSpeedDifference(testCase)
            % OneStepLargeSpeedDifference - Checks to see if the del_speed is
            % calculated correct if the distance is off by a large amount.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            atoc.time = atoc.time + .1;
            dV = pos(2, 1:3) - pos(1, 1:3);
            ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
            uas.gps.lon = ri(1) + randi(100);
            uas.gps.lat = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            expected = ATOCUnitTests.CalculateSpeedDifference(pos(1, 1:3), ...
                [uas.gps.lon, uas.gps.lat, uas.gps.alt], pos(1, 1:3), ri, .1);
            telemetry = atoc.laneData(res.lane_id).telemetry;
            actual = telemetry.del_speed(end);
            testCase.verifyEqual(actual, expected, "AbsTol",0.01);
        end
        function TwoStepNoSpeedDifference(testCase)
            % TwoStepNoSpeedDifference - Checks that moving two steps in the
            % flight trajectory following the path is zero.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 2
                prev_plan = pos(1, 1:3) + (atoc.time - res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function TwoStepSmallSpeedDifference(testCase)
            % TwoStepSmallSpeedDifference - Checks that the del_speed is
            % calculated correctly when taking two steps in the trajectory path
            % with some gausian noise.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 2
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1) + rand();
                uas.gps.lat = ri(2) + rand();
                uas.gps.alt = ri(3) + rand();
                uas.gps.commit();
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function TwoStepLargeSpeedDifference(testCase)
            % TwoStepLargeSpeedDifference - Checks that the del_speed is
            % calculated correctly when taking two steps in the trajactory with
            % a large amount of noise.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 2
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1) + randi(5);
                uas.gps.lat = ri(2) + randi(5);
                uas.gps.alt = ri(3) + randi(5);
                uas.gps.commit();
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function DeviationInXDirectionSpeedTest(testCase)
            % DeviationInXDirectionSpeedTest - Checks that the del_speed
            % calculates correctly when deviating in the x direction.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 10
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function DeviationInYDirectionSpeedTest(testCase)
            % DeviationInYDirectionSpeedTest - Checks to see if the del_speed
            % calculates correctly when continually deviating in the Y
            % direciton
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 10
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1);
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.alt = ri(3);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function DeviationInZDirectionSpeedTest(testCase)
            % DeviationInZDirectionSpeedTest - Checks to ensure that the
            % del_speed calculates correctly when distance is deviating in the
            % Z direction
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 10
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = uas.gps.alt + rand();
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function DeviationInXYDirectionSpeedTest(testCase)
            % DeviationInXYDirectionSpeedTest - Checks to ensure that the
            % del_speed is calculating correctly when continually deivating
            % in the x, y direction
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 10
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.alt = ri(3);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function DeviationInXZDirectionSpeedTest(testCase)
            % DeviationInXZDirectionSpeedTest - Checks to ensure that the
            % del_speed is calculating correctly when continually deviating in
            % the x and z direction
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 10
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = uas.gps.alt + rand();
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function DeviationInYZDirectionSpeedTest(testCase)
            % DeviationInYZDirectionSpeedTest - Checks to ensure that the
            % del_speed is calculating correctly when continually deviating in
            % the y, z direction
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 10
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .1;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1);
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.alt = uas.gps.alt + rand();
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .1);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function StressTestNoDeviationInSpeed(testCase)
            % StressTestNoDeviationInSpeed - Checks to see if the del_speed
            % function works in a large amount of steps over a small amount of
            % time without any deviation.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 100
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .001;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1);
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .001);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
        function StressTestSmallDeviationInSpeed(testCase)
            % StressTestSmallDeviationInSpeed - Is a stress test that checks if
            % the del_speed function can calculate correctly when running
            % multiple steps over a small amount of time with gausian noise.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 100
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .001;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1) + rand();
                uas.gps.lat = ri(2) + rand();
                uas.gps.alt = ri(3) + rand();
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan,...
                    ri, .001);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, 'RelTol', .1);
                step = step + 1;
            end
        end
        function StressTestLargeDeviationInSpeed(testCase)
            % StressTestLargeDeviationInSpeed - A stress test that checks if
            % the del_speed calculation is correct over multiple steps over a
            % small amount of time with large deviation in distances.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 100
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + .001;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = ri(1) + randi(5);
                uas.gps.lat = ri(2) + randi(5);
                uas.gps.alt = ri(3) + randi(5);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, .001);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                check1 = (actual < expected + expected*.1 && ...
                    actual > expected - expected*.1);
                check2 = (actual < expected + .1 && actual > expected - .1);
                testCase.verifyTrue(check1 || check2);
                step = step + 1;
            end
        end
        function VaryingTimeSmallSpeedTests(testCase)
            % VaryingTimeSmallSpeedTests - checks to ensure that the del_speed is
            % correctly calculating over small variation time changes.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 10
                del_time = rand();
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, del_time);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "RelTol",0.1);
                step = step + 1;
            end
        end
        function VaryingTimeLargeSpeedTests(testCase)
            % VaryingTimeLargeSpeedTests - Checks to see if the del_speed is
            % correctly calculating over varying large time jumps.
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            dV = pos(2, 1:3) - pos(1, 1:3);
            step = 0;
            while step < 3
                del_time = randi(3);
                prev_plan = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                prev_uas = [uas.gps.lon, uas.gps.lat, uas.gps.alt];
                
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time -res.entry_time_s)*dV;
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.lat = ri(2);
                uas.gps.alt = ri(3);
                uas.gps.commit();
                
                expected = ATOCUnitTests.CalculateSpeedDifference(prev_uas, ...
                    [uas.gps.lon, uas.gps.lat, uas.gps.alt], prev_plan, ri, del_time);
                telemetry = atoc.laneData(res.lane_id).telemetry;
                actual = telemetry.del_speed(end);
                testCase.verifyEqual(actual, expected, "AbsTol",0.01);
                step = step + 1;
            end
        end
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
            sim.subscribe_to_tick(@atoc.handle_events);
            atoc.time = 0;
            testCase.verifyEqual(0, atoc.time);
            sim.step(.1);
            testCase.verifyEqual(atoc.time, .1);
            sim.step(.3);
            testCase.verifyEqual(atoc.time, .4);
            sim.step(10);
            testCase.verifyEqual(atoc.time, 10.4);
        end
        function tickClusteringHandling(testCase)
            % tickClusteringHandling - checks to see if the findClusters method
            %   is entered when the tick handling happens.
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            atoc.time = 0;
            sz = size(atoc.overallDensity.data);
            numUas = atoc.overallDensity.data(1);
            testCase.verifyEqual(sz, [1,2]);
            testCase.verifyEqual(numUas, 0);
            sim.step(.1);
            sz = size(atoc.overallDensity.data);
            numUas = atoc.overallDensity.data(2);
            testCase.verifyEqual(sz, [2,2]);
            testCase.verifyEqual(numUas,0);
        end
        function telemetryInformationHandling(testCase)
            % telemetryHandling - Checks to ensure that telemetry data is
            % updating when the uas updates its position
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.res_ids = "1";
            uas.gps.commit();
            numRow = height(atoc.telemetry);
            testCase.verifyEqual(2, numRow);
            uas.gps.lat = pos(1, 1) + rand();
            uas.gps.lon = pos(1, 1) + rand();
            uas.gps.alt = pos(1, 3) + rand();
            uas.gps.commit();
            numRow = height(atoc.telemetry);
            testCase.verifyEqual(3, numRow);
        end
        function telemteryLaneDataHandling(testCase)
            % telemetryLaneDataHandling - Checks to ensure that the Lane Data
            % is updating when the uas updates its positions
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            res = lbsd.getLatestRes();
            id = res.lane_id;
            vertid = lbsd.getLaneVertexes(id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.res_ids = res.id;
            uas.gps.commit();
            data = atoc.laneData(id).telemetry;
            sz = height(data);
            testCase.verifyEqual(sz, 2);
            uas.gps.commit();
            data = atoc.laneData(id).telemetry;
            sz = height(data);
            testCase.verifyEqual(sz, 3);
        end
    end
end

