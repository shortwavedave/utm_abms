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
            normPlanned = norm(Planned);
            proj = (dotProduct/normPlanned);
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
        function lane_id = FindFurthestLane(lbsd, laneID)
            % FindFurthestLane - Finds the furthest lane from the specific lane
            % entered as the parameter
            start1 = lbsd.getVertPositions(laneID);
            ids = lbsd.getLaneIds();
            end1 = lbsd.getVertPositions(lbsd.getLaneVertexes(ids(1)));
            lane_id = laneID;
            dis = norm(start1(1, 1:3) - end1(1,1:3));
            for id = 3:length(ids)
                pos = lbsd.getVertPositions(lbsd.getLaneVertexes(ids(id)));
                dis2 = norm(start1(1, 1:3) - pos(1, 1:3));
                if(dis < dis2)
                    dis = dis2;
                    lane_id = ids(id);
                end
            end
        end
        function uas = SetNewPosition(uas, pos)
            uas.gps.lon = pos(1);
            uas.gps.lat = pos(2);
            uas.gps.alt = pos(3);
        end
        function lane_ids = randomFlightPath(num_lanes, lbsd)
        % randomFlightPath - this is a helper method that will find a
        % random path and will return the num_lanes specified by the user.
            lanuch = lbsd.getRandLaunchVert();
            land = lbsd.getRandLandVert();
            [lane_idx, ~, ~] = lbsd.getShortestPath(lanuch, land);
            if(num_lanes < length(lane_idx))
                lane_ids = lane_idx(1:num_lanes);
            else
                lane_ids = lane_idx;
            end
        end 
    end
    %% Create Data Structure Tests
    % This section is used to ensure that the data structures are being
    % created in the way they are meant to be designed.
    %
    % Data Structures to Test
    %   1. masterList - Stores all the flight information from the
    %       simulation
    %   2. sensoryData - stores sensory information from the simulation
    %   3. telemetryData - stores telemetry information from the simulation

    methods(Test)
        function MasterListSetUp(testCase)
            % MazsterListSetUp - ensures that the masterlist is empty when
            % it is first created.
            %
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            master = atoc.masterList;
            field_names = fieldnames(master);
            
            for index = 1:numel(field_names)
                testCase.assertEmpty(master.(field_names{index}));
            end
        end
        function SensorySetUp(testCase)
            % SensorySetUP - Checks to Ensure that the sensory data structure
            %    is setup correctly when the ATOC Object is created
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            radars = atoc.radars;
            field_names = fieldnames(radars);
            
            for index = 1:numel(field_names)
                testCase.assertEmpty(radars.(field_names{index}));
            end
        end
        function TelemeterySetUp(testCase)
            % TelemeterySetUp - Checks to ensure that the telemetry data
            % structure is setup correctly when ATOC Object was created.
            %
            
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            telemetry = atoc.telemetry;
            field_names = fieldnames(telemetry);
            
            for index = 1:numel(field_names)
                testCase.assertEmpty(telemetry.(field_names{index}));
            end
        end
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
            
            numRow = size(atoc.telemetry,1);
            testCase.verifyEqual(1, numRow);
            
            uas.gps.lat = pos(1, 1) + rand();
            uas.gps.lon = pos(1, 1) + rand();
            uas.gps.alt = pos(1, 3) + rand();
            uas.gps.commit();
            
            numRow = size(atoc.telemetry,1);
            testCase.verifyEqual(numRow,1);
        end
        function NewReservationUpdate(testCase)
            % NewReservationUpdate - Ensuring that the ATOC object's lbsd is
            %   updating when a new reservation is being made.
            %
            
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
        function NewReservationUpdateMultiple(testCase)
            % NewReservationUpdateMultiple - this tests ensures that
            %   multiple new reservations are handled by the atoc object
            %
            
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();
            atoc = ATOC(lbsd);
            lbsd.subscribeToNewReservation(@atoc.handle_events)
            ok = false;
            count = 0;
            
            while ~ok || count < 10
                startTime = randi(20);
                endTime = startTime + randi(100);
                [ok, ~] = lbsd.makeReservation(ids(randi(20)), ...
                    startTime, endTime, 1, 5, "1");
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
        function DetectionInformationHandling(testCase)
            % DetectionInformationHandling - This method is to ensure that
            % the atoc object is grabing the sensory information that has
            % been detected.
            %
            
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "20", 0, 10, 1, 5, "1");
            atoc = ATOC(lbsd);
            
            % Getting Latest Reservation
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes(res.lane_id);
            pos = lbsd.getVertPositions(vertid);
            atoc.time = res.entry_time_s;
            
            % Setup RADAR/SIM
            radar = ATOCUnitTests.RADARSetup([pos(1, 1:2), 0],...
                100,pi/4,[0,0,1], "1", lbsd);
            radar.subscribe_to_detection(@atoc.handle_events);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@radar.handle_events);
            
            % Setting up UAS
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), res.uas_id);
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            sim.uas_list = uas;
            
            % One Simulation Step
            sim.step(.1);
            
            % Test Sensory information
            sensory = atoc.radars;
            testCase.verifyEqual("1", sensory.ID(end));
            testCase.verifyEqual(1, size(sensory,1));
        end
    end
    %% Needs to Be Edited
    
    
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
    
  
    %% findClustering Function Tests
    % This section is used to test to ensure that the clustering function
    % is correctly clustering sensory data to telemetry data to ensure that
    % the correct number of UAS are currently flying.
    %
    % Clustering Data is stored in the overallDensity data structure in
    % ATOC.
    
    %% Projection Function Tests
    % This section is used to test that the projection function is
    % correctly calculating the projection value from the planned flight
    % informaiton and the UAS telemetry information. In addition, that the
    % uas is correctly projected to the correct lane.

    %% Calculate Speed and Distance Function Tests
    % This section is used to test that the calculations for deviation in
    % speed and distance are correct
    %

    %% Rogue Detection Tests
    % This section is used to test
    %
    methods(Test)
    end
end