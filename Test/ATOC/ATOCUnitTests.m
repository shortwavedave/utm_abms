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
            lbds = LBSD();
            lbsd = lbds.genSampleLanes(10, 15);
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
        function lane_id = ClosestLane(lbsd, pos)
            % ClosestLane - This function is to find the closest lane based
            % on the current position. 
            % findClosestLane - This is a helper function that will find the
            % closest lane for a uas that doesn't have a reservation.
            % Grab all the lane information
            lane_ids = lbsd.getLaneIds();
            minDis = Inf;
            % Loop through all the lanes to find the associated
            % lane
            for index = 1:length(lane_ids)
                ids = lbsd.getLaneVertexes(lane_ids(index));
                lane_pos = lbsd.getVertPositions(ids);
                mid = (lane_pos(2,:) + lane_pos(1, :))/2;
                dis = norm(pos - mid);
                if(dis < minDis)
                    minDis = dis;
                    lane_id = lane_ids(index);
                end
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
        function masterListSetUp(testCase)
            % MazsterListSetUp - ensures that the masterList is empty when
            % it is first created.
            %
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            master = atoc.masterList;
            field_names = fieldnames(master);
            
            for index = 1:numel(field_names)
                testCase.assertEmpty(master(1).(field_names{index}));
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
        function RogueUASListSetup(testCase)
        % RogueUASListSetip - this funciton is used to ensure that the
        % rogue uas list is empty when created
            lbsd = atocTests.LBSDSetup();
            atoc = ATOC(lbsd);
            testCase.verifyEmpty(atoc.rogue_uas);
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
            testCase.verifyEqual(atoc.time, 0);
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
                if(ok)
                    count = count + 1;
                end
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
            testCase.verifyEqual("1", sensory(end).ID);
            testCase.verifyEqual(1, size(sensory,1));
        end
        function ResetTemporaryListsNoItem(testCase)
        % ResetTemporaryLists - This test ensures that the temporary list
        % is correctly clearing after each time step. 
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            atoc.time = 0;
            sim.step(.1);
            testCase.verifyEmpty(atoc.telemetry.ID);
            testCase.verifyEmpty(atoc.radars.ID);
        end
        function ResetTemporaryListsWithUASAndRadar(testCase)
        % ResetTemporaryListsWithUASAndRadar - this tests ensures that the
        % temporary list is correctly cleared after each time step. 
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            uas.subscribeToTelemetry(@atoc.handle_events);
            radar.describeToDetection(@atoc.handle_events);
            sim.uas_list = uas;
            sim.subscribe_to_tick(@radar.handle_events);

            uas.res_ids = "1";
            uas.gps.commit();
            
            numRow = size(atoc.telemetry,1);
            testCase.verifyEqual(1, numRow);
            
            uas.gps.lat = pos(1, 1) + rand();
            uas.gps.lon = pos(1, 1) + rand();
            uas.gps.alt = pos(1, 3) + rand();
            uas.gps.commit();
            sim.step(.1);
            testCase.verifyEmpty(atoc.telemetry.ID);
            testCase.verifyEmpty(atoc.radars);
        end
    end

    %% Analysis Functions Tests
    %  This section ensures that all of the calculations to analyze the
    %  flights are correctly done. 
    %   Functions Tested:
    %       1. Projection to lane
    %       2. Del_distance
    %       3. Del_speed

    % Just Updating the masterList - using just informaiton
    methods(Test)
        function noUpdateNoUAS(testCase)
            % noUpdateNoUAS - this test makes sure that the masterList
            % isn't being updated when there is no sensory/telemetry data
            lbsd = ATOCUnitTests.LBSDSetup();
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            sim.step(.1);

            master = atoc.masterList;
            field_names = fieldnames(master);
            
            for index = 1:numel(field_names)
                testCase.assertEmpty(master(1).(field_names{index}));
            end
        end
        function UpdateTelemetryOnly(testCase)
            % UpdateTelemetryOnly - this tests makes sure that the
            % masterList will be updating the information when a uas is
            % sending informaiton only
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);
            sim.uas_list = uas;

            uas.res_ids = "1";
            uas.gps.commit();
            sim.step(.1);
            
            testCase.verifyEqual(2, size(atoc.masterList, 1));
        end
        function UpdateSensoryOneOnly(testCase)
            % UpdateSensoryOnly - this tests makes sure that the masterList
            % will be updated when sensory information is the only one
            % being updated.
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);

            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            sim.step(.1);
            
            testCase.verifyEqual(2, size(atoc.masterList, 1));
            
        end
        function UpdateSensoryMoreOnly(testCase)
            % UpdateSensoryMoreOnly - this test makes sure that the
            % masterList is updated with multiple sensory information
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar2 = ATOCUnitTests.RADARSetup(pos + [1,0,0], 50, pi, [0,0,1], "2", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            radar2.describeToDetection(@atoc.handle_events);

            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            sim.step(.1);
            
            testCase.verifyEqual(2, size(atoc.masterList, 1));
        end
        function UpdateSensoryTelemetry(testCase)
            % UpdateSensoryTelemetry - this test makes sure that the
            % mastlist is updated with telemetry and sensory information
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);

            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
            sim.step(.1);
            
            testCase.verifyEqual(2, size(atoc.masterList, 1));
        end
        function TwoUASUpdatemasterList(testCase)
            % TwoUASUpdatemasterList - This tests ensure that only two
            % pieces of information is updated on the masterList
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "2", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos + [1,0,0], 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            testCase.verifyEqual(2, size(atoc.masterList, 1));
        end
        function ContinualStepUpdatemasterList(testCase)
            % ContinualStepUpdatemasterList - This test ensures that the
            % update masterList is the correct size over multiple steps.
            
            % Creation of the lbsd object
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            end_lane = start_lane;
            % Pick two random indices
            while(end_lane ~= start_lane)
                end_lane = lanes(randi(length(lanes)));
            end
            start_vert = lbsd.getLaneVertexes(start_lane);
            end_vert = lbsd.getLaneVertexes(end_lane);
            [lane_ids, vert_ids, ~] = lbsd.getShortestPath(start_vert, ...
                end_vert);
            uas_id = "1";
            
            pos = lbsd.getVertPositions(vert_ids(1));

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            % Make Reservations
            entry_time = 0;
            for index = 1:length(lane_ids)-1
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);
                lane_id = lane_ids(index);
                exit_time = entry_time + dis;
                del_t = exit_time - exit_time;
                speed = dis/del_t;
                hd = 5;
                
                lbsd.makeReservation(lane_id, entry_time, exit_time, speed, ...
                hd, uas_id);
                entry_time = exit_time;
            end

            stepCounter = 0;
            % Fly the uas
            for index = 1:length(lane_ids)
                % Specific uas
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);

                pos = lbsd.getVertPositions(vert_ids(index));
                dir = pos(2, :) - pos(1, :);
                for move = 1:dis
                    del_t = mod(stepCounter,10)/10;
                    po = pos(1, :) + del_t*dir;
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2);
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    stepCounter = stepCounter + 1;
                end
            end

            testCase.verifyEqual(size(atoc.masterList, 1), stepCounter);
        end
    end

    % Distance Method tests
    methods(Test)
        function NoStepNoDifference(testCase)
            % NoSTepNoDifference - this test is to ensure that the
            % difference between what is being transmitted and the plan
            % path is comming back as zero. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(.1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows);
            testCase.verifyEqual(0, dis);
        end
        function OneStepNoDifference(testCase)
            % OneStepNoDifference - this test is to ensure that the
            % difference column in the mastlist is zero when a uas takes a
            % step that is the same path as its planned flight, after one
            % step. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Advance the UAS
            newPos = pos(1, :) - .1*(pos(2, :) - pos(1, :));
            uas.gps.lon = newPos(1);
            uas.gps.lat = newPos(2);
            uas.gps.alt = newPos(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(0, dis);
        end
        function MultipleStepNoDifference(testCase)
            % MultipleStepNoDifference - This test is to ensure that the
            % differences column over many differing steps through the same
            % length is zero when the uas is tracking with the plan route.
            
            % Creation of the lbsd object
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            lbsd.makeReservation(start_lane, 0, 10, norm(pos)/10, ...
                5, "1");

            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                uas.gps.lon = po(1);
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                [rows, ~] = find(atoc.masterList.id == "1");
                actual = atoc.masterList.del_dis(rows(end));
                testCase.verifyEqual(0, actual);
            end
        end
        function MultipleStepNoDifferenceAcrossLanes(testCase)
            % MultipleStepNoDifferenceAcrossLanes - This test is to ensure
            % that the differences column over many differing steps through
            % multiple different lanes is zero when the uas is tracking
            % exactly with the plan trajectory 
             % Creation of the lbsd object
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            end_lane = start_lane;
            % Pick two random indices
            while(end_lane ~= start_lane)
                end_lane = lanes(randi(length(lanes)));
            end
            start_vert = lbsd.getLaneVertexes(start_lane);
            end_vert = lbsd.getLaneVertexes(end_lane);
            [lane_ids, vert_ids, ~] = lbsd.getShortestPath(start_vert, ...
                end_vert);
            uas_id = "1";
            
            pos = lbsd.getVertPositions(vert_ids(1));

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            % Make Reservations
            entry_time = 0;
            for index = 1:length(lane_ids)-1
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);
                lane_id = lane_ids(index);
                exit_time = entry_time + dis;
                del_t = exit_time - exit_time;
                speed = dis/del_t;
                hd = 5;
                
                lbsd.makeReservation(lane_id, entry_time, exit_time, speed, ...
                hd, uas_id);
                entry_time = exit_time;
            end

            % Fly the uas
            for index = 1:length(lane_ids)
                % Specific uas
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);

                pos = lbsd.getVertPositions(vert_ids(index));
                dir = pos(2, :) - pos(1, :);
                for move = 1:dis
                    del_t = mod(stepCounter,10)/10;
                    po = pos(1, :) + del_t*dir;
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2);
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    [rows, ~] = find(atoc.masterList.id == "1");
                    actual = atoc.masterList.del_dis(rows(end));
                    testCase.verifyEqual(0, actual);
                end
            end
        end
        function NoStepSlightDifference(testCase)
            % NoStepSlightDifference - this tests is to ensure that the
            % distance calculations are correctly calculating the distance
            % from the planned trajectory, with no step
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,1], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(.1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows);
            testCase.verifyEqual(1, dis, "AbsTol", .01);
        end
        function OneStepBothSlightDifference(testCase)
            % OneStepSlightDifference - This test is to ensure that the
            % distance calculations are correctly calculating the distance
            % from the planned trajectory, with one step away. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,1], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Advance the UAS
            newPos = pos(1, :) - .1*(pos(2, :) - pos(1, :));
            uas.gps.lon = newPos(1) + 1;
            uas.gps.lat = newPos(2);
            uas.gps.alt = newPos(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(1, dis, "AbsTol", .01);
        end
        function OneStepOneSLightDifferenceBeginning(testCase)
            % OneStepOneSLightDifferenceBeginning - This test is to ensure
            % that the distance calcualtions are correct when a uas starts
            % off from the planned distance and then corrects to the
            % planned distance. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,1], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Advance the UAS
            newPos = pos(1, :) - .1*(pos(2, :) - pos(1, :));
            uas.gps.lon = newPos(1);
            uas.gps.lat = newPos(2);
            uas.gps.alt = newPos(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(0, dis, "AbsTol", .01);
        end
        function OneStepSlightDifferenceEnd(testCase)
            % OneStepSlightDifferenceEnd - This test is to ensure that the
            % distance calcualtions are correctly calculating the distance
            % from the planned trajectory when it starts on the right track
            % and then deviates.
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Advance the UAS
            newPos = pos(1, :) - .1*(pos(2, :) - pos(1, :));
            uas.gps.lon = newPos(1) + 1;
            uas.gps.lat = newPos(2);
            uas.gps.alt = newPos(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(1, dis, "AbsTol", .01);
        end
        function NoStepLargeDifference(testCase)
            % NoStepLargeDifference - This test is to ensure that the
            % distance calculations are correct, when the uas is
            % transmitting informaiton that is very far away from the
            % planned trajectory
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [10, 0, 0], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(.1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows);
            testCase.verifyEqual(10, dis, "AbsTol", .01);
        end
        function OneStepLargeDifferenceBoth(testCase)
            % OneStepLargeDifference - This test is to ensure that hte
            % distance calculations are correct, when the uas is not in the
            % planned trajectory for either step. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,10,0], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Advance the UAS
            newPos = pos(1, :) - .1*(pos(2, :) - pos(1, :));
            uas.gps.lon = newPos(1);
            uas.gps.lat = newPos(2) + 10;
            uas.gps.alt = newPos(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(10, dis, "AbsTol", .01);
        end
        function OneStepLargeDifferenceBeginning(testCase)
            % OneStepLargeDifferenceBeginning - This test is to ensure that
            % the distance calcualtions are correct, when the uas is off at
            % the beginning of the planned trajectory and then corrects
            % back to the planned trajectory. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3) + [10,01,1], "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Advance the UAS
            newPos = pos(1, :) - .1*(pos(2, :) - pos(1, :));
            uas.gps.lon = newPos(1);
            uas.gps.lat = newPos(2);
            uas.gps.alt = newPos(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(0, dis, "AbsTol", .01);
        end
        function OneStepLargeDifferenceEnd(testCase)
            % OneStepLargeDifferenceEnd - This test is to ensure that the
            % distance calculator is correctly calcualting the distance
            % from the planned trajectory when it starts out on the planned
            % path and then deviates. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Advance the UAS
            newPos = pos(1, :) - .1*(pos(2, :) - pos(1, :));
            uas.gps.lon = newPos(1) + 10;
            uas.gps.lat = newPos(2);
            uas.gps.alt = newPos(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            dis = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(10, dis, "AbsTol", .01);
        end
        function MultipleStepsInAndOutInOneLane(testCase)
            % MultipleStepsInAndOutInOneLane - This test is to ensure that
            % the distance calculator is working accross multiple steps in
            % a single lane. 
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            lbsd.makeReservation(start_lane, 0, 10, norm(pos)/10, ...
                5, "1");

            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                if(mod(move, 2) == 0)
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2);
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    [rows, ~] = find(atoc.masterList.id == "1");
                    actual = atoc.masterList.del_dis(rows(end));
                    testCase.verifyEqual(0, actual);
                else
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2) + 10;
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    [rows, ~] = find(atoc.masterList.id == "1");
                    actual = atoc.masterList.del_dis(rows(end));
                    testCase.verifyEqual(10, actual, "AbsTol", .01);
                end
            end
        end
        function MultipleStepsInAndOutMultipleLanes(testCase)
            % MultipleStepsInAndOutMultipleLanes - This test is to ensure
            % that the distance calculator is working across multiple steps
            % and through multiple lanes. 
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            end_lane = start_lane;
            % Pick two random indices
            while(end_lane ~= start_lane)
                end_lane = lanes(randi(length(lanes)));
            end
            start_vert = lbsd.getLaneVertexes(start_lane);
            end_vert = lbsd.getLaneVertexes(end_lane);
            [lane_ids, vert_ids, ~] = lbsd.getShortestPath(start_vert, ...
                end_vert);
            uas_id = "1";
            
            pos = lbsd.getVertPositions(vert_ids(1));

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            % Make Reservations
            entry_time = 0;
            for index = 1:length(lane_ids)-1
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);
                lane_id = lane_ids(index);
                exit_time = entry_time + dis;
                del_t = exit_time - exit_time;
                speed = dis/del_t;
                hd = 5;
                
                lbsd.makeReservation(lane_id, entry_time, exit_time, speed, ...
                hd, uas_id);
                entry_time = exit_time;
            end

            % Fly the uas
            for index = 1:length(lane_ids)
                % Specific uas
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);

                pos = lbsd.getVertPositions(vert_ids(index));
                dir = pos(2, :) - pos(1, :);
                for move = 1:dis
                    del_t = mod(stepCounter,10)/10;
                    po = pos(1, :) + del_t*dir;
                    if(mod(move, 2) == 0)
                        uas.gps.lon = po(1);
                        uas.gps.lat = po(2);
                        uas.gps.alt = pos(3);
                        uas.gps.commit();
                        sim.step(1);
                        [rows, ~] = find(atoc.masterList.id == "1");
                        actual = atoc.masterList.del_dis(rows(end));
                        testCase.verifyEqual(0, actual);
                    else
                        uas.gps.lon = po(1);
                        uas.gps.lat = po(2) + 10;
                        uas.gps.alt = pos(3);
                        uas.gps.commit();
                        sim.step(1);
                        [rows, ~] = find(atoc.masterList.id == "1");
                        actual = atoc.masterList.del_dis(rows(end));
                        testCase.verifyEqual(10, actual, "AbsTol", .01);
                    end
                end
            end
        end
        function DistinguishBetweenTwoUASOneWrong(testCase)
            % DistinguishBetweenTwoUASOneWrong - This test is to ensure
            % that the distance calculator is working at distinguishing
            % between the differing uas that are in flight, when one is
            % deviated from the path.
            (0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "2", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos + [1,0,0], 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);

            [rows, ~] = find(atoc.masterList.id == "1");
            actual = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(0, actual);

            [rows, ~] = find(atoc.masterList.id == "2");
            actual = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(10, actual, "AbsTol", .01);

        end
        function DistinguishBetweenTwoUASBothWrong(testCase)
            % DistinguishBetweenTwoUASBothWrong - This test is to ensure
            % that hte distance calculator is working at distinguishing
            % between the differing uas that are in flight, when both of
            % the uas have deviated from the path.
            (0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,3,0], "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "2", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos + [1,0,0], 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);

            [rows, ~] = find(atoc.masterList.id == "1");
            actual = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(3, actual, "AbsTol", .01);

            [rows, ~] = find(atoc.masterList.id == "2");
            actual = atoc.masterList.del_dis(rows(end));
            testCase.verifyEqual(10, actual, "AbsTol", .01);
        end
    end

    % Speed Method Tests
    methods(Test)
        function NoStepSpeedDifferenceTest(testCase)
            % NoStepSpeedDifferenceTest - this test is to ensure that the
            % speed calculations are working properly when there is no
            % speed difference in no simulation step
            % NoSTepNoDifference - this test is to ensure that the
            % difference between what is being transmitted and the plan
            % path is comming back as zero. 

            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(1));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(1), 0, 10, 1, speed, "1");

            
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            speed = atoc.masterList.del_speed(rows(end));
            testCase.verifyEqual(0, speed, "Abstol", .01);
        end
        function ONeStepNoSpeedDifferenceTest(testCase)
            % ONeStepNoSpeedDifferenceTest - This test is to ensure tha the
            % speed calculations are working properly when there is no
            % speed difference during one simulation step

            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(1));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(1), 0, 10, 1, speed, "1");

            
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Move the UAS one step
            dir = pos(2, :) - pos(1, :);
            velocity = dir/10;
            uas.gps.lon = uas.gps.lon + velocity(1);
            uas.gps.lat = uas.gps.lat + velocity(2);
            uas.gps.alt = uas.gps.alt + velocity(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            speed = atoc.masterList.del_speed(rows(end));
            testCase.verifyEqual(0, speed, "Abstol", .01);
        end
        function OneStepSlightSpeedDifferenceTestFaser(testCase)
            % OneStepSlightSpeedDifferenceTestFaser - This test is to
            % ensure that the speed calculations can handle uas traveling
            % faster during one simulation step. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(1));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(1), 0, 10, 1, speed, "1");

            
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Move the UAS one step
            dir = pos(2, :) - pos(1, :);
            velocity = dir/10;
            uas.gps.lon = uas.gps.lon + velocity(1) + 1;
            uas.gps.lat = uas.gps.lat + velocity(2);
            uas.gps.alt = uas.gps.alt + velocity(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            speed = atoc.masterList.del_speed(rows(end));
            testCase.verifyEqual(1, speed, "Abstol", .01);
        end
        function OneStepSlightSpeedDifferenceTestSlower(testCase)
            % OneStepSlightSpeedDifferenceTestSlower - This test is to
            % ensure that the speed calcualtions can handle uas traveling
            % slower during one simulation step. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(1));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(1), 0, 10, 1, speed, "1");

            
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            % Move the UAS one step
            dir = pos(2, :) - pos(1, :);
            velocity = dir/10;
            uas.gps.lon = uas.gps.lon + velocity(1) - 1;
            uas.gps.lat = uas.gps.lat + velocity(2);
            uas.gps.alt = uas.gps.alt + velocity(3);
            uas.gps.commit();

            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            speed = atoc.masterList.del_speed(rows(end));
            testCase.verifyEqual(-1, speed, "Abstol", .01);
        end
        function NoSpeedDifferenceInOneLane(testCase)
            % NoSpeedDifferenceInOneLane - This test is to ensure that
            % speed calculations are correct when the uas is sticking to
            % the plan trajectory speed through a single lane. 
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            lbsd.makeReservation(start_lane, 0, 10, norm(pos)/10, ...
                5, "1");

            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                uas.gps.lon = po(1);
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                [rows, ~] = find(atoc.masterList.id == "1");
                actual = atoc.masterList.del_speed(rows(end));
                testCase.verifyEqual(0, actual, "AbsTol", .01);
            end
            
        end
        function SlightSpeedDifferenceInOneLane(testCase)
            % SlightSpeedDifferenceInOneLane - This test is to ensure that
            % the speed calculations are correct when the uas is slightly
            % deviating in the speed through out the lane.
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            lbsd.makeReservation(start_lane, 0, 10, norm(pos)/10, ...
                5, "1");

            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                change = rand();
                uas.gps.lon = po(1) + change;
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                [rows, ~] = find(atoc.masterList.id == "1");
                actual = atoc.masterList.del_speed(rows(end));
                testCase.verifyEqual(change, actual, "AbsTol", .01);
            end
        end
        function ContinualFasterAndSlowerInOneLane(testCase)
            % ContinualFasterAndSlowerInOneLane - This test is to ensure
            % tha the speed calculations can handle continual changed
            % between travelling faster and slower thorughout a single
            % lane.
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            lbsd.makeReservation(start_lane, 0, 10, norm(pos)/10, ...
                5, "1");

            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                change = randi(10)*(rand());
                uas.gps.lon = po(1) + change;
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                [rows, ~] = find(atoc.masterList.id == "1");
                actual = atoc.masterList.del_speed(rows(end));
                testCase.verifyEqual(change, actual, "AbsTol", .01);
            end
        end
        function MultipleLanesNoSpeedDifference(testCase)
            % MultipleLanesNoSpeedDifference - This test is to ensure that
            % the speed calculations can handle continual changing in the
            % lane with no speed difference. 
            % Creation of the lbsd object
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            end_lane = start_lane;
            % Pick two random indices
            while(end_lane ~= start_lane)
                end_lane = lanes(randi(length(lanes)));
            end
            start_vert = lbsd.getLaneVertexes(start_lane);
            end_vert = lbsd.getLaneVertexes(end_lane);
            [lane_ids, vert_ids, ~] = lbsd.getShortestPath(start_vert, ...
                end_vert);
            uas_id = "1";
            
            pos = lbsd.getVertPositions(vert_ids(1));

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            % Make Reservations
            entry_time = 0;
            for index = 1:length(lane_ids)-1
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);
                lane_id = lane_ids(index);
                exit_time = entry_time + dis;
                del_t = exit_time - exit_time;
                speed = dis/del_t;
                hd = 5;
                
                lbsd.makeReservation(lane_id, entry_time, exit_time, speed, ...
                hd, uas_id);
                entry_time = exit_time;
            end

            stepCounter = 0;
            % Fly the uas
            for index = 1:length(lane_ids)
                % Specific uas
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);

                pos = lbsd.getVertPositions(vert_ids(index));
                dir = pos(2, :) - pos(1, :);
                for move = 1:dis
                    del_t = mod(stepCounter,10)/10;
                    po = pos(1, :) + del_t*dir;
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2);
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    stepCounter = stepCounter + 1;
                    [rows, ~] = find(atoc.masterList.id == "1");
                    speed = atoc.masterList.del_speed(rows(end));
                    testCase.verifyEqual(0, speed, "Abstol", .01);
                end
            end
        end
        function MultipleLanesSpeedDifferences(testCase)
            % MultipleLanesSpeedDifferences - This test is to ensure that
            % he speed calculations can handle speeding up and slowing down
            % through multiple lanes. 
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            end_lane = start_lane;
            % Pick two random indices
            while(end_lane ~= start_lane)
                end_lane = lanes(randi(length(lanes)));
            end
            start_vert = lbsd.getLaneVertexes(start_lane);
            end_vert = lbsd.getLaneVertexes(end_lane);
            [lane_ids, vert_ids, ~] = lbsd.getShortestPath(start_vert, ...
                end_vert);
            uas_id = "1";
            
            pos = lbsd.getVertPositions(vert_ids(1));

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            % Make Reservations
            entry_time = 0;
            for index = 1:length(lane_ids)-1
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);
                lane_id = lane_ids(index);
                exit_time = entry_time + dis;
                del_t = exit_time - exit_time;
                speed = dis/del_t;
                hd = 5;
                
                lbsd.makeReservation(lane_id, entry_time, exit_time, speed, ...
                hd, uas_id);
                entry_time = exit_time;
            end

            stepCounter = 0;
            % Fly the uas
            for index = 1:length(lane_ids)
                % Specific uas
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);

                pos = lbsd.getVertPositions(vert_ids(index));
                dir = pos(2, :) - pos(1, :);
                for move = 1:dis
                    del_t = mod(stepCounter,10)/10;
                    po = pos(1, :) + del_t*dir;
                    change = randi(20)*rand();
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2) + change;
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    stepCounter = stepCounter + 1;
                    [rows, ~] = find(atoc.masterList.id == "1");
                    speed = atoc.masterList.del_speed(rows(end));
                    testCase.verifyEqual(change, speed, "Abstol", .01);
                end
            end
        end
    end

    % Projection Method Tests
    methods(Test)
        function UASReserverationCorrectlyProjectedToTheCorrectLane(testCase)
            % UASReserverationCorrectlyProjectedToTheCorrectLane - this
            % test ensures that those that have a reserveration correctly
            % projects to the correct lane. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(1));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(1), 0, 10, 1, speed, "1");

            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            lane_id = atoc.masterList.lane_id(rows(end));
            testCase.verifyEqual(ids(1), lane_id);
        end
        function UASNoReverationLaunchLaneProjectionTest(testCase)
            % UASNoReverationLaunchLaneProjectionTest - this test is to
            % ensure that uas without a reserveration in a lanuch or land 
            % lanes are correctly linked to the correct lane
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            

            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            pos = (pos(2, :) + pos(1, :))/2;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            lane_id = atoc.masterList.lane_id(rows(end));
            testCase.verifyEqual(ids(1), lane_id);
        end
        function UASNoReverationMiddleOfLaneSystem(testCase)
            % UASNoReverationMiddleOfLaneSystem - this test is to ensure
            % that the uas without a reservation in the middle of the lanes
            % are correctly linked to the correct lane. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(5));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            

            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            pos = (pos(2, :) + pos(1, :))/2;
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            lane_id = atoc.masterList.lane_id(rows(end));
            testCase.verifyEqual(ids(5), lane_id);
        end
        function LaunchLaneBeginningProjectionTest(testCase)
            % LaunchLaneProjectionTest - This test is used to insure that
            % the projection method correctly calculates the lane
            % projection when uas are in the launch/land lane.
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(1));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(1), 0, 10, 1, speed, "1");

            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            proj = atoc.masterList.projection(rows(end));
            testCase.verifyEqual(0, proj, "AbsTol", .01);
        end
        function LaunchLaneMiddleProjection(testCase)
            % LaunchLaneMiddleProjection - This test is used to ensure that
            % the projection method can project the uas to the correct
            % calculate the projection when the uas are in the middle of
            % the lanuch/land lane. 
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(1));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(1), 0, 10, 1, speed, "1");

            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            mid = (pos(2, :) + pos(1, :))/2;
            uas = ATOCUnitTests.UASSetup(mid, "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            proj = atoc.masterList.projection(rows(end));
            planned = pos(2, :) - pos(1, :);
            actual = mid - pos(1, :);
            expected = ATOCUnitTests.ProjectionCalculation(actual, planned);
            testCase.verifyEqual(expected, proj, "AbsTol", .01);
        end
        function MiddleLaneSystemProjection(testCase)
            % MiddleLaneSystemProjection - This test is used to ensure that
            % hte projection method can correctly calculate the projection
            % to a lane in the middle of the system.
            rng(0);
            % Set up the LBSD Object
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();

            laneLen = lbsd.getLaneLengths(ids(6));
            speed = laneLen/10;
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                ids(6), 0, 10, 1, speed, "1");

            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the ATOC/SIM Object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);
            
            % Set up the Radar/UAS Objects
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            mid = (pos(2, :) + pos(1, :))/2;
            uas = ATOCUnitTests.UASSetup(mid, "1");
            uas.subscribeToTelemetry(@atoc.handle_events);         
            sim.uas_list = uas;
            uas.res_ids = "1";
            uas.gps.commit();
    
            % Simulation Step
            sim.step(1);

            [rows, ~] = find(atoc.masterList.id == "1");
            proj = atoc.masterList.projection(rows(end));
            planned = pos(2, :) - pos(1, :);
            actual = mid - pos(1, :);
            expected = ATOCUnitTests.ProjectionCalculation(actual, planned);
            testCase.verifyEqual(expected, proj, "AbsTol", .01);
        end
        function ProjectionThroughASingleLaneReserveration(testCase)
            % ProjectionThroughASingleLaneReserveration - this test is used
            % to ensure that the projection method correctly calculates as
            % the uas is moving through a single lane.
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            lbsd.makeReservation(start_lane, 0, 10, norm(pos)/10, ...
                5, "1");

            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                actual = po - pos(1, :);
                change = randi(10)*(rand());
                uas.gps.lon = po(1) + change;
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                expected = ATOCUnitTests.ProjectionCalculation(actual, dir);
                [rows, ~] = find(atoc.masterList.id == "1");
                actual = atoc.masterList.projection(rows(end));
                testCase.verifyEqual(expected, actual, "AbsTol", .01);
            end
        end
        function ProjectionThroughASingleLaneWithoutReservation(testCase)
            % ProjectionThroughASingleLaneWithoutReservation - this test is
            % used to ensure that the projection method correctly
            % calculates as a uas without a reservation is moving through a
            % single lane.
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                change = randi(10)*(rand());
                uas.gps.lon = po(1) + change;
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                lane_id = ATOCUnitTests.ClosestLane(lbsd, po);
                vert = lbsd.getLaneVertexes(lane_id);            
                lane_pos = lbsd.getVertPositions(vert);
                lane_dir = lane_pos(2, :) - lane_pos(1, :);
                actual = po -lane_pos(1, :);
                expected = ATOCUnitTests.ProjectionCalculation(actual, lane_dir);
                [rows, ~] = find(atoc.masterList.id == "1");
                actual = atoc.masterList.projection(rows(end));
                testCase.verifyEqual(expected, actual, "AbsTol", .01);
            end
        end
        function MultipleLanesUASReserverationProjectionTest(testCase)
            % MultipleLanesUASReserverationProjectionTest - this test is
            % used to ensure that the projection method can correctly
            % calculate the projection of a uas with a reserveration to
            % their corrsponding lane.
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            end_lane = start_lane;
            % Pick two random indices
            while(end_lane ~= start_lane)
                end_lane = lanes(randi(length(lanes)));
            end
            start_vert = lbsd.getLaneVertexes(start_lane);
            end_vert = lbsd.getLaneVertexes(end_lane);
            [lane_ids, vert_ids, ~] = lbsd.getShortestPath(start_vert, ...
                end_vert);
            uas_id = "1";
            
            pos = lbsd.getVertPositions(vert_ids(1));

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            % Make Reservations
            entry_time = 0;
            for index = 1:length(lane_ids)-1
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);
                lane_id = lane_ids(index);
                exit_time = entry_time + dis;
                del_t = exit_time - exit_time;
                speed = dis/del_t;
                hd = 5;
                
                lbsd.makeReservation(lane_id, entry_time, exit_time, speed, ...
                hd, uas_id);
                entry_time = exit_time;
            end

            stepCounter = 0;
            % Fly the uas
            for index = 1:length(lane_ids)
                % Specific uas
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);

                pos = lbsd.getVertPositions(vert_ids(index));
                dir = pos(2, :) - pos(1, :);
                for move = 1:dis
                    del_t = mod(stepCounter,10)/10;
                    po = pos(1, :) + del_t*dir;
                    change = randi(20)*rand();
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2) + change;
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    actual = po(1, :) - pos(1, :);
                    stepCounter = stepCounter + 1;
                    expected = ATOCUnitTests.ProjectionCalculation(actual, dir);
                    [rows, ~] = find(atoc.masterList.id == "1");
                    actual = atoc.masterList.projection(rows(end));
                    testCase.verifyEqual(expected, actual, "AbsTol", .01);
                end
            end
        end
        function MultipleLanesUASNoReserverationProjectionTest(testCase)
            % MultipleLanesUASNoReserverationProjectionTest - This test is
            % used to ensure that the projection method can correctly
            % calculate the projection of a uas without a reserveration to
            % the closet lane.
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            end_lane = start_lane;
            % Pick two random indices
            while(end_lane ~= start_lane)
                end_lane = lanes(randi(length(lanes)));
            end
            start_vert = lbsd.getLaneVertexes(start_lane);
            end_vert = lbsd.getLaneVertexes(end_lane);
            [lane_ids, vert_ids, ~] = lbsd.getShortestPath(start_vert, ...
                end_vert);
            
            pos = lbsd.getVertPositions(vert_ids(1));

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);

            stepCounter = 0;
            % Fly the uas
            for index = 1:length(lane_ids)
                % Specific uas
                dis = lbsd.getLaneLengths([lane_ids(index), ...
                    lane_ids(index+1)]);

                pos = lbsd.getVertPositions(vert_ids(index));
                dir = pos(2, :) - pos(1, :);
                for move = 1:dis
                    del_t = mod(stepCounter,10)/10;
                    po = pos(1, :) + del_t*dir;
                    change = randi(20)*rand();
                    uas.gps.lon = po(1);
                    uas.gps.lat = po(2) + change;
                    uas.gps.alt = pos(3);
                    uas.gps.commit();
                    sim.step(1);
                    
                    lane_id = ATOCUnitTests.ClosestLane(lbsd, po);
                    vert = lbsd.getLaneVertexes(lane_id);            
                    lane_pos = lbsd.getVertPositions(vert);
                    lane_dir = lane_pos(2, :) - lane_pos(1, :);
                    actual = po -lane_pos(1, :);
                    expected = ATOCUnitTests.ProjectionCalculation(actual, ...
                        lane_dir);
                    [rows, ~] = find(atoc.masterList.id == "1");
                    actual = atoc.masterList.projection(rows(end));
                    testCase.verifyEqual(expected, actual, "AbsTol", .01);
                end
            end
        end
    end

    %% Rogue Detection Testing Methods
    % This section ensures that the rogue analysis is correctly correctly
    %   Rogue Behaviors:
    %       1. Headway distances incorrect
    %       2. No Reservation
    %       3. Not Transmitting telemetry data
    %       4. Too large of a difference in speed
    %       5. Too large of a difference between dis
    
    % Headway Distance Test Methods
    methods(Test)
        function NoDifferenceInHeadWayOneUAS(testCase)
            % NoDifferenceInHeadWayOneUAS - this test is to ensure that the
            % headway distance rogue detection is negative
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            sim.uas_list = uas1;
            uas1.res_ids = "1";
            uas1.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
        function CorrectHeadWayDistancesTwoUAS(testCase)
            % CorrectHeadWayDistancesTwoUAS - this test is to ensure that
            % the headway distance rogue detection is negative for correct
            % headway distances
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "2", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos + [1,0,0], 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
        function SlightlyTooCloseHeadwayDistances(testCase)
            % SlightlyTooCloseHeadwayDistances - This test is to ensure
            % that the headway distance rogue detection is positive to for
            % uas that are slightly to close to each other
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);  

            sim.uas_list = uas1;
            sim.step(1);

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 1, 10, 1, 5, "2");

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "2");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            uas1.gps.lon = uas1.lon + randi(2);
            uas1.gps.lan = uas1.lon + randi(2);
            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));
        end
        function HalfOfHeadwayDistance(testCase)
            % HalfOfHeadwayDistance - This test is to ensure that the
            % headway distance rogue detection is positive for uas that are
            % half their distances away from each other. 
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);  

            sim.uas_list = uas1;
            sim.step(1);

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 1, 10, 1, 5, "2");

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "2");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            uas1.gps.lon = uas1.lon + 3;
            uas1.gps.lan = uas1.lon;
            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));
        end
        function OnTopOfEachOtherHeadwayDistance(testCase)
            % OnTopOfEachOtherHeadwayDistance - This test is to ensure that
            % the headway distance rogue detection is positive for uas that
            % are on top of one another
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);  

            sim.uas_list = uas1;
            sim.step(1);

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 1, 10, 1, 5, "2");

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "2");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            uas1.gps.lon = uas1.lon + 1;
            uas1.gps.lan = uas1.lon + 1;
            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));
        end
        function DifferingHeadwayDistanceNoRogue(testCase)
            % DifferingHeadwayDistanceNoRogue - This is to test that
            % differing headway distances are accepted with the max(hd1,
            % hd2) distance away
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);  

            sim.uas_list = uas1;
            sim.step(1);

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "2", 1, 10, 1, 1, "2");

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "2");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            uas1.gps.lon = uas1.lon + 3;
            uas1.gps.lan = uas1.lon + 4;
            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
        function DifferingHeadwayDistanceOneRogue(testCase)
            % DifferingHeadwayDistanceOneRogue - This is to test that
            % differing headway distances that one is valid and the other
            % is invalid, and the masterList will reflect this.
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);  

            sim.uas_list = uas1;
            sim.step(1);

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "2", 1, 10, 1, 1, "2");

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "2");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            uas1.gps.lon = uas1.lon + 3;
            uas1.gps.lan = uas1.lon;
            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
        function DifferingHeadwayDistanceTwoRogue(testCase)
            % DifferingHeadwayDistanceTwoRogue - This test is to ensure
            % that two uas with differing headway distances are min(hd1,
            % hd2) away the rogue detection is correctly indicating for the
            % two pairs.
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 15, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);  

            sim.uas_list = uas1;
            sim.step(1);

            % Set up the second reservation
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "2", 1, 10, 1, 5, "2");

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "2");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            uas1.gps.lon = uas1.lon + 3;
            uas1.gps.lan = uas1.lon + 1;
            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));
        end
        function NoReservationRogueDetectionHeadwayDistance(testCase)
            % NoReservationRogueDetectionHeadwayDistance - this test is to
            % ensure that headway rogue detection is working with uas that
            % don't have a reservation.
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();

            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);  

            sim.uas_list = uas1;
            sim.step(1);

            % Set up the second UAS and radar
            uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3), "2");
            uas2.subscribeToTelemetry(@atoc.handle_events);  
            radar2 = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "2", lbsd);
            radar2.describeToDetection(@atoc.handle_events);

            uas1.gps.lon = uas1.lon + 3;
            uas1.gps.lan = uas1.lon;
            sim.uas_list = [uas1; uas2];
            uas1.res_ids = "1";
            uas2.res_ids = "2";
            uas1.gps.commit();
            uas2.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));

            [rows, ~] = find(atoc.masterList.id == "2");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));
        end
    end
    
    % UAS not Transmitting informaiton
    methods(Test)
        function OneUASTransmittingInformation(testCase)
            % OneUASTransmittingInformation - This test ensures that the
            % transmittion detection is correctly working and will state a
            % negative rogue behavior for transmittion
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            sim.uas_list = uas1;
            uas1.res_ids = "1";
            uas1.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
        function OneUASNotTransmittingInformation(testCase)
            % OneUASNotTransmittingInformation - This test ensures that the
            % transmittion detection is correctly working and will start a
            % positive rogue behavior for a uas not transmitting
            % information
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            sim.uas_list = uas1;
            uas1.res_ids = "1";
            sim.step(.1);

            testCase.verifyTrue(atoc.masterList.Rogue(end));
        end
        function OneUASContinueTransmittionInformation(testCase)
            % OneUASContinueTransmittionInformation - This test ensures
            % that the transmittion detection correctly associates the
            % single uas through the flight in a lane
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            lbsd.makeReservation(start_lane, 0, 10, norm(pos)/10, ...
                5, "1");

            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                uas.gps.lon = po(1);
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                [rows, ~] = find(atoc.masterList.id == "1");
                testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
            end
            
        end
        function TwoUASTransmittionOneRogueInformaiton(testCase)
            % TwoUASTransmittionOneRogueInformaiton - This test ensures
            % that when there are multiple uas and one not transmiting it
            % will correctly find the correct uas. 
                rng(0);
                % Create the first reservation
                lbsd = ATOCUnitTests.LBSDSetup();
                lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                    "1", 0, 10, 1, 5, "1");
                ids = lbsd.getLaneIds();
                vertid = lbsd.getLaneVertexes(ids(1));
                pos = lbsd.getVertPositions(vertid);
                
                % Set up the Atoc and Sim object
                atoc = ATOC(lbsd);
                sim = ATOCUnitTests.SIMSetup();
                sim.subscribe_to_tick(@atoc.handle_events);
    
                % Set up first radar & UAS
                radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
                radar.describeToDetection(@atoc.handle_events);
                uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
                uas1.subscribeToTelemetry(@atoc.handle_events);   
    
                % Set up the second reservation
                lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                    "2", 0, 10, 1, 5, "1");
                ids = lbsd.getLaneIds();
                vertid = lbsd.getLaneVertexes(ids(1));
                pos = lbsd.getVertPositions(vertid);
    
                % Set up the second UAS and radar
                uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
                uas2.subscribeToTelemetry(@atoc.handle_events);  
                radar2 = ATOCUnitTests.RADARSetup(pos + [1,0,0], 50, pi, [0,0,1], "2", lbsd);
                radar2.describeToDetection(@atoc.handle_events);
    
                sim.uas_list = [uas1; uas2];
                uas1.res_ids = "1";
                uas2.res_ids = "2";
                uas2.gps.commit();
                sim.step(1);

                [rows, ~] = find(atoc.masterList.id == "1");
                testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));

                [rows, ~] = find(atoc.masterList.id == "2");
                testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
    end
    
    % UAS No Reservation Information
    methods(Test)
        function OneUASNoReservationInformation(testCase)
            % OneUASNoReservationInformation - This test is to ensure that
            % the reservation rogue detection can correct indicate rogue
            % behavior when uas is flying without reservation.
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            sim.uas_list = uas1;
            uas1.res_ids = "1";
            uas1.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyTrue(atoc.masterList.Rogue(rows(end)));
        end
        function OneUASWithReservationInformation(testCase)
            % OneUASWithReservationInformation - This test is to ensure
            % that the reservation rogue detection can correct indicate no
            % rogue behavior when there is a reservation
            rng(0);
            % Create the first reservation
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                "1", 0, 10, 1, 5, "1");
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            
            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);   

            sim.uas_list = uas1;
            uas1.res_ids = "1";
            uas1.gps.commit();
            sim.step(.1);
            
            [rows, ~] = find(atoc.masterList.id == "1");
            testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
        function OneUASConitualNoReserverionINformation(testCase)
            % OneUASConitualNoReserverionINformation - This test case is to
            % ensure that the reservation rogue detection can correct flow
            % a uas without a reservation. 
            rng(0);
            lbsd = ATOCUnitTests.LBSDSetup();
            lanes = lbsd.getLaneIds();
            start_lane = lanes(randi(length(lanes)));
            
            start_vert = lbsd.getLaneVertexes(start_lane);            
            pos = lbsd.getVertPositions(start_vert);

            % Set up the Atoc and Sim object
            atoc = ATOC(lbsd);
            sim = ATOCUnitTests.SIMSetup();
            sim.subscribe_to_tick(@atoc.handle_events);

            % Set up first radar & UAS
            radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
            radar.describeToDetection(@atoc.handle_events);
            sim.subscribe_to_tick(@radar.handle_events);
            uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas1.subscribeToTelemetry(@atoc.handle_events);
            dis = lbsd.getLaneLengths([lane_ids(index)]);

            pos = lbsd.getVertPositions(vert_ids(index));
            dir = pos(2, :) - pos(1, :);
            for move = 1:dis
                del_t = mod(stepCounter,10)/10;
                po = pos(1, :) + del_t*dir;
                uas.gps.lon = po(1);
                uas.gps.lat = po(2);
                uas.gps.alt = pos(3);
                uas.gps.commit();
                sim.step(1);
                testCase.verifyTrue(atoc.masterList.Rogue(end));
            end
        end
        function OneUASNoReservationTwoUASFLights(testCase)
            % OneUASNoReservationTwoUASFLights - This test case is to
            % ensure that the reservation rogue detection can correctly
            % link the rogue behavior when two uas are flying
            rng(0);
                % Create the first reservation
                lbsd = ATOCUnitTests.LBSDSetup();
                ids = lbsd.getLaneIds();
                vertid = lbsd.getLaneVertexes(ids(1));
                pos = lbsd.getVertPositions(vertid);
                
                % Set up the Atoc and Sim object
                atoc = ATOC(lbsd);
                sim = ATOCUnitTests.SIMSetup();
                sim.subscribe_to_tick(@atoc.handle_events);
    
                % Set up first radar & UAS
                radar = ATOCUnitTests.RADARSetup(pos, 50, pi, [0,0,1], "1", lbsd);
                radar.describeToDetection(@atoc.handle_events);
                uas1 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
                uas1.subscribeToTelemetry(@atoc.handle_events);   
    
                % Set up the second reservation
                lbsd = ATOCUnitTests.SpecificLBSDReservationSetup(lbsd, ...
                    "2", 0, 10, 1, 5, "1");
                ids = lbsd.getLaneIds();
                vertid = lbsd.getLaneVertexes(ids(1));
                pos = lbsd.getVertPositions(vertid);
    
                % Set up the second UAS and radar
                uas2 = ATOCUnitTests.UASSetup(pos(1, 1:3) + [0,0,10], "1");
                uas2.subscribeToTelemetry(@atoc.handle_events);  
                radar2 = ATOCUnitTests.RADARSetup(pos + [1,0,0], 50, pi, [0,0,1], "2", lbsd);
                radar2.describeToDetection(@atoc.handle_events);
    
                sim.uas_list = [uas1; uas2];
                uas1.res_ids = "1";
                uas2.res_ids = "2";
                uas2.gps.commit();
                sim.step(1);

                [rows, ~] = find(atoc.masterList.id == "2");
                testCase.verifyFalse(atoc.masterList.Rogue(rows(end)));
        end
    end
end