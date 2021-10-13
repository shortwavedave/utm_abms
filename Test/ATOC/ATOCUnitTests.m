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
        function lbsd = SpecificLBSDReservationSetup(lane_id, entry_time, ...
                exit_time, speed, hd)
            % SpecificLBSDReservationSetup - Creates an LBSD object with a
            %       specifc reseveration for unit tests
            lbsd = ATOCUnitTests.LBSDSetup();
            lbsd.clearReservations();
            lbsd.genRandReservations(entry_time, exit_time, ...
                1, lane_id, speed, hd)
        end
        function proj = ProjectionCalculation(Actual, Planned)
        % ProjectionCalculation - A Helper method that calculates the
        %    projection of the Actual onto the Planned vector. 
            dotProduct = dot(Actual, Planned);
            normPlanned = norm(Planned)^2;
            proj = (dotProduct/normPlanned)*Planned;
            proj = norm(Planned - proj);
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
            res = lbsd.getLatestRes();
            ids = res.lane_id;
            vertid = lbsd.getLaneVertexes(ids);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.res_ids = res.id;
            uas.gps.commit();
            id = atoc.laneData(ids).telemetry.ID;
            uasPos = atoc.laneData(ids).telemetry.pos(end, :);
            testCase.verifyEqual(pos(1:3), uasPos, "AbsTol",1);
            testCase.verifyEqual(id(end), "1");
            pos = [pos(1) + rand, pos(2) + rand, pos(3) + rand];
            uas.gps.lat = pos(1);
            uas.gps.lon = pos(2);
            uas.gps.alt = pos(3);
            uas.gps.commit();
            id = atoc.laneData(ids).telemetry.ID;
            uasPos = atoc.laneData(ids).telemetry.pos(end, :);
            testCase.verifyEqual(pos(1:3), uasPos, "AbsTol",1);
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
            lbsd.subscribeToNewReservation(@atoc.handle_events)
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
            lbsd.subscribeToNewReservation(@atoc.handle_events);
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
        function startingProjectTest(testCase)
            % startingProjectTest - Testing that when the UAS is on track at
            % the beginning of the lane the projection is zero.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(0, telemetry.projection);
        end
        function noDeviationProjectionTest(testCase)
            % noDeviationProjectionTest - Testing that if the UAS is following
            %   the desirable path - the projection should be zero or close to
            %   zero.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1, 1:3), "1");
            uas.res_ids = res.id;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            while atoc.time < endTime
                % Submit the new position
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                
                % Testing if the projection is equal to zero
                testCase.verifyEqual(0, telemetry.projection(end));
                
                % Calculate the new position
                atoc.time = atoc.time + del_time;
                ri = pos(1, 1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1);
                uas.gps.lon = ri(2);
                uas.gps.alt = ri(3);
            end
        end
        function OneSetXDeviationProjectionTest(testCase)
        % OneSetXDeviationProjectionTest - Checks one step deviation in the
        %   x direction.
            % SetUp Lane Base System/UAS
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lat = ri(1) + 1;
            uas.gps.lon = ri(2);
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                uas.gps.lon, uas.gps.alt], ri),  "RelTol",0.1);
            
        end
        function MultipleXResetEachStepSetsDeviationProjectionTest(testCase)
        % MultipleXResetEachStepSetsDeviationProjectionTest - Checks over a longer
        %   period of time of moving x direction deviation of some randi
        % 	distribution
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1) + rand();
                uas.gps.lon = ri(2);
                uas.gps.alt = ri(3);
            end
        end
        function ContinualMultipleXStepDeviationProjectionTest(testCase)
        % ContinualMultipleXStepDeviationProjectionTest - Checks to see if
        %   the projection is correct as the x direction deviation with each
        %   step without reseting it back to the planned route with each
        %   step.
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.lon = ri(2);
                uas.gps.alt = ri(3);
            end
        end
        function OneSetYDeviationProjectionTest(testCase)
        % OneSetYDeviationProjectionTest - Checks to see if the projection
        %   helper function is working when one step deviation in the Y
        %   direction
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lat = ri(1);
            uas.gps.lon = ri(2) + randi;
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                uas.gps.lon, uas.gps.alt], ri));
        end
        function MultipleYResetStepDeviationProjectionTest(testCase)
        % MultipleXResetEachStepSetsDeviationProjectionTest - Checks over a longer
        %   period of time of moving x direction deviation of some randi
        % 	distribution
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1);
                uas.gps.lon = ri(2) + rand();
                uas.gps.alt = ri(3);
            end
        end
        function MultipleYNoResetStepDeviationProjectionTest(testCase)
        % MultipleYNoResetStepDeviationProjectionTest - Checks to see if
        %   the helper projection method calculates the project when the UAS
        %   is deviating from the path in the y direction.
            % Set up LBSD/UAS objects
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1);
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.alt = ri(3);
            end
        end
        function OneSetZDeviationProjectionProjectionTest(testCase)
        % OneSetZDeviationProjectionProjectionTest - Checks to see if one
        %   step in the z direction that is deviated from the planned path is
        %   calculated correctly in the helper projection method. 
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lat = ri(1);
            uas.gps.lon = ri(2);
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                uas.gps.lon, uas.gps.alt], ri));
        end
        function MultipleZResetStepDeviationProjectionTest(testCase)
        % MultipleZResetStepDeviationProjectionTest - Checks a longer trial
        %   run of reseting the z direction based on the planned path and
        %   then add some noise deviation. 
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1);
                uas.gps.lon = ri(2);
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleZNoResetStepDeviationProjectionTest(testCase)
        % MultipleZNoResetStepDeviationProjectionTest - Checks to see if
        %   the projection helper method correctly calculates the deviation
        %   when the Z direction continues to further separate from the
        %   planned path.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1);
                uas.gps.lon = ri(2);
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
        function OneStepXYResetStepDeviationProjectionTest(testCase)
        % OneStepXYResetStepDeviationProjectionTest - Checks to see if the
        % projection method works when calculating the projection of actual
        % onto planned when x & y direction has deviations, reseting with
        % each step.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lat = ri(1)  + rand();
            uas.gps.lon = ri(2)  + rand();
            uas.gps.alt = ri(3);
            uas.gps.commit();
            
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                uas.gps.lon, uas.gps.alt], ri));
        end
        function MultipleXYResetStepDeviationProjectionTest(testCase)
        % MultipleXYResetStepDeviationProjectionTest - Checks to see if the
        %   projection helper method correctly projects the acutal onto the
        %   planned when continually deviating in the XY direction while
        %   resting with each step.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1) + rand();
                uas.gps.lon = ri(2) + rand();
                uas.gps.alt = ri(3);
            end
        end
        function MultipleXYNoResetStepDeviationProjectionTest(testCase)
        % MultipleXYNoResetStepDeviationProjectionTest - Checks to see if
        % the projection method works when calculating the projection of
        % actual onto planned when x and y direction continually deviate.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.alt = ri(3);
            end
        end
        function OneStepXZDeviationProjectionTest(testCase)
        % OneStepXZResetStepDeviationProjectionTest - Checks to see fi the
        % projection method works with the x and z deviates from the
        % planned path after one step
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lat = ri(1) + rand();
            uas.gps.lon = ri(2);
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                uas.gps.lon, uas.gps.alt], ri));
        end
        function MultipleXZRestStepDeviationProjectionTest(testCase)
        % MultipleXZRestStepDeviationProjectionTest - Checks to see fi the
        % projection methods works when calculating the projection of the
        % acutal onto planned in the xz direction over a longer period of
        % time, resting with each step.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1) + rand();
                uas.gps.lon = ri(2);
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleXZNoResetDeviationProjectionTest(testCase)
        % MultipleXZNoResetDeviationProjectionTest - Checks to see if the
        % projection methods works when calculating the projection fo the
        % actual onto the planned with continually deviations in the x and
        % z direction over a longer period of time.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.lon = ri(2);
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
        function OneStepYZDeviationProjectionTest(testCase)
        % OneStepYZDeviationProjectionTest - Checks to see if the
        % projection method works when calculating the projection of the
        % acutal onto the planned with one step deviation in the y and z
        % direction.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lat = ri(1);
            uas.gps.lon = ri(2) + rand();
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                uas.gps.lon, uas.gps.alt], ri));
        end
        function MultipleYZResetStepDeviationProjectionTest(testCase)
        % MultipleYZResetStepDeviationProjectionTest - Checks to see fi the
        % projection methods works when calculating the projection of the
        % actual onto the planned with deviations of Y and Z direction,
        % resting after each step.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1);
                uas.gps.lon = ri(2) + rand();
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleYZNoResetStepDeviationProjectionTest(testCase)
        % MultipleYZNoResetStepDeviationProjectionTest - Checks to see if
        % the projection methods work when calculating the projection of
        % the actual onto the planned with deviations of the Y and Z
        % continually deviating form the planned flight.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1);
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
        function OneStepAllDeviationProjectionTest(testCase)
        % OneStepAllDeviationProjectionTest - checks to see if the
        %   projection helper works when the deviation from the planned path
        %   is in all directions.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.gps.commit();
            del_time = .01;
            atoc.time = atoc.time + del_time;
            
            % Calculate new Position
            ri = pos(1:3) + (atoc.time - startTime)*dirVector;
            uas.gps.lat = ri(1) + rand();
            uas.gps.lon = ri(2) + rand();
            uas.gps.alt = ri(3) + rand();
            uas.gps.commit();
            
            telemetry = atoc.laneData("1").telemetry;
            testCase.verifyEqual(telemetry.projection(end), ...
                ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                uas.gps.lon, uas.gps.alt], ri));
        end
        function MultipleStepAllResetDeviationProjection(testCase)
        % MultipleStepAllResetDeviationProjection - Checks to see if the
        %   projection methods works when the deviation in all directions
        %   over a longer period of time works, with each step reseting back
        %   to the planned path and then added deviation.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = ri(1) + rand();
                uas.gps.lon = ri(2) + rand();
                uas.gps.alt = ri(3) + rand();
            end
        end
        function MultipleStepAllNoResetDeviationProjection(testCase)
        % MultipleStepAllNoResetDeviationProjection - Checks to see if the
        %   projection helper methods correctly projects the actual onto the
        %   planned when all the directions are deviating from the planned
        %   with each step, while not reseting the position.
            lbsd = ATOCUnitTests.SpecificLBSDReservationSetup("1", 0, 10, 1, 5);
            res = lbsd.getLatestRes();
            vertid = lbsd.getLaneVertexes("1");
            pos = lbsd.getVertPositions(vertid);
            startTime = res.entry_time_s;
            endTime = res.exit_time_s;
            dirVector = pos(2, 1:3) - pos(1, 1:3);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.res_ids = res.id;
            
            % ATOC Setup
            atoc = ATOC(lbsd);
            atoc.time = res.entry_time_s;
            uas.subscribeToTelemetry(@atoc.handle_events);
            del_time = .01;
            
            while atoc.time < endTime
                % Test the Projection Function 
                uas.gps.commit();
                telemetry = atoc.laneData("1").telemetry;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                testCase.verifyEqual(telemetry.projection(end), ...
                    ATOCUnitTests.ProjectionCalculation([uas.gps.lat,...
                    uas.gps.lon, uas.gps.alt], ri))
                
                % Update ATOC/UAS Information
                atoc.time = atoc.time + del_time;
                ri = pos(1:3) + (atoc.time - startTime)*dirVector;
                uas.gps.lat = uas.gps.lat + rand();
                uas.gps.lon = uas.gps.lon + rand();
                uas.gps.alt = uas.gps.alt + rand();
            end
        end
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
            ids = lbsd.getLaneIds();
            vertid = lbsd.getLaneVertexes(ids(1));
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
            uas.subscribeToTelemetry(@atoc.handle_events);
            uas.res_ids = "1";
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
            res = lbsd.getLatestRes();
            id = res.lane_id;
            vertid = lbsd.getLaneVertexes(id);
            pos = lbsd.getVertPositions(vertid);
            atoc = ATOC(lbsd);
            uas = ATOCUnitTests.UASSetup(pos(1:3), "1");
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

