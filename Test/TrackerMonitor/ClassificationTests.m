%% Track Monitor Unit/Load/Integeration Tests For Classification of Flights
% This class is used to test the Track Monitor class to ensure that the
% classification of flights and the setup for this is working properly.

classdef ClassificationTests < matlab.unittest.TestCase
    properties
        lbsd
        monitor
    end

    methods(TestClassSetup)
        function attachFilesForTestClass(TestClassSetup)
            % attchFilesForTestClass - adds the path for the unit tests for
            % the overall classes.
            addpath('..\..\..\utm_abms');
        end
    end
    %% Before and After Test
    % This section is used to setup each test in the class, as well as
    % breakdown any variables in the class.

    methods(TestMethodSetup)
        function createLBSD(testCase)
            % createLBSD - creates a lane system for each unit test.
            % Input:
            %   testCase (test class handle)
            lane = LBSD();
            lane = lane.genSampleLanes(10, 15);
            testCase.lbsd = lane;
        end
        function createTrackMonitor(testCase)
            % createTrackMonitor - Creates the track monitor for each unit
            % test.
            testCase.monitor = TrackMonitor();
        end
    end

    methods(TestMethodTeardown)
    end

    %% Static Helper Methods
    % This section is where all of the helper methods are located that are
    % used by all of the other tests.

    methods(Static)
        function [waypoints, timeOfArrival] = GenerateWayPointsNormal(testCase)
            % GenerateWayPointsNormal - Generates waypoints for a normal
            % trajectory
            launchVert = testCase.lbsd.getRandLaunchVert();
            landVert = testCase.lbsd.getRandLandVert();
            [lane_ids, vert_ids, dis] = testCase.lbsd.getShortestPath(...
                launchVert, landVert);

            waypoints = zeros(length(vert_ids), 3);
            timeOfArrival = zeros(length(vert_ids), 1);
            for index = 1:length(vert_ids)
                waypoints(index, :) = testCase.lbsd.getVertPositions(...
                    vert_ids(index));
            end

            for index = 2:length(vert_ids)
                dis = testCase.lbsd.getLaneLengths(lane_ids(index-1));
                timeOfArrival(index) = timeOfArrival(index-1) + dis;
            end
        end
        function [waypoints, timeOfArrival] = GenerateHobbistTwo(testCase)
            launchVert = testCase.lbsd.getRandLaunchVert();

            % Generate A waypoints upwards with halts
            waypoints = testCase.lbsd.getVertPositions(launchVert);
            landPos = [waypoints(1:2), 20];
            dist = waypoints - landPos;
            timeOfArrival = 0;

            for index = 1:4
                change = waypoints(end, :) + dist/4;
                waypoints = [waypoints; change; change];
                timeOfArrival = [timeOfArrival; ...
                    timeOfArrival(end)+norm(dist/4); ...
                    timeOfArrival(end)+norm(dist/4) + 1];
            end

            s_pts = ClassificationTests.LEM_sphere_pts(waypoints(end, :), ...
                2, 10);
            endLength = size(waypoints, 1) + size(s_pts, 1);
            waypoints(end+1:endLength, :) = s_pts;

            for index = 1:size(s_pts, 1)
                timeOfArrival = [timeOfArrival; timeOfArrival(end) + .01];
            end


            for index = 1:4
                change = waypoints(end, :) - dist/4;
                waypoints = [waypoints; change; change];
                timeOfArrival = [timeOfArrival; ...
                    timeOfArrival(end)+norm(dist/4); ...
                    timeOfArrival(end)+norm(dist/4) + 1];
            end
        end
        function pts = LEM_sphere_pts(center, radius, num_pts)
            pts = zeros(num_pts,3);

            % For the number of points
            for k = 1:num_pts
                % Create some random theta in the x-y plane
                theta = 2*pi*rand;
                % x is the x axis
                x = cos(theta);
                % y is the y axis
                y = sin(theta);
                % some direction
                P = [x;y;0];
                % Reverse direction
                u = [-y;x;0];
                ux = -y;
                uy = x;
                uz = 0;

                phi = 2*pi*rand;
                c = cos(phi);
                s = sin(phi);
                R = zeros(3,3);
                R(1,1) = c + ux^2*(1-c);
                R(1,2) = ux*uy*(1-c) - uz*s;
                R(1,3) = ux*uz*(1-c) + uy*s;
                R(2,1) = uy*ux*(1-c) + uz*s;
                R(2,2) = c + uy^2*(1-c);
                R(2,3) = uy*uz*(1-c) - ux*s;
                R(3,1) = uz*ux*(1-c) - uy*s;
                R(3,2) = uz*uy*(1-c) + ux*s;
                R(3,3) = x + uz^2*(1-c);

                pt = radius*R*P;
                pts(k,:) = pt;
            end
            pts(:,1) = pts(:,1) + center(1);
            pts(:,2) = pts(:,2) + center(2);
            pts(:,3) = pts(:,3) + center(3);
        end
    end

    %% Setup Functions
    % This section is to test that the setup for classifications are
    % working properly. This includes building KD tress as well as ensuring
    % that the integration code is not throwing errors and properly
    % integrated.

    methods(Test)
        function noErrorThrowsKDTree(testCase)
            % noErrorThrowsKDTree - Ensures that when calling the method
            % that the function doesn't throw any errors.
            % Input:
            %   testCase (test class handle)
            try
                testCase.monitor.initializeLaneStructor(testCase.lbsd);
                testCase.verifyTrue(true);
            catch
                testCase.verifyTrue(false);
            end
        end
    end

    %% Nominally Tests
    % This section is used to test that a flight is nominally rather than
    % a Hobby or Rogue flight. These flights include slightly off course,
    % headway distances are not meet, etc. These flights are heading in the
    % right direction and are close to the KD Points.

    methods(Test)
    end

    %% Hobbyist Type 1:
    % This section is used to test flights that display Hobbyist Type 1:
    % Flies up from one place and makes a few moves above the launch site,
    % then eventually lands at the same site.

    methods(Test)
    end

    %% Hobby 2 Tests
    % This section is used to test flights that display Hobbyist Type 2:
    % Flies up from one place and makes a few moves above the launch site,
    % hovers after each move, then eventually lands at the launch site.

    methods(Test)
        function noErrorTestsForHobbistTwoIntegration(testCase)
            % noErrorTestsForHobbistTwoIntegration - This ensures that the
            % integration of the Hobbist two method is has no errors.
            try
                telemetry = table("1", [0,0,0], [1,1,1], 0);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], [1,1,1], 0);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry,radar,[], 1);
                testCase.verifyTrue(true);
            catch
                testCase.verifyTrue(false);
            end
        end
        function noHobbie2Tests(testCase)
            % noHobbie2Tests - This test ensures that no hobbist detection
            % was given the waypoints.
            % Generate normal point
            testCase.createLBSD();
            testCase.createTrackMonitor();
            [waypoints1, timeOfArrival1] = ...
                ClassificationTests.GenerateWayPointsNormal(testCase);
            trajectory1 = waypointTrajectory(waypoints1, timeOfArrival1);
            time = 0;
            count = 0;

            while ~isDone(trajectory1)
                % Pull flight information - create a table with tel_info
                [currentPosition, ~] = trajectory1();
                vel = [1,1,1];
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                time = time + .01;
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], .01);
                if(count > 6)
                    flightInfo = testCase.monitor.flights;
                    testCase.verifyTrue(flightInfo.classification(end), "normal");
                end
                count = count + 1;
            end
        end
        function ensureThatHobbistCodeNoError(testCase)
            % ensureThatHobbistCodeNoError - Ensures that the helper static
            % function has no errors when bulding a hobbist two flights.

            testCase.createLBSD();
            try
                [waypoints1, timeOfArrival1] = ...
                    ClassificationTests.GenerateHobbistTwo(testCase);
                waypointTrajectory(waypoints1, timeOfArrival1);
                testCase.verifyTrue(true);
            catch
                testCase.verifyTrue(false);
            end
        end
        function hobbie2TestsSingle(testCase)
            % hobbie2TestsSingle - this test ensures that
            % the hobbist 2 detection correctly given the two UAS Flights
            % one hobbist 2 and the other normal.

            testCase.createLBSD();
            testCase.createTrackMonitor();
            [waypoints1, timeOfArrival1] = ...
                ClassificationTests.GenerateHobbistTwo(testCase);
            trajectory1 = waypointTrajectory(waypoints1, timeOfArrival1);
            time = 0;
            count = 0;

            while ~isDone(trajectory1)
                % Pull flight information - create a table with tel_info
                [currentPosition, ~] = trajectory1();
                vel = [1,1,1];
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                time = time + .01;
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], .01);
                if(count > 6)
                    flightInfo = testCase.monitor.flights;
                    testCase.verifyTrue(flightInfo.classification(end),...
                        "Hobbist Two");
                end
                count = count + 1;
            end
        end
        function hobbie2TestsDoubleFlightsSameTime(testCase)
            % hobbie2TestsDoubleFlightsSameTime - This test ensures that hobbist 2
            % detection for a single flight.
            testCase.createLBSD();
            testCase.createTrackMonitor();
            [waypoints1, timeOfArrival1] = ...
                ClassificationTests.GenerateHobbistTwo(testCase);
            trajectory1 = waypointTrajectory(waypoints1, timeOfArrival1);

            [waypoints1, timeOfArrival1] = ...
                ClassificationTests.GenerateWayPointsNormal(testCase);
            trajectory2 = waypointTrajectory(waypoints1, timeOfArrival1);

            time = 0;
            count = 0;

            while ~isDone(trajectory1) && ~isDone(trajectory2)
                % Pull flight information - create a table with tel_info
                [currentPosition, ~] = trajectory1();
                [currentPosition2, ~] = trajectory2();
                vel = [1,1,1];
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                name = "2";
                pos = currentPosition2;
                speed = randi(5, [1,3]);
                telemetry{2, {'ID', 'pos', 'speed', 'time'}} ...
                    = [name, pos, speed, time];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                time = time + .01;
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], .01);
                if(count > 6)
                    flightInfo = testCase.monitor.flights;
                    [row1, ~] = find("1", flightInfo.uas_id);
                    [row2, ~] = find("2", flightInfo.uas_id);
                    testCase.verifyTrue(flightInfo.classification(row1(end)),...
                        "Hobbist Two");
                    testCase.verifyTrue(flightInfo.classification(row2(end)),...
                        "normal");
                end
                count = count + 1;
            end
        end
        
        function hobbie2TwoFlightsSame(testCase)
            % hobbie2TwoFlightsSame - This test ensures that the hobbist 2
            % detection works if there are two hobbist 2 flights in the
            % same structure
        end

        function multipleNormalFlights(testCase)
            % multipleNormalFlights - Runs multiple UAS Normal flights to
            % ensure that no Hobbist 2 detection happens.
        end

        function MultipleNormalWithOneHobbistFlight(testCase)
            % MultipleNormalWithOneHobbistFlight - Runs multiple UAS Normal
            % Flights with one Hobbist flight in the mixture. This is to
            % ensure that the track monitor classifies the Hobbist 2-flight
            % correctly.
        end

        function MultipleNormalWithTwoHobbistFlight(testCase)
            % MultipleNormalWithTwoHobbistFlight - Runs multiple UAS Normal
            % flights with two Hobbist flight in the mixture and ensure
            % that track monitor classifies the Hobbist 2 flight correctly.
        end

        function MultipleHobbistTwoFlights(testCase)
            % MultipleHobbistTwoFlights - Runs multiple UAS Hobbist 2
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end

    %% Hobby 3 Tests
    % This section is used to test flights that display Hobbyist Type 3:
    % Flies up in a circular motion to some highest point then flies down
    % in a circular motion to land.

    methods(Test)
        function noHobbie3Tests(testCase)
            % noHobbie3Tests - This test ensures that the Hobbist three
            % isn't detected with one normal UAS Flight
        end

        function hobbie3TestsSingle(testCase)
            % hobbie3TestsSingle - This test ensures that hobbist 3
            % detection for a single flight.
        end

        function hobbie3TestsDoubleFlightsSameTime(testCase)
            % hobbie3TestsDoubleFlightsSameTime - this test ensures that
            % the hobbist 3 detection correctly given the two UAS Flights
            % one hobbist 3 and the other normal.
        end

        function hobbie3TwoFlightsSame(testCase)
            % hobbie3TwoFlightsSame - This test ensures that the hobbist 3
            % detection works if there are two hobbist 3 flights in the
            % same structure
        end

        function multipleNormalFlightsHobbist3(testCase)
            % multipleNormalFlightsHobbist3 - Runs multiple UAS Normal flights to
            % ensure that no Hobbist 3 detection happens.
        end

        function MultipleNormalWithOneHobbist3Flight(testCase)
            % MultipleNormalWithOneHobbistFlight - Runs multiple UAS Normal
            % Flights with one Hobbist flight in the mixture. This is to
            % ensure that the track monitor classifies the Hobbist 2-flight
            % correctly.
        end

        function MultipleNormalWithTwoHobbist3Flight(testCase)
            % MultipleNormalWithTwoHobbist3Flight - Runs multiple UAS Normal
            % flights with two Hobbist flight in the mixture and ensure
            % that track monitor classifies the Hobbist 3 flight correctly.
        end

        function MultipleHobbistThreeFlights(testCase)
            % MultipleHobbistThreeFlights - Runs multiple UAS Hobbist 3
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end

    %% Rogue 1 Tests
    % This section is used to test flights that display Rogue Type 1: Flies
    % up over and down as for a delivery.

    methods(Test)
        function noRogue1Tests(testCase)
            % noRogue1Tests - This test ensures that the Rogue one
            % isn't detected with one normal UAS Flight
        end

        function rogue1TestsSingle(testCase)
            % rogue1TestsSingle - This test ensures that rogue 1
            % detection for a single flight.
        end

        function rogue1TestsDoubleFlightsSameTime(testCase)
            % rogue1TestsDoubleFlightsSameTime - this test ensures that
            % the rogue 1 detection correctly given the two UAS Flights
            % one rogue 1 and the other normal.
        end

        function rogueOneTwoFlightsSame(testCase)
            % rogueOneTwoFlightsSame - This test ensures that the rogue 1
            % detection works if there are two rogue 1 flights in the
            % same structure
        end

        function multipleNormalFlightsRogue1(testCase)
            % multipleNormalFlightsRogue1 - Runs multiple UAS Normal flights to
            % ensure that no Rogue 1 detection happens.
        end

        function MultipleNormalWithOneRogue1Flight(testCase)
            % MultipleNormalWithOneRogue1Flight - Runs multiple UAS Normal
            % Flights with one Hobbist flight in the mixture. This is to
            % ensure that the track monitor classifies the rogue 1 flight
            % correctly.
        end

        function MultipleNormalWithTwoRogue1Flight(testCase)
            % MultipleNormalWithTwoRogue1Flight - Runs multiple UAS Normal
            % flights with two Rogue 1 flight in the mixture and ensure
            % that track monitor classifies the rogue 1 flight correctly.
        end

        function MultipleRogue1Flights(testCase)
            % MultipleRogue1Flights - Runs multiple UAS Rogue 1
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end

    %% Rogue 2 Tests
    % This section is used to test flights that display Rogue Type 2: Flies
    % up to a lane, flies along the lane to the end, then flies to another
    % lane, and eventually flies down to land.

    methods(Test)
        function noRogue2Tests(testCase)
            % noRogue2Tests - This test ensures that the Rogue two
            % isn't detected with one normal UAS Flight
        end

        function rogue2TestsSingle(testCase)
            % rogue2TestsSingle - This test ensures that rogue 2
            % detection for a single flight.
        end

        function rogue2TestsDoubleFlightsSameTime(testCase)
            % rogue2TestsDoubleFlightsSameTime - this test ensures that
            % the rogue 2 detection correctly given the two UAS Flights
            % one rogue 2 and the other normal.
        end

        function rogueTwoTwoFlightsSame(testCase)
            % rogueTwoTwoFlightsSame - This test ensures that the rogue 2
            % detection works if there are two rogue 2 flights in the
            % same structure
        end

        function multipleNormalFlightsRogue2(testCase)
            % multipleNormalFlightsRogue2 - Runs multiple UAS Normal flights to
            % ensure that no Rogue 2 detection happens.
        end

        function MultipleNormalWithOneRogue2Flight(testCase)
            % MultipleNormalWithOneRogue2Flight - Runs multiple UAS Normal
            % Flights with one Rogue flight in the mixture. This is to
            % ensure that the track monitor classifies the rogue 2 flight
            % correctly.
        end

        function MultipleNormalWithTwoRogue2Flight(testCase)
            % MultipleNormalWithTwoRogue2Flight - Runs multiple UAS Normal
            % flights with two Rogue 2 flight in the mixture and ensure
            % that track monitor classifies the rogue 2 flight correctly.
        end

        function MultipleRogue2Flights(testCase)
            % MultipleRogue2Flights - Runs multiple UAS Rogue 2
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end
end