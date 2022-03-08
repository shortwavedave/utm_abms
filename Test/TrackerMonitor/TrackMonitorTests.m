%% Track Monitor Unit/Load/Integeration Tests
% This class is used to unit test the main functions in the track monitor
% class. In addition, this class is also used to load test the methods as
% well as their integeration with the tracker class.

classdef TrackMonitorTests < matlab.unittest.TestCase
    % Helper Functions
    methods(Static)
        function telemetry = GenerateRandomTelemetryData(numTel)
            % GenerateRandomTelemetryData - Generates random telemetry data
            % Input:
            %   numTel (float): Number of telemetry data to be generated
            % Output:
            %   telemetry (numTel X 4)
            %       .ID (string): UAS Indentification
            %       .pos (1x3): x,y,z coordinates
            %       .speed (1x3): vx, vy, vz values
            %       .time (float): time recorded

            time = randi(40);
            tnew = table();
            tnew.ID = "";
            tnew.pos = zeros(1, 3);
            tnew.speed = zeros(1,3);
            tnew.time = 0;
            telemetry = tnew;
            for row = 1:numTel
                name = num2str(row);
                pos = randi(100, [1,3]);
                speed = randi(5, [1,3]);
                telemetry{row, {'ID', 'pos', 'speed', 'time'}} ...
                    = [name, pos,speed, time];
            end
        end
        function sensory = GenerateRandomSensoryData(numSen)
            % GenerateRandomSensoryData - Generates random telemetry data
            % Input:
            %   numSen (float): Number of Sensory data to be generated
            % Output:
            %   Sensory (numTel X 4)
            %       .ID (string): UAS Indentification
            %       .pos (1x3): x,y,z coordinates
            %       .speed (1x3): vx, vy, vz values
            %       .time (float): time recorded
            time = randi(40);
            sensory = table();
            for row = 1:numSen
                sensory{row, {'ID', 'pos', 'speed', 'time'}} ...
                    = [num2str(row), randi(10, [1,3]),randi(5, [1,3]), time];
            end
        end
        function sensory = GenerateEmptySensory()
            % GenerateRandomTelemetryData - Generates an empty Sensory
            % informaiton
            % Output:
            %   sensory (numTel X 4)
            %       .ID (string): UAS Indentification
            %       .pos (1x3): x,y,z coordinates
            %       .speed (1x3): vx, vy, vz values
            %       .time (float): time recorded
            sensory = table();
            sensory.ID = "";
            sensory.pos = zeros(1, 3);
            sensory.speed = zeros(1,3);
            sensory.time = 0;
        end
        function TestEquality(testCase, actual, expected, index)
            % TestEquality - Runs the test case code to test the equality
            % of the telemetry and sensory information for the last index.
            testCase.verifyEqual(expected.ID, actual.ID(index));
            testCase.verifyEqual(expected.time, actual.time(index));
            testCase.verifyEqual(expected.pos(1), actual.pos(index, 1));
            testCase.verifyEqual(expected.pos(2), actual.pos(index, 2));
            testCase.verifyEqual(expected.pos(3), actual.pos(index, 3));
            testCase.verifyEqual(expected.speed(1), actual.speed(index, 1));
            testCase.verifyEqual(expected.speed(2), actual.speed(index, 2));
            testCase.verifyEqual(expected.speed(3), actual.speed(index, 3));
        end
        function [sim, radarList, num_steps] = setUpSimulationFlights()
            lbsd = LBSD.genSampleLanes(10, 15);
            sim = Sim();
            sim.lbsd = lbsd;
            sim.initialize();
            radarList = sim.radar_list;
            res = sim.lbsd.getReservations();
            minTime = min(res.entry_time_s);
            maxTime = max(res.exit_time_s);
            num_steps = floor((maxTime - minTime)/sim.step_rate_hz);
            sim.atoc.time = minTime;
            for numradar = 1:length(sim.radar_list)
                sim.radar_list(numradar).time = minTime;
            end
        end
        function radars = MakeTables(radar_list)
            tnew = table();
            tnew.ID = "";
            tnew.pos = zeros(1,3);
            tnew.speed = zeros(1,3);
            tnew.time = 0;
            radars = tnew;
            index = 1;
           
            for num = 1:size(radar_list)
                radar = radar_list(num);
                if(~isempty(radar.targets))
                    for item = 1:size(radar.targets, 2)
                        radars{index, {'ID', 'pos', 'speed', 'time'}}...
                            = [radar.ID, [radar.targets(item).x, ...
                            radar.targets(item).y, radar.targets(item).z],...
                            [radar.targets(item).s], 0];
                        index = index + 1;
                    end
                end
            end
        end

    end
    %% Constructor Tests
    % Ensures that the constructor is working properly with assigning
    % instance variables to their correct values when being constructed.

    % Test the Constructor
    methods(Test)
        function emptyTrackerList(testCase)
            % emptyTrackerList - Ensures that the tracker monitor object
            % has an empty tacker list on construction
            monitor = TrackMonitor();
            testCase.verifyEmpty(monitor.tackers);
        end
        function emptyUpdateListerners(testCase)
            % emptyUpdateListerners - Ensure that the tracker monitor
            % object has an empty update listener on construction
            monitor = TrackMonitor();
            testCase.verifyEmpty(monitor.update_listers);
        end
        function emptyClassifiedFlights(testCase)
            % emptyClassifiedFlights - Ensure that the current flight
            % information is empty upon construction
            monitor = TrackMonitor();
            testCase.verifyEqual(1, size(monitor.classifiedFlights, 1));
        end
    end

    %% Event Tests
    % Ensures that the event handling methods are working properly between
    % atoc class and the tracker class when transmitting information back
    % and forth.

    % Test Event Functions
    methods(Test)
        function singleSubscribingListener(testCase)
            % singleSubscribingListener - Ensures that the subscription
            % function is correctly adding a single element to its list.
            monitor = TrackMonitor();
            track = Tracker([0;0;0;0;0;0]);
            monitor.subscribe_to_updates(@track.start_update);
            testCase.verifyEqual(1, size(monitor.update_listers, 1));
        end
        function TwoSubscribingListener(testCase)
            % TwoSubscribingListener - Ensures that the subscription
            % function is correctly adding two elements to its list
            monitor = TrackMonitor();
            track = Tracker([0;0;0;0;0;0]);
            monitor.subscribe_to_updates(@track.start_update);
            track2 = Tracker([0;0;0;0;0;0]);
            monitor.subscribe_to_updates(@track2.start_update);
            testCase.verifyEqual(2, size(monitor.update_listers, 1));
        end
        function MultipleScribingListeners(testCase)
            % MultipleScribingListeners - Ensure that the subscription
            % function is correctly adding multiple elements to its list.
            monitor = TrackMonitor();
            for count = 1:10
                track = Tracker([1;2;3;4;5;6]);
                monitor.subscribe_to_updates(@track.start_update);
            end
            testCase.verifyEqual(10, size(monitor.update_listers, 1));
        end
    end

    %% Track Monitor - Tracker
    % Ensures that the analysis of tracker objects contained in the track
    % monitor class are working properly, by ensuring proper grouping of
    % objects in the simulation and associating them with the correct
    % tracker object.

    % Correctly Updates Classified Flights with information
    methods(Test)
        function TelemetryReportedCorrectSingleUAS(testCase)
            % TelemetryReportedCorrectSingleUAS - Ensures that the telemetry
            % information is being correctly updated for single UAS object
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(telemetry, sensory, [], 1);
            actualTelemetry = monitor.classifiedFlights(end).telemetry;
            TrackMonitorTests.TestEquality(testCase, actualTelemetry, ...
                telemetry, 1);
        end
        function TelemetryReportedCorrectTwoUAS(testCase)
            % TelemetryReportedCorrectTwoUAS - Ensures that the telemetry
            % information is being correctly updated for two UAS Object
            rng(0);
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(2);
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            actualTelemetry = monitor.classifiedFlights(end).telemetry;

            % Testing the position should equal one another
            testCase.verifyTrue(actualTelemetry(end).pos(1) == ...
                telemetry.pos(end, 1) || actualTelemetry(end).pos(1) == ...
                telemetry.pos(1, 1));
            testCase.verifyTrue(actualTelemetry(end).pos(2) == ...
                telemetry.pos(end, 2) || actualTelemetry(end).pos(2) == ...
                telemetry.pos(1, 2));
            testCase.verifyTrue(actualTelemetry(end).pos(3) == ...
                telemetry.pos(end, 3) || actualTelemetry(end).pos(3) == ...
                telemetry.pos(1, 3));
        end
        function TelemetryReportedCorrectMultipleSteps(testCase)
            % TelemetryReportedCorrectMultipleSteps - Ensures that the
            % telemetry information is being correctly updated through
            % multiple steps of a flight path.

            rng(0);
            monitor = TrackMonitor();
            for steps = 1:10
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
                sensory = TrackMonitorTests.GenerateEmptySensory();
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                actualTelemetry = monitor.classifiedFlights(end).telemetry;
                TrackMonitorTests.TestEquality(testCase, actualTelemetry, ...
                    telemetry, 1);
            end
        end
        function RadarReportCorrectOneSteps(testCase)
            % RadarReportCorrectOneStep - Ensures that the sensory
            % information is being correctly updated through a single step.
            %
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
            sensory = telemetry;
            sensory.pos = telemetry.pos*.1 + telemetry.pos;
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            actualTelemetry = monitor.classifiedFlights(end).sensory;
            TrackMonitorTests.TestEquality(testCase, actualTelemetry, ...
                sensory, 1);
        end
        function RadarReportCorrectTwoSteps(testCase)
            % RadarReportCorrectTwoSteps - Ensures that the sensory
            % information is being correctly updated through two steps
            rng(0);
            monitor = TrackMonitor();
            for steps = 1:2
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
                sensory = telemetry;
                sensory.pos = telemetry.pos*.1 + telemetry.pos;
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                actualTelemetry = monitor.classifiedFlights(end).sensory;
                TrackMonitorTests.TestEquality(testCase, actualTelemetry, ...
                    sensory, 1);
            end
        end
        function RadarReportCorrectMultipleStepsSingleUASSingleRadar(testCase)
            % RadarReportCorrectMultipleSteps - Ensures that the sensory
            % information is being correctly updated through multiple steps
            % for a single uas and a single radar object.
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 10)
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
                sensory = telemetry;
                sensory.pos = telemetry.pos*.1 + telemetry.pos;
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                actualSensory = monitor.classifiedFlights(end).sensory;
                TrackMonitorTests.TestEquality(testCase, actualSensory, ...
                    sensory, 1);
                steps = steps + 1;
            end
        end
        function RadarReportOnlyCorrectSingleStep(testCase)
            % RadarReportONlyCorrectSingleStep - testing that the function
            % can handle only sensory information encase their is no
            % telemetry being transmitting through a single step.
            monitor = TrackMonitor();
            sensory = TrackMonitorTests.GenerateRandomTelemetryData(1);
            telemetry = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            actualSensory = monitor.classifiedFlights(end).sensory;
            TrackMonitorTests.TestEquality(testCase, actualSensory, ...
                sensory, 1);
        end
        function RadarReportOnlyCorrectTwoSteps(testCase)
            % RadarReportOnlyCorrectTwoSteps - Testing that the function
            % can handle sensory information only through a couple of steps
            % of the simulation.
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 2)
                sensory = TrackMonitorTests.GenerateRandomTelemetryData(1);
                telemetry = TrackMonitorTests.GenerateEmptySensory();
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                actualSensory = monitor.classifiedFlights(end).sensory;
                TrackMonitorTests.TestEquality(testCase, actualSensory, ...
                    sensory, 1);
                steps = steps + 1;
            end
        end
        function RadarReportOnlyMultipleSteps(testCase)
            % RadarReportOnlyMultipleSteps - Testing that the function can
            % handle sensory information only through multiple steps of the
            % simulation.
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 10)
                sensory = TrackMonitorTests.GenerateRandomTelemetryData(1);
                telemetry = TrackMonitorTests.GenerateEmptySensory();
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                actualSensory = monitor.classifiedFlights(end).sensory;
                TrackMonitorTests.TestEquality(testCase, actualSensory, ...
                    sensory, 1);
                steps = steps + 1;
            end
        end
        function noTelemetryAndRadarNoStep(testCase)
            % noTelemetryAndRadarNoStep - Tests that everything is update
            % based on no receiving any information
            monitor = TrackMonitor();
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(sensory, sensory, [],1);
            Flights = monitor.classifiedFlights;
            testCase.verifyEqual(1, size(Flights, 1));
        end
    end

    % Clustering Tests
    methods(Test)
        function singleUASNoRadarInformation(testCase)
            %singleUASNoRadarInformation - Ensures that the clustering
            % information can work when there is only one UAS transmitting
            % information.
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            testCase.verifyEqual(2, size(Flights, 2));
        end
        function singleUASWithRadarInformation(testCase)
            % SingleUASWithRadarInformation - Ensures that the clustering
            % information can work when there is only one UAS transmitting
            % information and contains sensory information.
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
            sensory = telemetry;
            sensory.pos = telemetry.pos + mvnrnd([0,0,0], eye(3)*.5);
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            testCase.verifyEqual(2, size(Flights, 2));
        end
        function singleUASNoRadarMultipleStepsClustering(testCase)
            % singleUASNoRadarMultipleStepsClustering - ensures that the
            % clustering method can detect that a single uas without any
            % sensory information through multiple steps
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 10)
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
                sensory = TrackMonitorTests.GenerateEmptySensory();
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                Flights = monitor.classifiedFlights;
                testCase.verifyEqual(2, size(Flights, 2));
                steps = steps + 1;
            end
        end
        function singleUASWithRadarTwoStepsClustering(testCase)
            % singleUASWithRadarTwoStepsClustering - tests that the
            % clustering method will cluster the radar and telemetry
            % information from a single UAS object through two steps.
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 2)
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
                sensory = telemetry;
                sensory.pos = telemetry.pos + mvnrnd([0,0,0], eye(3)*.5);
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                Flights = monitor.classifiedFlights;
                testCase.verifyEqual(2, size(Flights, 2));
                steps = steps + 1;
            end
        end
        function singleUASWithMultipleStepsClustering(testCase)
            % singleUASWithMultipleStepsClustering - Ensures that the
            % clustering method will cluster the radar and telemetry
            % information through multiple steps
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 10)
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
                sensory = telemetry;
                sensory.pos = telemetry.pos + mvnrnd([0,0,0], eye(3)*.5);
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                Flights = monitor.classifiedFlights;
                testCase.verifyEqual(2, size(Flights, 2));
                steps = steps + 1;
            end
        end
        function TwoUASNoRadarInformation(testCase)
            % TwoUASNoRadarInformation - Ensures that the clustering
            % information can work with two UAS in differing lanes are
            % transmitting information without sensory information.
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(2);
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            testCase.verifyEqual(3, size(Flights, 2));
        end
        function TwoUASLanesWithRadarInformation(testCase)
            % TwoUASDifferentLanesWithRadarInformation - Ensures that the
            % clustering information can work with two UAS in differing
            % lanes are transmitting information with sensory information.
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(2);
            sensory = telemetry;
            sensory.pos(1, :) = telemetry.pos(1, :) + mvnrnd([0,0,0], eye(3)*.5);
            sensory.pos(2, :) = telemetry.pos(2, :) + mvnrnd([0,0,0], eye(3)*.5);
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            testCase.verifyEqual(3, size(Flights, 2));
        end
        function TwoUASMultipleStpesWithoutRadar(testCase)
            % StressTestUASOnly - Ensures that the clustering informaiton
            % can work with multiple UAS transmitting their information
            % without sensory information.
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 10)
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(2);
                sensory = TrackMonitorTests.GenerateEmptySensory();
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                Flights = monitor.classifiedFlights;
                testCase.verifyEqual(3, size(Flights, 2));
                steps = steps + 1;
            end
        end
        function StressTestWithoutClustering(testCase)
            % StressTEstUASRadar - Ensures that the clustering information
            % can work with multiple UAS transmitting along with sensory
            % information being detected.
            rng(0);
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 10)
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(9);
                sensory = TrackMonitorTests.GenerateEmptySensory();
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                Flights = monitor.classifiedFlights;
                testCase.verifyEqual(10, size(Flights, 2));
                steps = steps + 1;
            end
        end
        function StressTestWithRadarClustering(testCase)
            % StressTestWithoutRadarClustering - tests multiple uas without
            % sensory information ensuring that the clustering is working
            % correctly.
            monitor = TrackMonitor();
            steps = 0;
            while(steps < 10)
                telemetry = TrackMonitorTests.GenerateRandomTelemetryData(9);
                sensory = telemetry;
                for index = 1:9
                    sensory.pos(index, :) = telemetry.pos(index, :) ...
                        + mvnrnd([0,0,0], eye(3)*.5);
                end
                monitor.AnalyzeFlights(telemetry, sensory, [],1);
                Flights = monitor.classifiedFlights;
                testCase.verifyEqual(10, size(Flights, 2));
                steps = steps + 1;
            end
        end
    end

    % Tracker linker tests
    methods(Test)
        function doesntDeactiveAfterOnemissedStep(testCase)
            % doesntDeactiveAfterOnemissedStep - testing that the tracker
            % doesn't deactive when one step is missed.
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            monitor.AnalyzeFlights(sensory, sensory, [], 1);
            tracker = monitor.tackers;
            testCase.verifyTrue(tracker.active);
        end
        function doesDeactiveAfterTwoMissedSteps(testCase)
            % doesDeactiveAfterTwoMissedSteps - testing that the tracker
            % becomes inactive after not being updated in two steps.
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(1);
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            monitor.AnalyzeFlights(sensory, sensory, [], 1);
            monitor.AnalyzeFlights(sensory, sensory, [], 1);
            tracker = monitor.tackers;
            testCase.verifyTrue(~tracker.active);
        end
        function singleUASOneStepTracker(testCase)
            % singleUASOneStepTracker - Ensures that the number of trackers
            % equals the number of UAS in the simulation. 
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            uas = sim.uas_list(1);
            sim.uas_list = uas;
            
            for i = 1:num_steps
                uas_step = uas.stepTrajectory();
                if uas.active
                    pos = uas.exec_traj;
                    if ~isempty(pos)
                        uas.gps.lon = pos(uas_step, 1);
                        uas.gps.lat = pos(uas_step, 2);
                        uas.gps.alt = pos(uas_step, 3);
                        uas.gps.commit();
                        traj = uas.exec_traj;
                        set(uas.h, 'XData', traj(:,1), ...
                            'YData', traj(:,2), ...
                            'ZData', traj(:,3));
                        sim.step(1);
                        [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables([uas], sim);
                        monitor.AnalyzeFlights(telemetry, radars, [], sim.tick_del_t);
                        trackers = monitor.tackers;
                        testCase.verifyEqual(1, length(trackers));
                        sim.atoc.createRadarTelemetryData();
                        break;
                    end
                end
            end
        end
        function singleUASTwoStepTracker(testCase)
            % singleUASTwoStepTracker - Ensures that the number of trackers
            % are equal to the number of uas during two steps
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            uas = sim.uas_list(1);
            sim.uas_list = uas;
            counter = 0;
            
            for i = 1:num_steps
                uas_step = uas.stepTrajectory();
                if uas.active
                    pos = uas.exec_traj;
                    if ~isempty(pos)
                        uas.gps.lon = pos(uas_step, 1);
                        uas.gps.lat = pos(uas_step, 2);
                        uas.gps.alt = pos(uas_step, 3);
                        uas.gps.commit();
                        traj = uas.exec_traj;
                        set(uas.h, 'XData', traj(:,1), ...
                            'YData', traj(:,2), ...
                            'ZData', traj(:,3));
                        sim.step(1);
                        [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables([uas], sim);
                        monitor.AnalyzeFlights(telemetry, radars, [],sim.tick_del_t);
                        trackers = monitor.tackers;
                        testCase.verifyEqual(1, length(trackers));
                        sim.atoc.createRadarTelemetryData();
                        if(counter < 1)
                            counter = counter + 1;
                        else
                            i = num_steps;
                            break;
                        end
                    end
                end
            end
        end
        function singleUASMultipleStepTracker(testCase)
            % singleUASMultipleStepTracker - Ensures that the number of
            % tracker objects are equal to the number of uas during the
            % simulation.
            rng(0);
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            uas = sim.uas_list(1);
            sim.uas_list = uas;

            for i = 1:num_steps
                uas_step = uas.stepTrajectory();
                if uas.active
                    pos = uas.exec_traj;
                    if ~isempty(pos)
                        uas.gps.lon = pos(uas_step, 1);
                        uas.gps.lat = pos(uas_step, 2);
                        uas.gps.alt = pos(uas_step, 3);
                        uas.gps.commit();
                        traj = uas.exec_traj;
                        set(uas.h, 'XData', traj(:,1), ...
                            'YData', traj(:,2), ...
                            'ZData', traj(:,3));
                        sim.step(1);
                        [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables([uas], sim);
                        monitor.AnalyzeFlights(telemetry, radars, [],sim.tick_del_t);
                        trackers = monitor.tackers;
                        testCase.verifyEqual(length(trackers),1);
                        sim.atoc.createRadarTelemetryData();
                    end
                end
            end
        end
        function TwoUASTwoStepTracker(testCase)
            % TwoUASTwoSTepTracker - Ensures that the number of trackers
            % are equal to the maximum number of uas objects during the
            % simulation.
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            sim.uas_list = [sim.uas_list(1); sim.uas_list(2)];
            stepCounter = 0;
            
            for i = 1:num_steps
                activeFlights = 0;
                activeuas = [];
                for j = 1:2
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        activeuas = [activeuas, uas];
                        activeFlights = activeFlights + 1;
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                        end
                    end
                end
                sim.step(1);
                [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables(activeuas, sim);
                monitor.AnalyzeFlights(telemetry, radars, [],sim.tick_del_t);
                trackers = monitor.tackers;
                if(~isempty(trackers))
                    testCase.verifyEqual(1, length(trackers));
                    sim.atoc.createRadarTelemetryData();
                    trackers = monitor.tackers;
                    testCase.verifyTrue(2 >= length(trackers));
                    sim.atoc.createRadarTelemetryData();
                end

                if(stepCounter < 2)
                    stepCounter = stepCounter + 1;
                else
                    i = num_steps;
                    break;
                end
            end
        end
        function TwoUASMultipleStepTracker(testCase)
            % TwoUASOneStepTracker - Ensures that the number of trackers
            % are no more than the number of uas during multiple steps.
            rng(0);
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            sim.uas_list = [sim.uas_list(1); sim.uas_list(2)];
            
            for i = 1:num_steps
                activeFlights = 0;
                activeuas = [];
                radars = [];
                tnew = table();
                tnew.ID = "";
                tnew.pos = zeros(1,3);
                tnew.speed = zeros(1,3);
                tnew.time = 0;
                telemetry = tnew;
                index = 1;
                for j = 1:2
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active        
                        activeFlights = activeFlights + 1;
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            telemetry{index, {'ID', 'pos', 'speed', 'time'}} = ...
                                [uas.id, [uas.gps.lon, uas.gps.lat, uas.gps.alt], ...
                                [uas.gps.vx, uas.gps.vy, uas.gps.vz], 0];
                            index = index + 1;
                        end
                    end
                end
                sim.step(1);
                if(activeFlights == 2)
                    disp("break");
                end
                radars = TrackMonitorTests.MakeTables(sim.radar_list);
                monitor.AnalyzeFlights(telemetry, radars, [], sim.tick_del_t);
                trackers = monitor.tackers;
                if(~isempty(trackers))
                    testCase.verifyTrue(2 >= length(trackers));
                    sim.atoc.createRadarTelemetryData();
                end
            end
        end
        function ThreeUASMultipleStepTracker(testCase)
            % TwoUASOneStepTracker - Ensures that the number of trackers
            % are no more than the number of uas during multiple steps.
            rng(0);
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            sim.uas_list = [sim.uas_list(1); sim.uas_list(3);sim.uas_list(5)];
            
            for i = 1:num_steps
                activeFlights = 1;
                activeuas = [];
                radars = [];
                for j = 1:2
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        activeFlights = activeFlights + 1;
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            activeuas = [activeuas, uas];
                        end
                    end
                end
                sim.step(1);
                [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables(activeuas, sim);
                monitor.AnalyzeFlights(telemetry, radars, [],sim.tick_del_t);
                trackers = monitor.tackers;
                if(~isempty(trackers))
                    testCase.verifyTrue(3 >= length(trackers));
                    sim.atoc.createRadarTelemetryData();
                end
            end
        end
        function FourUASMultipleStepTracker(testCase)
            % TwoUASOneStepTracker - Ensures that the number of trackers
            % are no more than the number of uas during multiple steps.
            rng(0);
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            sim.uas_list = [sim.uas_list(1); sim.uas_list(3);...
                sim.uas_list(5); sim.uas_list(7)];
            
            for i = 1:num_steps
                activeFlights = 1;
                activeuas = [];
                radars = [];
                for j = 1:2
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        
                        activeFlights = activeFlights + 1;
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            activeuas = [activeuas, uas];
                        end
                    end
                end
                sim.step(1);
                [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables(activeuas, sim);
                monitor.AnalyzeFlights(telemetry, radars, [],sim.tick_del_t);
                trackers = monitor.tackers;
                if(~isempty(trackers))
                    testCase.verifyTrue(4 >= length(trackers));
                    sim.atoc.createRadarTelemetryData();
                end
            end
        end
        function FiveUASMultipleStepTracker(testCase)
            % TwoUASOneStepTracker - Ensures that the number of trackers
            % are no more than the number of uas during multiple steps.
            rng(0);
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            sim.uas_list = [sim.uas_list(1); sim.uas_list(3);...
                sim.uas_list(5); sim.uas_list(7);sim.uas_list(9)];
            
            for i = 1:num_steps
                activeFlights = 1;
                activeuas = [];
                radars = [];
                for j = 1:2
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        
                        activeFlights = activeFlights + 1;
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            activeuas = [activeuas, uas];
                        end
                    end
                end
                sim.step(1);
                [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables(activeuas, sim);
                monitor.AnalyzeFlights(telemetry, radars, [],sim.tick_del_t);
                trackers = monitor.tackers;
                if(~isempty(trackers))
                    testCase.verifyTrue(5 >= length(trackers));
                    sim.atoc.createRadarTelemetryData();
                end
            end
        end
       
        function TrackerIDCorrectTwoSteps(testCase)
            % TrackerIDCorrectTwoSteps - Ensures that the number ID is the
            % same throughout two steps for a single uas. 
            monitor = TrackMonitor();
            [sim, ~, num_steps] = ...
                TrackMonitorTests.setUpSimulationFlights();
            uas = sim.uas_list(1);
            sim.uas_list = uas;
            stepCounter = 0;
            
            for i = 1:num_steps
                uas_step = uas.stepTrajectory();
                if uas.active
                    pos = uas.exec_traj;
                    if ~isempty(pos)
                        uas.gps.lon = pos(uas_step, 1);
                        uas.gps.lat = pos(uas_step, 2);
                        uas.gps.alt = pos(uas_step, 3);
                        uas.gps.commit();
                        traj = uas.exec_traj;
                        set(uas.h, 'XData', traj(:,1), ...
                            'YData', traj(:,2), ...
                            'ZData', traj(:,3));
                        sim.step(1);
                        [telemetry, radars] = ...
                            TrackMonitorTests.MakeTables([uas], sim);
                        monitor.AnalyzeFlights(telemetry, radars, [], sim.tick_del_t);
                        track_ID = monitor.classifiedFlights(end).ID;
                        testCase.verifyEqual('0', track_ID);
                        sim.atoc.createRadarTelemetryData();
                        if(stepCounter < 1)
                            stepCounter = stepCounter + 1;
                        else
                            i = num_steps;
                            break;
                        end
                    end
                end
            end
        end
        function TwoUASIDCorrectTwoStep(testCase)
            % TwoUASIDCorrectTwoStep - Ensures that the tracker ID is
            % correctly linked to the correct UAS given two steps in the
            % simulation. 
            monitor = TrackMonitor();
            firstIDS = [];
            secondIDS = [];
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(2);
            
            sensory = telemetry;
            sensory.pos(1, :) = telemetry.pos(1, :) + ...
                mvnrnd([0,0,0], eye(3)*.5);
            sensory.pos(2, :) = telemetry.pos(2, :) + ...
                mvnrnd([0,0,0], eye(3)*.5);
            
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            for index = 2:size(Flights, 2)
                firstIDS = [firstIDS; ...
                    Flights(index).telemetry.ID, Flights(index).ID];
            end
            telemetry.pos(1, :) = telemetry.pos(1, :) + telemetry.speed(1, :)/3;
            telemetry.pos(2, :) = telemetry.pos(2, :) + telemetry.speed(2, :)/3;
            sensory.pos(1, :) = sensory.pos(1, :) + telemetry.speed(1, :)/3;
            sensory.pos(2, :) = sensory.pos(2, :) + telemetry.speed(2, :)/3;
            
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            for index = 2:size(Flights, 2)
                secondIDS = [secondIDS; ...
                    Flights(index).telemetry.ID, Flights(index).ID];
            end

            testCase.verifyEqual(size(firstIDS,1), size(secondIDS,1));
            testCase.verifyEqual(firstIDS(1,1),secondIDS(1,1));
            testCase.verifyEqual(firstIDS(1,2),secondIDS(1,2));
            testCase.verifyEqual(firstIDS(2,1),secondIDS(2,1));
            testCase.verifyEqual(firstIDS(2,2),secondIDS(2,2));
        end
        function ThreeUASIDCorrectCoupleOfStep(testCase)
            % ThreeUASIDCorrectSingleStep - Ensures that the tracker ID is
            % correctly linked to the correct UAS given a single step in
            % the simulation between three UAS.
            monitor = TrackMonitor();
            firstIDS = [];
            secondIDS = [];
            thirdIDS = [];
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData(3);
            
            sensory = telemetry;
            sensory.pos(1, :) = telemetry.pos(1, :) + ...
                mvnrnd([0,0,0], eye(3)*.5);
            sensory.pos(2, :) = telemetry.pos(2, :) + ...
                mvnrnd([0,0,0], eye(3)*.5);
            sensory.pos(3, :) = telemetry.pos(3, :) + ...
                mvnrnd([0,0,0], eye(3)*.5);
            
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            for index = 2:size(Flights, 2)
                firstIDS = [firstIDS; ...
                    Flights(index).telemetry.ID, Flights(index).ID];
            end
            
            telemetry.pos(1, :) = telemetry.pos(1, :) + telemetry.speed(1, :)/3;
            telemetry.pos(2, :) = telemetry.pos(2, :) + telemetry.speed(2, :)/3;
            telemetry.pos(3, :) = telemetry.pos(3, :) + telemetry.speed(3, :)/3;
            sensory.pos(1, :) = sensory.pos(1, :) + telemetry.speed(1, :)/3;
            sensory.pos(2, :) = sensory.pos(2, :) + telemetry.speed(2, :)/3;
            sensory.pos(3, :) = sensory.pos(3, :) + telemetry.speed(3, :)/3;
            
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            for index = 2:size(Flights, 2)
                secondIDS = [secondIDS; ...
                    Flights(index).telemetry.ID, Flights(index).ID];
            end
            
            telemetry.pos(1, :) = telemetry.pos(1, :) + telemetry.speed(1, :)/3;
            telemetry.pos(2, :) = telemetry.pos(2, :) + telemetry.speed(2, :)/3;
            telemetry.pos(3, :) = telemetry.pos(3, :) + telemetry.speed(3, :)/3;
            sensory.pos(1, :) = sensory.pos(1, :) + telemetry.speed(1, :)/3;
            sensory.pos(2, :) = sensory.pos(2, :) + telemetry.speed(2, :)/3;
            sensory.pos(3, :) = sensory.pos(3, :) + telemetry.speed(3, :)/3;
            
            monitor.AnalyzeFlights(telemetry, sensory, [],1);
            Flights = monitor.classifiedFlights;
            for index = 2:size(Flights, 2)
                thirdIDS = [thirdIDS; ...
                    Flights(index).telemetry.ID, Flights(index).ID];
            end

            testCase.verifyEqual(size(firstIDS,1), size(secondIDS,1));
            testCase.verifyEqual(size(firstIDS,1), size(thirdIDS,1));
            testCase.verifyEqual(firstIDS(1,1),secondIDS(1,1));
            testCase.verifyEqual(firstIDS(1,2),secondIDS(1,2));
            testCase.verifyEqual(firstIDS(2,1),secondIDS(2,1));
            testCase.verifyEqual(firstIDS(2,2),secondIDS(2,2));
            testCase.verifyEqual(firstIDS(1,1),thirdIDS(1,1));
            testCase.verifyEqual(firstIDS(2,2),thirdIDS(2,2));
        end
    end

end