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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
            monitor.AnalyzeFlights(sensory, sensory, []);
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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
            monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
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
                monitor.AnalyzeFlights(telemetry, sensory, []);
                Flights = monitor.classifiedFlights;
                testCase.verifyEqual(10, size(Flights, 2));
                steps = steps + 1;
            end
        end
    end

    % Tracker linker tests
    methods(Test)
        function singleUASOneStepTracker(testCase)
            % singleUASOneStepTracker - Ensures that the linking between a
            % single UAS with on step is tied correctly to the same tracker
        end

        function singleUASTwoStepTracker(testCase)
            % singleUASTwoStepTracker - Ensures that the linking between a
            % single UAS over two steps is correctly tied to the same
            % tracker object. 
        end

        function singleUASMultipleStepTracker(testCase)
            % singleUASMultipleStepTracker - Ensures that the linking
            % between a single UAS over multiple steps is correctly tied to
            % the same tracker object. 
        end

        function TwoUASOneStepTracker(testCase)
            % TwoUASOneStepTracker - Ensures that the linking between two
            % UAS objects are correctly tied to the correct tracker object
            % over one step
        end

        function TrakerIDCorrectSingleStep(testCase)
            % TrackerIDCorrectSingleStep - Ensures that the tracker ID is
            % correctly updated through a single step.
        end

        function TrackerIDCorrectTwoSteps(testCase)
            % TrackerIDCorrectTwoSteps - Ensures that the tracker ID is
            % correctly updated through two steps.
        end
        
        function TrackerIDCorrectlyMultipleSteps(testCase)
            % TrackerIDCorrectlyMultipleSteps - Ensures that the tracker ID
            % is correctly updated through multiple steps. 
        end
        
        function TwoUASTwoStepTracker(testCase)
            % TwoUASTwoSTepTracker - Ensures that the linking between two
            % UAS objects are correctly tied to the correct tracker object
            % over two steps
        end

        function TwoUASMultipleStepTracker(testCase)
            % TwoUASMultipleStepTracker - Ensures that the linking between
            % two UAS objects are correctly tied to the correct tracker
            % object over multiple steps. 
        end

        function StressTestTracker(testCase)
            % StressTestTracker - Ensures that the linking between multiple
            % UAS objects are correctly tied to the correct tracker object
            % over multiple flight plans. This is a stress test. 
        end
    end

end