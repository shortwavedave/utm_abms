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
            telemetry = table();
            for row = 1:numTel
                telemetry.ID(row) = num2str(row);
                telemetry.pos(row) = randi(10, [1,3]);
                telemetry.speed(row) = randi(5, [1,3]);
                telemetry.time(row) = time;
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
                sensory.ID(row) = num2str(row);
                sensory.pos(row) = randi(10, [1,3]);
                sensory.speed(row) = randi(5, [1,3]);
                sensory.time(row) = time;
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
            testCase.verifyEqual(expected.ID, actual.ID(end));
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
            monitor.AnalyzeFlight(telemetry, sensory, []);
            actualTelemetry = monitor.classifiedFlights;
            TrackMonitorTests.TestEquality(testCase, actualTelemetry, ...
                telemetry, 1);
        end

        function TelemetryReportedCorrectTwoUAS(testCase)
            % TelemetryReportedCorrectTwoUAS - Ensures that the telemetry
            % information is being correctly updated for two UAS Object
            monitor = TrackMonitor();
            telemetry = TrackMonitorTests.GenerateRandomTelemetryData();
            sensory = TrackMonitorTests.GenerateEmptySensory();
            monitor.AnalyzeFlight(telemetry, sensory, []);
            actualTelemetry = monitor.classifiedFlights;

            % Testing the position should equal one another
            testCase.verifyTrue(actualTelemetry.pos(end, 1) == ...
                telemetry.pos(end, 1) || actualTelemetry.pos(end, 1) == ...
                telemetry.pos(1, 1));
            testCase.verifyTrue(actualTelemetry.pos(end, 2) == ...
                telemetry.pos(end, 2) || actualTelemetry.pos(end, 2) == ...
                telemetry.pos(1, 2));
            testCase.verifyTrue(actualTelemetry.pos(end, 3) == ...
                telemetry.pos(end, 3) || actualTelemetry.pos(end, 3) == ...
                telemetry.pos(1, 3));
        end

        function TelemetryReportedCorrectMultipleSteps(testCase)
            % TelemetryReportedCorrectMultipleSteps - Ensures that the
            % telemetry information is being correctly updated through
            % multiple steps of a flight path. 
        end

        function RadarReportCorrectOneSteps(testCase) 
            % RadarReportCorrectOneStep - Ensures that the sensory
            % information is being correctly updated through a single step.
            % 
        end

        function RadarReportCorrectTwoSteps(testCase)
            % RadarReportCorrectTwoSteps - Ensures that the sensory
            % information is being correctly updated through two steps
        end

        function RadarReportCorrectMultipleSteps(testCase)
            % RadarReportCorrectMultipleSteps - Ensures that the sensory
            % information is being correctly updated through multiple steps
            % 
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
    end

    % Clustering Tests
    methods(Test)
        function singleUASNoRadarInformation(testCase)
            %singleUASNoRadarInformation - Ensures that the clustering
            % information can work when there is only one UAS transmitting
            % information. 
        end

        function SingleUASWithRadarInformation(testCase)
            % SingleUASWithRadarInformation - Ensures that the clustering
            % information can work when there is only one UAS transmitting
            % information and contains sensory information. 
        end

        function TwoUASDifferentLanesNoRadarInformation(testCase)
            % TwoUASNoRadarInformation - Ensures that the clustering
            % information can work with two UAS in differing lanes are
            % transmitting information without sensory information. 
        end

        function TwoUASDifferentLanesWithRadarInformation(testCase)
            % TwoUASDifferentLanesWithRadarInformation - Ensures that the
            % clustering information can work with two UAS in differing
            % lanes are transmitting information with sensory information. 
        end

        function TwoUASInSameLaneWithoutRadarInformation(testCase)
            % TwoUASInSameLaneWithoutRadarInformation - Ensures that the
            % clustering information can work with two UAS in same lane
            % without any sensory information being transmitted. 
        end

        function TwoUASInSameLaneWithRadarInformation(testCase)
            % TwoUASInSameLaneWithRadarInformation - Ensures that the
            % clustering information can work with two UAS in same lane
            % with sensory transmittion. 
        end

        function StressTestUASOnly(testCase)
            % StressTestUASOnly - Ensures that the clustering informaiton
            % can work with multiple UAS transmitting their information
            % without sensory information. 
        end

        function StressTestUASRadar(testCase)
            % StressTEstUASRadar - Ensures that the clustering information
            % can work with multiple UAS transmitting along with sensory
            % information being detected. 
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