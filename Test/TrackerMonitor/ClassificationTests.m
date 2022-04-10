%% Track Monitor Unit/Load/Integeration Tests For Classification of Flights
% This class is used to test the Track Monitor class to ensure that the
% classification of flights and the setup for this is working properly.

classdef ClassificationTests < matlab.unittest.TestCase
    properties
        lbsd
        monitor
    end

    methods(TestClassSetup)
        function attachFilesForTestClass(testCase)
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
        function noHobbie2Tests(testCase)
            % noHobbie2Tests - This test ensures that no hobbist detection
            % was given the waypoints. 
        end

        function hobbie2TestsSingle(testCase)
            % hobbie2TestsSingle - This test ensures that hobbist 2
            % detection for a single flight. 
        end

        function hobbie2TestsDoubleFlightsSameTime(testCase)
            % hobbie2TestsDoubleFlightsSameTime - this test ensures that
            % the hobbist 2 detection correctly given the two UAS Flights
            % one hobbist 2 and the other normal.
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