%% Track Monitor Unit/Load/Integeration Tests For Classification of Flights
% This class is used to test the Track Monitor class to ensure that the
% classification of flights and the setup for this is working properly.

classdef ClassificationTests < matlab.unittest.TestCase

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
    end

    %% Nominally Tests
    % This section is used to test that a flight is nominally rather than
    % a Hobby or Rogue flight. These flights include slightly off course,
    % headway distances are not meet, etc. 

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
    end

    %% Hobby 3 Tests
    % This section is used to test flights that display Hobbyist Type 3:
    % Flies up in a circular motion to some highest point then flies down
    % in a circular motion to land.

    methods(Test)
    end

    %% Rogue 1 Tests
    % This section is used to test flights that display Rogue Type 1: Flies
    % up over and down as for a delivery.

    methods(Test)
    end

    %% Rogue 2 Tests
    % This section is used to test flights that display Rogue Type 2: Flies
    % up to a lane, flies along the lane to the end, then flies to another
    % lane, and eventually flies down to land.

    methods(Test)
    end
end