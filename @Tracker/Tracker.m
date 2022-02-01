classdef Tracker < handle
    % Tracker - Tracks individual uas by using kalman model.

    properties
        model % Kalman model for flight behaviors.
        pos % The current Position of the Object being tracked.
    end

    methods
        function obj = Tracker()
            % Create the kalman model
        end

        function handle_events(obj, event)
            % This function is to handle the update in the Kalman model
            % with the new information presented.
            if event.EventName == "UpdateModel"
            end
        end
    end
end

