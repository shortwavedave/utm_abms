classdef Tracker < handle
    % Tracker - Uses a Kalman Filter to track objects within the simulation
    % by using the sensory and telemetry information from the object.

    properties
        pos % The current Position/velocity of the Object being tracked.
    end

    properties(Access=private)
        P % Process Covariance matrix
        A % Position/Velocity Transformation Matrix
        B % Acceleration Transformation Position Matrix
    end

    %% Main Functions that Run the Kalman Filter
    methods
        function obj = Tracker(pos)
        % Tracker - A constructor to create a tracker object, which uses a
        % kalman filter to predict the next position of the object.
        % Input:
        %   pos(6x1 array): pos cordinates and velocity cordinates
        %
            pt = [1;1;1;2;2;2]; 
            obj.P =  pt*transpose(pt);
            obj.A = eye(6,6);
            obj.pos = pos;
        end
        function UpdateModel(obj, telemetry, sensory, del_t)
            % UpdateModel - Predicts the next position given the next
            % simulation step. 
            % Input:
            %   telemetry (table) : Telemetry information
            %   sensory (table) : sensory informaiton
            %   del_t (float) : The change in time;

            acceration = (transpose(telemetry.speed) - obj.pos(4:6))/del_t;

            % New Predicted State
            obj.PredictNextState(del_t, obj.pos, acceration);

            % Calculate the Process Matrix
            obj.P = obj.A * obj.P * transpose(obj.A);

            % Calculate the Kalman Gain
            partA = eye(3)*.006*del_t;
            partB = eye(3) * 1.27;
            H = eye(6);
            R = [partA, zeros(3,3); zeros(3,3), partB];
            k = (obj.P*H)/(H*obj.P*H + R);

            % Grab Observeration Data
            y = obj.CalculateAverageInformaiton(telemetry, sensory);
            z = mvnrnd([0,0,0, 0, 0,0], eye(6)*.5);
            obj.pos = eye(6)*y + transpose(z);

            % Calculate the current state
            obj.pos = obj.pos + k*(y - obj.pos);

            % Update the Process Covariance matrix
            obj.P = (eye(6) - k)*obj.P;
        end

        function y = CalculateAverageInformaiton(obj, telemetry, sensory)
            % CalculateAverageInformation - Calculate the mean observed
            % position from the sensory information for the kalman filter
            % Input
            %   sensory (table): Sensory information tied to the Object
            %   being tracked
            y = [];
            if(~isempty(sensory))
                obs = height(sensory);
                y = zeros(obs+1, 6);
                y(1:obs, :) = [sensory.pos, sensory.speed];
            end
            y(end, :) = [telemetry.pos, telemetry.speed];
            y = transpose(mean(y));
        end

        function PredictNextState(obj, del_t, prev_pos,accerlation)
            % PredictNextState - Predicts the next position 
            % Input:
            %   obj (tracker handle)
            %   del_t (float) - change in time from simulation steps
            %   prev_pos (6x1): x,y,z and vx,vy,vz from previous simulation
            %   accerlation (3x1): ax,ay,az for current time
            
            % Update the Pos/Vec Transformation Matrix
            obj.A = eye(6,6);
            obj.A(1,4) = del_t;
            obj.A(2,5) = del_t;
            obj.A(3,6) = del_t;
            
            % Update the accerlation Transformation Matrix
            partA = eye(3) * (del_t^2)/2;
            partB = eye(3) * del_t;
            obj.B = [partA; partB];

            obj.pos = obj.A*prev_pos + obj.B*accerlation;
        end
    end
end

