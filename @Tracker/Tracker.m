classdef Tracker < handle
    % Tracker - Tracks individual uas by using kalman model.

    properties
        P % Process Covariance matrix
        A % Position/Velocity Transformation Matrix
        B % Acceleration Transformation Position Matrix
        pos % The current Position of the Object being tracked.
    end

    methods
        function obj = Tracker()
        % Tracker - A constructor to create a tracker object, which uses a
        % kalman filter to predict the next position of the object.
        % Input:
        %   del_t (float): Simulation time step
        %
            pt = [1;1;1;2;2;2]; 
            obj.P =  pt*transpose(pt);
            obj.A = eye(6,6);
        end

        function UpdateModel(obj, telemetry, sensory, del_t)
            % UpdateModel - Predicts the next position given the next
            % simulation step. 
            % Input:
            %   telemetry (table) : Telemetry information
            %   sensory (table) : sensory informaiton

            acceration = (telemetry.velocity - obj.pos(4:6))/del_t;

            % Calculate the Kalman Gain
            partA = eye(3)*.006*del_t;
            partB = eye(3) * 1.27;
            R = [partA, zeros(3,3); zeros(3,3), partB];
            k = obj.P/(obj.P + R);

            % Grab Observeration Data
            y = obj.ObserverationPosition(telemetry, sensory);

            % Calculate the current state
            x = obj.pos + k*(y - obj.pos);

            % Update the Process Covariance matrix
            obj.P = (I - k)*obj.P;

            % New Predicted State
            obj.PredictNextState(del_t, x,acceration);

            % Calculate the Process Matrix
            obj.P = obj.A * Obj.P * transpose(obj.A);

        end

        function obj.PredictNextState(obj, del_t, prev_pos,accerlation)
            % PredictNextState - Predicts the next position 
            % Input:
            %   del_t (float) - change in time from simulation steps
            %
            
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
        function ObserverationPosition(obj, telemetry, sensory)
            % ObserverationPosition - This method is used to caluclate the
            % observed space. 
            obs = height(telemetry) + height(sensory);
            C = ones(obs, obs)*1/obs;
            y = [telemetry.pos(1); telemetry.pos(2); telemetry.pos(3); ...
                telemetry.velocity(1); telemetry.velocity(2); ...
                telemetry.velocity(3)];
            z = ones(6,1) * 1.2;
            obj.pos = C*y + z;
        end
    end
end

