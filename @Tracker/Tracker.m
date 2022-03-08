classdef Tracker < handle
    % Tracker - Uses a Kalman Filter to track objects within the simulation
    % by using the sensory and telemetry information from the object.

    properties
        pos % The current Position/velocity of the Object being tracked.
        ID % Identification of the tracker
        traj % Predicted Trajectory information
        active % Indicates if there uas is active currently
    end

    properties(Access=private)
        P % Process Covariance matrix
        A % Position/Velocity Transformation Matrix
        B % Acceleration Transformation Position Matrix
        y % Observeration Data Gathered For Specific Simulation Step
        vel % Current Velocity
        changed % Indication of change in observeration
        updated % Boolean to help deactive the Tracker Object
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
            obj.vel = zeros(3,1);
            obj.y = [];
            obj.changed = false;
            obj.updated = true;
        end
        function start_update(obj, src, event)
            if event.EventName == "UpdateModel"
                obj.UpdateModel(src.del_t);
            end
        end

        function UpdateModel(obj, del_t)
            % UpdateModel - Predicts the next position given the next
            % simulation step. 
            % Input:
            %   telemetry (table) : Telemetry information
            %   sensory (table) : sensory informaiton
            %   del_t (float) : The change in time;

            if(~obj.changed)
                if(obj.updated)
                    obj.updated = false;
                else
                    obj.active = false;
                end
                return;
            end
            
            acceration = (obj.vel - obj.pos(4:6))/del_t;
            if(all(isnan(acceration)))
                acceration = ones(3,1);
            end

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
            z = mvnrnd([0,0,0,0,0,0], eye(6)*.5);
            obj.pos = eye(6)*obj.y + transpose(z);

            % Calculate the current state
            obj.pos = obj.pos + k*(obj.y - obj.pos);

            % Update the Process Covariance matrix
            obj.P = (eye(6) - k)*obj.P;
            obj.changed = false;
            obj.traj = [obj.traj; transpose(obj.pos)];
        end

        function RecieveObservationData(obj, telemetry, sensory)
            % CalculateAverageInformation - Calculate the mean observed
            % position from the sensory information for the kalman filter
            % Input
            %   sensory (table): Sensory information tied to the Object
            %   being tracked
            cur_y = zeros(1,6);

            if(~isempty(sensory))
                obs = height(sensory);
                cur_y = zeros(obs, 6);
                cur_y(1:obs, :) = [sensory.pos, sensory.speed];
            end

            if(~isempty(telemetry))
                cur_y = [cur_y; telemetry.pos, telemetry.speed];
                obj.vel = transpose(telemetry.speed);
            end
            
            if(size(cur_y, 1) > 1)
                obj.y = transpose(mean(cur_y));
            else
                obj.y = transpose(cur_y);
            end

            obj.changed = true;
            obj.updated = true;
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

