classdef ATOC < handle
    % ATOC - Monitors UAS Activity During Simulations
    %   Monitors: Density in Lanes, Planned Routes, UAS Positions, and
    %   Sensory Data.
    %   Displays: Density/time graphs and Space-Time Lane Diagrams.
    
    properties
        reseverationData  % Reseveration Data (planned flight data)
        telemetryData % Drone Position Data
        laneData % lane Informaiton
        radars % Radar Information
        time % Keep track of time
    end
    
    % User Accessible Functions
    methods
        
        function obj = ATOC(res, range, noise, angle, time)
            % ATOC - Constructor to Monitor UAS activity
            % Input:
            %   res (results struct)
            %       .airways (airway struct)
            %       .reservations (reservation struct)
            %       .flights (flights struct)
            %   range (float): radars sensor range
            %   noise (3x3 matrix): noise in the radar sensors
            %   angle (float): The radar's beamwidth
            %   time (float): starting time
            obj.telemetryData = containers.Map('KeyType', 'string', ...
                'ValueType', 'any');
            obj.laneData = containers.Map('KeyType', 'string', ...
                'ValueType', 'any');
            obj.reseverationData = res;
            obj.radars = LEM_radars_placement_coverage(res, range, ...
                noise, angle);
            obj.time = time;
            createlaneData(res.airways);
        end
        
        % Needs to Be completed
        function aDensity = averageDensity(obj, plot, time, lanes)
            % Average Density/time Graph Creation
            if plot == 1
                % Show the average density graph
                if ~isEmpty(time)
                    % Show the selected time selection
                else
                    % Show all time
                end
                if ~isEmpty(lanes)
                    % Show the selected lanes/density lane graph
                else
                    % Show across all lanes
                end
            end
            % Return average Density given the time interval
            % Have to calculate the number of lanes/drones.
            return lanesnum/dronesnum;
        end
        
        % Needs to Be completed
        function denseList = mostDense(obj, number, time)
            % Returns the specified number of most dense lanes
            if isEmpty(number)
                % Return the most dense lane
            end
            
            if isEmpty(time)
                % return the most dense throughout the simulation
            end
            
        end
        
        % Find way to update Flights instead of plotting each time/
        % Find a way to show connected lanes together on same graph.
        function laneTrajectory(obj, lanes, time)
           % laneTrajectory - shows the planned/actual/sensory data per
           %    lane
           % Input:
           %    lanes (1 x n array): lane indexes 
           %    time (1 x 2): the start and end time, empty for totalTime
           for lane = 1:length(lanes)
               laneIndex = lanes(lane); 
               lane_length = ...
                   obj.reseverationData.airways.lane_lengths(laneIndex);
               lane_flights = ...
                   obj.reseverationData.reservations(laneIndex).flights;
               specificLane = obj.laneData(lane).UAS{:, :}; % Grabs all 
               if(~isempty(time)) % Wanted to see a specific time range
                   [rows, ~] = find(lane_flights(:, 2) >= time(1) & ...
                       lane_flights(:,3) <= time(2));
                   lane_flights = lane_flights(rows, :);
                   [rows, ~] = find(specificLane(:,3) >= time(1) & ...
                       specificLane(:,3) <= time(2));
                   specificLane = specificLane(rows, :);
               end
               figure;
               plot(min(lane_flights(:, 2)),0, 'w.'); hold on;
               plot(max(lane_flights(:,3)), lane_length, 'w.');
               for s = 1:size(lane_flights, 1) % Plot Lane Trajectory
                   plot(lane_flights(s, 2:3), [0, lane_length], 'k');
               end
               for d = 1:size(specificLane, 1)
                   plot(specificLane(d, 2), specificLane(d, 3));
               end         
           end
        end
        
        % Needs to Be completed
        function averDev = averageDev(obj, drones, time)
            if ~isEmpty(time)
                % Only find it for the specfied Time interval
            end
            % Calculates the average deviation between the
            % sensory/drone from planned
            % Can use LEM_dist_line_line(line1, line2) - to calculate the
            % different between actual and planned.
        end
        
        % Needs to be debugged/Radar Added
        function handle_events(obj, src, event)
            % handle_events - handles any listening event during simulation
            if event.EventName == "Tick"
                obj.time = src.time;
            end
            if event.EventName == "reseveration"
                obj.reseverationData = src.reseverationData;
            end
            if event.EventName == "telemetry"
                id = src.id;
                flightID = src.flightNum; % Lane Index
                droneData = [src.x, src.y, src.z, src.v];
                laneNum = src.lane;
                obj.updateUAS(id, flightID, droneData);
                obj.updateLane(laneNum, src);
            end
        end
    end
    
    % Helper Private Functions
    methods (Access = private)
        % Needs to be debugged
        function updateUAS(obj, id, flightID, UASData)
            % updateUAS - updates the status of the drones
            % Input:
            %   id (float) - UAS Identification Number
            %   flightID (float) - UAS flight number
            %   UASData (1x4 array) - UAS position and speed
            % Update the Drone Map Data - current position
            if (~obj.telemetryData(id)) % Add UAS to MAP
                droneInfo(flightID) = UASData;
                obj.telemetryData(id) = droneInfo;
            else % Update UAS Position
                value = obj.telemetryData(id);
                value(flightID).droneData(end + 1) = UASData;
                obj.telemetryData(id) = value;
            end
        end

        function updateLane(laneNumber, src)
            % updateLane - updates the UAS distance along the specific lane
            % Input:
            %   laneNumber (float) - The Specific Lane Index
            %   src (UAS Object) - The UAS Reporting Telemetry Data
            value = obj.laneData(laneNumber);
            lanes = value.pos;
            posLanes = [lanes(4) - lanes(1), lanes(5) - lanes(2), ...
                lanes(6) - lanes(3)];
            posUAS = [src.x - lanes(1), src.y - lanes(2),...
                src.z - lanes(3)];
            dis = projectUAS(posUAS, posLanes);
            value.UAS{end + 1, {'ID', 'Distance', 'Time'}} = [src.id, dis,...
                obj.time];
            obj.laneData(laneNumber) = value;
        end
        
        function createLaneData(airways)
        % createLaneData - initializes lane data
        % Input:
        %   airways (airway struct) - airway information
            for lane = 1:size(airways.lanes, 1)
                info = struct();
                sz = [1 3];
                varTypes = {'double', 'double', 'double'};
                varNames = {'ID', 'Distance', 'Time'};
                tnew = table('Size',sz,'VariableTypes',varTypes,...
                    'VariableNames',varNames);
                position = airways.lanes(lane, :);
                info.UAS = tnew;
                info.pos = position;
                obj.laneData(lane) = info;
            end
        end

        function dis = projectUAS(posUAS, posLane)
            % projectUAS - locates the UAS distance along the lane
            % Input:
            %   posUAS (1x3 array): Vec Coord UAS Position
            %   posLane (1x3 array): Vec Coord Lane Position
            % Output:
            %   dis (float): The UAS Distance Along The Lane
            dotProd = dot(posUAS, posLane);
            normLane = norm(posLane);
            dis = (dotProd/normLane);
        end
    end
end

