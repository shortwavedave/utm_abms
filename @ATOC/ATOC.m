classdef ATOC < handle
    % ATOC - Monitors UAS Activity During Simulation
    %   Monitors: Lane Density, Lane Deviations, Rogue Flights
    %   Tracks: UAS Telemetry Data, Radar Sensors
    
    properties
        lbsd  % Reseveration Data (planned flight data)
        laneData % lane Informaiton
        radars % Store Radar Objects or Radar handle?
        time % Keep track of time
    end
    
    methods (Static)
        radar = LEM_radars_placement_coverage(res,range, noise, angle)
    end
    
    % User Accessible Functions
    methods
        
        function obj = ATOC(lbsd, range, noise, angle)
            % ATOC - Constructor to Monitor UAS activity
            % Input:
            %   lbsd (LBSD Handle): LBSD handle from simulation
            %   range (float): radars sensor range
            %   noise (3x3 matrix): noise in the radar sensors
            %   angle (float): The radar's beamwidth
            obj.lbsd = lbsd;
            obj.radars = ATOC.LEM_radars_placement_coverage(lbsd, range, ...
                noise, angle); % Change radars to a class.
            obj.createLaneData();
        end       
        
        function laneGraphs(obj, lanes, time)
            % laneGraphs - displays lane space diagrams and change in speed
            %   and distance graphs for specific lanes and time interval
            % Input:
            %    lanes (1 x n array): lane indexes
            %    time (1 x 2 array): the start and end time, 
            %       empty for totalTime
            laneTrajectory(obj, lanes, time);
            speedvsdisGraph(obj, lanes, time);
        end
                
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
               
        function updateLane(obj, laneNumber, src)
            % updateLane - updates the UAS distance along the specific lane
            % Input:
            %   laneNumber (float) - The Specific Lane Index
            %   src (UAS Object) - The UAS Reporting Telemetry Data
            value = obj.laneData(laneNumber);
            value = updateTelemetry(value, src, laneNumber);
            obj.laneData(laneNumber) = value;
        end
        
        function createLaneData(obj)
            % createLaneData - creates a lane data structure
            lanes = obj.lbsd.getLaneIds();
            obj.laneData = containers.Map('KeyType', 'char', ...
                'ValueType', 'any'); % Initinializing/Declaring LaneData
            for l = 1:size(lanes, 1)
                info = struct();
                info.pos = obj.getPosition(lanes(l));
                sz = [1 2];
                varTypes = {'double', 'double'};
                varNames = {'Number', 'Time'};
                tnew = table('Size',sz,'VariableTypes',varTypes,...
                    'VariableNames',varNames);
                info.density = tnew;
                sz = [1 6];
                varTypes = {'string', 'double', 'double', 'double', ...
                    'double', 'double'};
                varNames = {'ID', 'pos', 'time', 'del_speed', 'del_dis',...
                    'projection'};
                tnew2 = table('Size',sz,'VariableTypes',varTypes,...
                    'VariableNames',varNames);
                info.telemetry = tnew2;
                obj.laneData(lanes(l)) = info;
            end
        end
        
        function pos = getPosition(obj, laneIndex)
            % getPosition - Obtains the starting and ending vertexes
            %   positions
            % Input
            %   laneIndex (string): The lane index
            ids = obj.lbsd.getLaneVertexes(laneIndex);
            idx = obj.lbsd.getVertPositions(ids);
            pos = [idx(1, :) idx(2, :)];
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
        
        function laneTrajectory(obj, lanes, time)
            % laneTrajectory - graphs lane space diagrams
            % Input:
            %    lanes (1 x n array): lane indexes
            %    time (1 x 2): the start and end time, empty for totalTime
            for lane = 1:length(lanes)
                laneIndex = lanes(lane);
                lane_length = obj.lbsd.getLaneLengths(laneIndex);
                lane_flights = obj.lbsd.getLaneReservations(laneIndex);
                UASData = obj.laneData(laneIndex).telemetry; % Grabs all
                if(~isempty(time)) % Wanted to see a specific time range
                    [rows, ~] = find(lane_flights.entry_time_s >= time(1) & ...
                        lane_flights.exit_time_s <= time(2));
                    lane_flights = lane_flights(rows, :);
                    [rows, ~] = find(UASData.time >= time(1) & ...
                        UASData.time <= time(2));
                    UASData = UASData(rows, :);
                end
                if ~isempty(lane_flights)
                    figure;
                    pts = lane_flights{:, 3:4};
                    for p = 1:size(pts, 1)
                        x = [pts(p, 1) pts(p, 2)];
                        y = [0 lane_length];
                        plot(x, y, 'DisplayName', strcat("UAS ID ", lane_flights{p, 1}));
                        hold on;
                    end
                    
                    uniqueID = unique(UASData.ID);
                    for d = 1:size(uniqueID, 1)
                        [rows, ~] = find(UASData.ID == uniqueID(d));
                        pts = UASData{rows, [3,6]};
                        scatter(pts(d, 1), pts(d, 2), 'DisplayName', ...
                            strcat("UAS : ", uniqueID(d)));
                    end
                    
                    xlabel("Time");
                    ylabel("lane Distance");
                    title(strcat("Lane : ", lane_flights{1, 2}));
                    legend('Location', 'westoutside');
                end
            end
        end
        
        function UASInfo = updateTelemetry(obj, UASInfo, src, laneNumber)
            % updateTelemetry - updates the telemetry data for the lane data
            %    structure
            % Input
            %   UASInfo (n x 5 table)
            %       .pos (1 x 6) entry and exit coordinates
            %       .telemetry (n x 6 table)
            %   src (Object) UAS causing reporting their telemetry data.
            
            lanes = UASInfo.pos; % Entry - exit cord
            lane_flights = obj.lbsd.getLaneReservations(laneNumber);
            UASpos = [src.x, src.y, src.z];
            del_speed = calculateSpeedDifference(src, laneNumber);
            del_t = timeAdjustment(obj.time, lane_flights, src.id);
            del_dis = delDistance(UASpos, lanes, del_t);
            % Set up for Projection
            posLanes = [lanes(4) - lanes(1), lanes(5) - lanes(2), ...
                lanes(6) - lanes(3)];
            uUAS = [src.x - lanes(1), src.y - lanes(2),...
                src.z - lanes(3)];
            project = projectUAS(uUAS, posLanes);
            % Update telemetry data
            UASInfo.UAS{end + 1, {'ID', 'pos', 'time',...
                'del_speed', 'del_dis', 'projection'}}...
                = [src.id, UASpos, obj.time, del_speed, del_dis, project];
        end
        
        function time = timeAdjustment(time, lane_flights, id)
        % timeDifference - finds the amount of time spent in a given lane
        % Input
        %   time (float) - current time
        %   lane_flights (n x 6 table) - lane reservations for a particular
        %       lane
        %   id (string) - The UAS ID.
            [rows, ~] = find(lane_flights.ID == id);
            entryTime = lane_flights(rows).entry_time_s;
            time = time - entryTime;
        end
        
        function dis = delDistance(posUAS, posLane, time)
        % delDistance - calculates the deviation in distance from actual
        %   UAS position and planned UAS Position
        % Input - 
        %   posUAS (1 x 3): the pos coordinates of the UAS
        %   posLane (1 x 6): The entry and exit coordinates
        %   time (double): the time difference from expected entry time and
        %      current time.
            dirVector = posLane(4:6) - posLane(1:3);
            ro = posLane(1:3);
            r = ro + time*dirVector;
            dis = norm(r, posUAS);
        end
        
        function speedvsdisGraph(obj, lanes, time)
        % speedvdisGraph - graphs difference in speed and distance from the
        %   actual UAS flight and planned flight.
        % Input:
        %   lanes (1 x n): lane id's
        %   time (1 x 2): start and ending time to look at
            for lane = 1:length(lanes)
                lane_id = lanes(lane); % Grab the ID
                UASInfo = obj.laneData(lane_id).telemetry; % Grab the telemetry data
                [rows, ~] = find(UASInfo.time >= time(1) ...
                    & UASInfo.time <= time(2)); % Grab the specific time
                UASInfo = UASInfo(rows, :);
                times = UASInfo.time;
                times = unique(times); % Grab the unique times for graghing
                for t = 1:length(times)
                    index = mod(time, 3);
                    if (index == 0) % Create new figure
                        figure;
                        subplot(1, 3);
                    end
                    % Grab points
                    [rows, ~] = find(UASInfo.time == times(t));
                    tnew = UASInfo(rows, :);
                    subplot(1, 3, index + 1);
                    hold on;
                    for uas = 1:size(tnew, 1)
                        plot(tnew(uas).del_speed, tnew(uas).del_dis, ...
                            'DisplayName', tnew(uas).ID);
                    end
                    title(strcat("Lane ", lanes(lane), " Time ",...
                        num2str(times(t))));
                    xlabel('Speed Deviation');
                    ylabel('Distance Deviation');
                    legend('Location', 'westoutside');
                end
            end
        end
    end
end

