classdef ATOC < handle
    % ATOC - Monitors UAS Activity During Simulation
    %   Monitors: Lane Density, Lane Deviations, Rogue Flights
    %   Tracks: UAS Telemetry Data, Radar Sensors
    
    properties
        lbsd  % Reseveration Data (planned flight data)
        laneData % lane Informaiton
        radars % Store Radar Sensory Data
        telemetry % Store UAS Telemetry Data
        time % Keep track of time
        allDen % plot handle for overall density in a graph
        denfig % Density Figure
    end
    
    methods (Static)
        radar = LEM_radars_placement_coverage(res,range, noise, angle)
    end
    
    % User Accessible Functions
    methods
        
        function obj = ATOC(lbsd) %range, noise, angle)
            % ATOC - Constructor to Monitor UAS activity
            % Input:
            %   lbsd (LBSD Handle): LBSD handle from simulation
            %   range (float): radars sensor range
            %   noise (3x3 matrix): noise in the radar sensors
            %   angle (float): The radar's beamwidth
            obj.lbsd = lbsd;
            % obj.radars = ATOC.LEM_radars_placement_coverage(lbsd, range, ...
            % noise, angle); % Change radars to a class.
            obj.createLaneData();
            obj.createRadarTelemetryData();
            obj.denfig = figure('Visible', 'off');
            obj.allDen = plot(0,0,'.');
        end
        
        function showDensity(obj)
            if(isvalid(obj.denfig))
                obj.denfig.Visible = 'on';
            end
        end
        
        function dontShowDensity(obj)
            if(isvalid(obj.denfig))
                obj.denfig.Visible = 'off';
            end
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
                obj.findClusters();
                obj.time = src.tick_del_t;
            end
            if event.EventName == "NewReservation"
                obj.lbsd = src;
            end
            if event.EventName == "telemetry"
                laneNum = obj.lbsd.getLaneIdFromResId(src.res_ids(end));
                obj.updateLaneData(src, laneNum);
                obj.updateTelemetry(src);
            end
            
            if event.EventName == "Detection"
                obj.radars{end + 1, {'ID', 'pos', 'speed', 'time'}}...
                = [src.ID, [src.targets.x, src.targets.y, src.targets.z],...
                src.targets.s, obj.time];
                % Grab Radar informaiton and analyze the data
            end
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
                        plot(x, y, 'DisplayName', ...
                            strcat("Planned UAS ID ", lane_flights{p, 1}));
                        hold on;
                    end
                    
                    uniqueID = unique(UASData.ID);
                    for id = 1:length(uniqueID)
                        if(~(uniqueID(id) == ""))
                            [rows, ~] = find(UASData.ID == uniqueID(id));
                            pts = UASData{rows, [3,6]};
                            scatter(pts(:, 1), pts(:, 2), 'DisplayName', ...
                                strcat("Actual UAS : ", uniqueID(id)));
                        end
                    end
                    xlabel("Time");
                    ylabel("lane Distance");
                    ylim([0, lane_length]);
                    title(strcat("Lane : ", lane_flights{1, 2}));
                    legend('Location', 'eastoutside');
                end
            end
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
                if ~(size(UASInfo, 1) == 1 && strcmp(UASInfo.ID,'empty'))
                    times = UASInfo.time;
                    times = unique(times); % Grab the unique times for graghing
                    figure;
                    tiledlayout("flow");
                    for t = 1:length(times)
                        if (mod(t, 13) == 0)
                            figure;
                        end
                        nexttile;
                        [rows, ~] = find(UASInfo.time == times(t));
                        tnew = UASInfo(rows, :);
                        hold on;
                        for uas = 1:height(tnew)
                            scatter(tnew(uas, :).del_speed, tnew(uas, :).del_dis, ...
                                'DisplayName', strcat("UAS ID:", tnew(uas, :).ID));
                        end
                        title(strcat("Lane ", lanes(lane), " Time ",...
                            num2str(times(t))));
                        ylim([0, (max(tnew(:,:).del_dis) + 1)]);
                        xmin = 0 - (max(tnew(:, :).del_speed));
                        xlim([xmin - 1, max(tnew(:,:).del_speed) + 1]);
                        xlabel('Speed Deviation');
                        ylabel('Distance Deviation');
                        legend('Location', 'westoutside');
                        hold off;
                    end
                end
            end
            
        end
        
        function updateLaneData(obj, src, laneNumber)
            % updateLaneData - updates the telemetry data for the lane data
            %    structure
            % Input
            %   UASInfo (n x 5 table)
            %       .pos (1 x 6) entry and exit coordinates
            %       .telemetry (n x 6 table)
            %   src (Object) UAS causing reporting their telemetry data.
            UASInfo = obj.laneData(laneNumber);
            lanes = UASInfo.pos; % Entry - exit cord
            lane_flights = obj.lbsd.getLaneReservations(laneNumber);
            UASgps = src.gps;
            UASpos = [UASgps.lat, UASgps.lon, UASgps.alt];
            del_speed = obj.calculateSpeedDifference(src,UASInfo,lane_flights);
            del_t = obj.timeAdjustment(lane_flights, src.id);
            del_dis = obj.delDistance(UASpos, lanes, del_t);
            % Set up for Projection
            posLanes = [lanes(4) - lanes(1), lanes(5) - lanes(2), ...
                lanes(6) - lanes(3)];
            uUAS = UASpos - lanes(1:3);
            project = projectUAS(obj, uUAS, posLanes);
            % Update telemetry data
            UASInfo.telemetry{end + 1, {'ID', 'pos', 'time',...
                'del_speed', 'del_dis', 'projection'}}...
                = [src.id, UASpos, obj.time, del_speed, del_dis, project];
            obj.laneData(laneNumber) = UASInfo;
        end
        
        function updateTelemetry(obj, src)
            obj.telemetry{end + 1, {'ID', 'pos', 'speed', 'time'}} ...
                    = [src.id, [src.gps.lat, src.gps.lon, src.gps.alt], ...
                    src.nominal_speed, obj.time];
        end
    end
    
    % Helper Private Functions
    methods (Access = private)
        
        function findClusters(obj) 
        % findClusters - clusters the telemetry data and the sensory data
        [rows, ~] = find(obj.telemetry.time == obj.time);
        UASInfo = obj.telemetry(rows, :);
        [rows, ~] = find(obj.radars.time == obj.time);
        RadarInfo = obj.radars(rows, :);
        datapts = [UASInfo.pos; RadarInfo.pos];
        [rows, ~] = find(obj.lbsd.getReservations.entry_time_s <= obj.time & ...
            obj.lbsd.getReservations.exit_time_s >= obj.time);
        res = obj.lbsd.getReservations(rows, :);
        idx = dbscan(datapts, 3, 1);
        if (size(unique(idx), 1) ~= size(res, 1)) %Rogue Detection
        end
        obj.allDen.XData = [obj.allDen.XData, obj.time];
        obj.allDen.YData = [obj.allDen.YData, size(unique(idx), 1)];
        end
        
        function createRadarTelemetryData(obj)
            tnew = table();
            tnew.ID = "";
            tnew.pos = zeros(1, 3);
            tnew.speed = 0;
            tnew.time = 0;
            obj.radars = tnew;
            obj.telemetry = tnew;
        end
        
        function createLaneData(obj)
            % createLaneData - creates a lane data structure
            lanes = obj.lbsd.getLaneIds();
            obj.laneData = containers.Map('KeyType', 'char', ...
                'ValueType', 'any'); % Initinializing/Declaring LaneData
            for l = 1:size(lanes, 1)
                info = struct();
                info.pos = obj.getPosition(lanes(l));
                tnew = table();
                tnew.number = 0;
                tnew.time = 0;
                info.density = tnew;
                tnew2 = table();
                tnew2.ID = "";
                tnew2.pos = zeros(1, 3);
                tnew2.time = 0;
                tnew2.del_speed = 0;
                tnew2.del_dis = 0;
                tnew2.projection = 0;
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
        
        function dis = projectUAS(obj, posUAS, posLane)
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
        
        function del_speed = calculateSpeedDifference(obj, src, UASInfo, lane_flights)
            % Grab previous position
            rows = UASInfo.telemetry.ID == src.id;
            tel_info = UASInfo.telemetry(rows, :);
            speedUAS = 0;
            scheduled_speed = 0;
            if (~isempty(tel_info))
                prev_pos = tel_info.pos(end, :);
                prev_time = tel_info.time(end);
                rows = lane_flights.id == src.id;
                prev_info = lane_flights(rows, :);
                scheduled_speed = prev_info(end, :).speed;
                current_pos = [src.gps.lat, ...
                    src.gps.lon, src.gps.alt];
                del_time = obj.time - prev_time;
                del_dis = norm((prev_pos - current_pos));
                speedUAS = del_dis/del_time;
            end
            del_speed = speedUAS - scheduled_speed;
        end
        
        function time = timeAdjustment(obj, lane_flights, id)
            % timeDifference - finds the amount of time spent in a given lane
            % Input
            %   time (float) - current time
            %   lane_flights (n x 6 table) - lane reservations for a particular
            %       lane
            %   id (string) - The UAS ID.
            [rows, ~] = find(lane_flights.id == id);
            entryTime = lane_flights(rows, :).entry_time_s;
            time = obj.time - entryTime;
        end
        
        function dis = delDistance(obj, posUAS, posLane, time)
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
            dis = norm(r - posUAS);
        end
    end
end