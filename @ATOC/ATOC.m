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
        overallDensity % Keeps track of overall Density
    end
    
    %% General Functions
    % This section deals with the creation of the ATOC instance object as
    % well as maintain all of the data structures and any particular
    % calculations that are used to assist with informaiton added into the
    % data structures.
    %
    % The main data structures
    %   lane data: stores information regaurding the individual lanes
    %   telemetry data: stores the informaiton that is gathered from the
    %       UAS informaiton
    %   Sensory Data: stores the information that is gathered from sensors
    %      within the simulation
    methods
        function obj = ATOC(lbsd)
            % ATOC - Constructor to Monitor UAS activity
            % Input:
            %   lbsd (LBSD Handle): LBSD handle from simulation
            % Output:
            %   obj (atoc handle): ATOC instance object
            % Call:
            %   atoc = ATOC(lbsd);
            obj.lbsd = lbsd;
            obj.createLaneData();
            obj.createRadarTelemetryData();
            obj.overallDensity = struct('data', [0,0], ...
                'fHandle', figure('Visible', 'off'), ...
                'pHandle', plot(0, 0));
            obj.time = 0;
            linkdata(obj.overallDensity.fHandle)
        end
        
        function handle_events(obj, src, event)
            % handle_events - handles any listening event during simulation
            % Input:
            %   obj (ATOC handle): atoc instance object
            %   src : an instance object that created the event
            %   event (event handle) - The action that triggered an action
            if event.EventName == "Tick"
                findClusters(obj);
                obj.time = obj.time + src.tick_del_t;
            end
            if event.EventName == "NewReservation"
                obj.lbsd = src;
            end
            if event.EventName == "telemetry"
                laneNum = obj.findLaneId(src);
                obj.updateLaneData(src, laneNum);
                obj.updateTelemetry(src);
            end
            
            if event.EventName == "Detection"
                for item = 1:size(src.targets)
                    obj.radars{end + 1, {'ID', 'pos', 'speed', 'time'}}...
                        = [src.ID, [src.targets(item).x, ...
                        src.targets(item).y, src.targets(item).z],...
                        src.targets(item).s, obj.time];
                end
                % Grab Radar informaiton and analyze the data
            end
        end
    end
    
    % Helper Private Functions
    methods (Access = private)
        
        function laneNumber = findLaneId(obj, src)
            res = obj.lbsd.getReservations();
            rows = zeros(length(src.res_ids), 1);
            for id = 1:length(src.res_ids)
                rows(id) = find(res.id == num2str(src.res_ids(id)), 1);
            end
            res = res(rows, :).lane_id;
            pos = [src.gps.lat, src.gps.lon, src.gps.alt];
            laneMin = norm(obj.laneData(res(1)).pos(4:6) - pos);
            lane_id = res(1);
            for lane = 2:length(res)
                dis = norm(obj.laneData(res(lane)).pos(4:6) - pos);
                if( dis < laneMin)
                    lane_id = res(id);
                    laneMin = dis;
                end
            end
            laneNumber = lane_id;
        end
        
        function updateLaneData(obj, src, laneNumber)
            % updateLaneData - updates the Lane Information Given the
            % specific Sensory and Telemetry Data
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
            % updateTelemetry - Updates the telemetry data database
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
            %   src (UAS Handle) - UAS instance object
            obj.telemetry{end + 1, {'ID', 'pos', 'speed', 'time'}} ...
                = [src.id, [src.gps.lat, src.gps.lon, src.gps.alt], ...
                src.nominal_speed, obj.time];
        end
        
        function findClusters(obj)
            % findClusters - clusters the telemetry data and the sensory data
            %   to find the number of UAS flying in the Simulation at a
            %   given time
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
            [rows, ~] = find(obj.telemetry.time == obj.time & ...
                obj.telemetry.ID ~= "");
            UASInfo = obj.telemetry(rows, :);
            [rows, ~] = find(obj.radars.time == obj.time& ...
                obj.radars.ID ~= "");
            RadarInfo = obj.radars(rows, :);
            if (~isempty(UASInfo) && ~isempty(RadarInfo))
                datapts = [UASInfo.pos; RadarInfo.pos];
                [rows, ~] = find(obj.lbsd.getReservations.entry_time_s <= obj.time & ...
                    obj.lbsd.getReservations.exit_time_s >= obj.time);
                res = obj.lbsd.getReservations();
                res = res(rows, :);
                idx = dbscan(datapts, 2, 1);
                if (size(unique(idx), 1) ~= size(res, 1)) %Rogue Detection
                end
                obj.overallDensity.data = [obj.overallDensity.data; ...
                    obj.time, size(unique(idx), 1)];
            else
                obj.overallDensity.data = [obj.overallDensity.data; ...
                    obj.time, 0];
            end
            obj.updatePlot();
        end
        
        function createRadarTelemetryData(obj)
            % createRadarTelemetryData - Creates the data structure for the
            %   sensor and telemetry data produced from the simulation
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
            tnew = table();
            tnew.ID = "";
            tnew.pos = zeros(1, 3);
            tnew.speed = 0;
            tnew.time = 0;
            obj.radars = tnew;
            obj.telemetry = tnew;
        end
        
        function createLaneData(obj)
            % createLaneData - creates a lane data structure to store all the
            %   telemetry and sensory information
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
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
            % projectUAS - locates the UAS distance along the lane using
            %   projection
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
            % calculateSpeedDifference - Calculates the deivation from the
            %   planned speed and the actual speed of an UAS
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
            %   src (UAS Handle) - UAS instance object
            %   UASInfo (nx6 table) - Previous Telemeletry Data
            %   lane_flights (n x 6 table) - Reservation data for the specific lane
            % Output:
            %   The difference between the planned speed versus the actual
            %   speed
            rows = UASInfo.telemetry.ID == src.id;
            tel_info = UASInfo.telemetry(rows, :);
            speedUAS = 0;
            scheduled_speed = 0;
            if (~isempty(tel_info))
                prev_pos = tel_info.pos(end, :);
                prev_time = tel_info.time(end);
                rows = lane_flights.id == num2str(src.res_ids(end));
                prev_info = lane_flights(rows, :);
                if(~isempty(prev_info))
                    scheduled_speed = prev_info(end, :).speed;
                    current_pos = [src.gps.lat, ...
                        src.gps.lon, src.gps.alt];
                    del_time = obj.time - prev_time;
                    del_dis = norm((prev_pos - current_pos));
                    speedUAS = del_dis/del_time;
                else
                    %warning("Not Scheduled Flight");
                end
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
            % Output:
            %   dis (float) - the deviation in distance
            dirVector = posLane(4:6) - posLane(1:3);
            ro = posLane(1:3);
            r = ro + time*dirVector;
            dis = norm(r - posUAS);
        end
        
        function updatePlot(obj)
            % updatePlot - updates the density plot throughout the simulation,
            %   this is what makes the autonomatically updating plot.
            % Input:
            %   obj. (ATOC Handle) - ATOC instance object
            if(isvalid(obj.overallDensity.pHandle))
                obj.overallDensity.pHandle.XData = obj.overallDensity.data(:,1);
                obj.overallDensity.pHandle.YData = obj.overallDensity.data(:,2);
            end
        end
    end
    %% Density Graphs
    % This section deals with the graphs that show the density of UAS
    % during the simulation.
    %
    % The main two functions are
    %   1. Show the total density in the lane   system through the simulation
    %   2. Show the density in each given lane at a specific interval
    methods
        function showDensity(obj)
            % showDensity - shows the density in all lanes throughout time
            % Input:
            %   obj (atoc object) - the atoc instance
            % Output:
            %   shows the density of UAS throughout the time of the simulation
            % Call:
            %   atoc.showDensity()
            if(isvalid(obj.overallDensity.fHandle)) % The figure wasn't closed
                set(obj.overallDensity.fHandle, 'visible', 'on');
                refreshdata();
            else % Regenerate the figure
                obj.overallDensity.fHandle = figure();
                obj.overallDensity.pHandle = plot(obj.allDen(:,1), obj.allDen(:,2));
                obj.overallDensity.pHandle.XData = obj.overallDensity.data(:,1);
                obj.overallDensity.pHandle.YData = obj.overallDensity.data(:,2);
                linkdata(obj.overallDensity.fHandle)
            end
            xlabel("time");
            ylabel("Density");
            title("Density Time Graph");
        end
    end
    %% Lane Graphs
    % This section deals with the graphs that show trajectory, sensory, and
    % telemetry data with reguards to lane system.
    %
    % The main functions are
    %   1. Show the space lane diagrams of actual vs planned flights
    %   2. Show the deviation between speed and distance between actual vs
    %   planned flights
    %   3. Show flight trajectory for a given UAS of actual vs planned
    %   flights
    methods
        function laneGraphs(obj, lanes, time)
            % laneGraphs - displays lane space diagrams and change in speed
            %   and distance graphs for specific lanes and time interval
            % Input:
            %   obj (atoc handle) - the atoc object
            %   lanes (1 x n array): lane indexes
            %   time (1 x 2 array): the start and end time,
            %       empty for totalTime
            % Output:
            %   shows two graphs - lane space diagrams and speed vs
            %   distance graphs
            % Call:
            %   atoc.laneGraphs(["1", "2"], [0, 23])
            laneTrajectory(obj, lanes, time);
            speedvsdisGraph(obj, lanes, time);
        end
        
        function laneTrajectory(obj, lanes, time)
            % laneTrajectory - displays the lane space diagrams
            % Input:
            %   lanes (1 x n array): lane indexes
            %   time (1 x 2): the start and end time, empty for totalTime
            % Output:
            %   Displays the planned flights versus the UAS telemetry data
            % Call:
            %   atoc.laneTrajector(["1", "2"], [0, 2]);
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
            % Output:
            %   Displays the deviation of speed and distance from the
            %   planned flights and the actual UAS telemetry data at
            %   specific lane and time interval
            % Call:
            %   atoc.speedvsdisGraph(["1"], [3, 9]);
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
                        if (mod(t, 6) == 0)
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
    end
    
    %% Rogue Detection
    % This section deals with identify any Rogue Behaviors of UAS' during
    % the life of the simulation, and then notify when certain behaviors
    % are observed
    %
    methods
    end
end