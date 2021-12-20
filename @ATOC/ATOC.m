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
            end
        end
    end
    
    %% Data Structure Creation and Updates
    % This section deals with the creation of the ATOC data structures and  
    %  the maintainance of each of these data structures. 
    %
    % The main data structures
    %   lane data: stores information regaurding the individual lanes
    %   telemetry data: stores the informaiton that is gathered from the
    %       UAS informaiton
    %   Sensory Data: stores the information that is gathered from sensors
    %      within the simulation
    methods (Access = private)
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
            UASpos = [UASgps.lon, UASgps.lat, UASgps.alt];
            del_speed = obj.calculateSpeedDifference(src,UASInfo,...
                lane_flights, lanes);
            del_t = obj.timeAdjustment(lane_flights, src.id);
            del_dis = obj.delDistance(UASpos, lanes, del_t);
            % Set up for Projection
            posLanes = lanes(4:6) - lanes(1:3);
            ri = [0,0,0] + del_t*posLanes;
            uUAS = UASpos - lanes(1, 1:3);
            if(sum(abs(ri)) == 0)
                project = norm(uUAS);
            else
                project = projectUAS(obj, uUAS, ri);
            end
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
                = [src.id, [src.gps.lon, src.gps.lat, src.gps.alt], ...
                src.nominal_speed, obj.time];
        end
        function UpdateLaneInformation(obj, lane_id, radarPts)
            % UpdateLaneInformation - This method is used to update the lane
            % data structure in the atoc system.
            % Input
            %   obj (atoc handle): the atoc object
            %   lane_id (string): The lane id that needs to be updated.
            %   radarPts (nx4 table): Contains the radar points
            %       .id (string): Radar ID
            %       .pos (nx3 array): Target Positions
            %       .time (float): Time of detection
            % Output:
            % Call:
            %   obj.UpdateLaneInformation("1", radar);
            %
            % Updating Density Information
            density = obj.laneData(lane_id).density;
            UASInfo = obj.laneData(lane_id);
            if(obj.laneData(lane_id).density.time(end) < ...
                    obj.time + 100*eps & ...
                    obj.laneData(lane_id).density.time(end) > ...
                    obj.time - 100*eps)
                density{end, {'number'}} = density.number(end) + 1;
            else % Add a new row in Data Table
                density{end+1, {'number', 'time'}} = [1, obj.time];
            end
            
            UASInfo.density = density;
            UASInfo.sensory = [UASInfo.sensory; radarPts];
            obj.laneData(lane_id) = UASInfo;
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
                tnew3 = table();
                tnew3.ID = "";
                tnew3.pos = zeros(1,3);
                tnew3.speed = 0;
                tnew3.time = 0;
                info.sensory = tnew3;
                obj.laneData(lanes(l)) = info;
            end
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
    %% Calculation Helper Methods
    % This section deals with analyzing the general behavior of the objects
    % that are within the simulation. 
    % 
    % Behaviors being analyzed
    %   1. Difference between planned and actual- dis, speed
    %   2. Projection to the lane
    %   3. Analomy Behaviors - Handled in different sections
    methods (Access = private)
        function laneNumber = findLaneId(obj, src)
            res = obj.lbsd.getReservations();
            rows = find(res.entry_time_s <= obj.time & res.exit_time_s >= ...
                obj.time & res.uas_id == src.id);
            res = res(rows, :);
            laneNumber = res.lane_id;
        end
        
        function [tel_rows, sen_rows] = findRows(obj)
        % findRows - this is a private helper method that is used to
        % finding comparison between double times when updating the lane
        % data information.
        % Input:
        %   obj (atoc handle)
        % Output
        %   tel_rows (nx1): The rows in the telemetry data set that
        %       corrspond with the current time stamp
        %   sen_rows (nx1): The rows in the sensory data set that corrspond
        %      with the current time stamp
        %
        
            % Atoc Clock
            cur_time = round(obj.time*(10^15))./(10^15);
            
            % Data sturcture clock
            telemetry_times = round(obj.telemetry.time*(10^15))./(10^15);
            sensory_times = round(obj.radars.time*(10^15))./(10^15);
            
            % Find Differences
            dif = round(abs(telemetry_times - cur_time), 2, 'decimals');
            dif2 = round(abs(sensory_times - cur_time), 2, 'decimals');
            
            % Select the Current Data Rows
            [tel_rows, ~] = find(dif == 0 & obj.telemetry.ID ~= "");
            [sen_rows, ~] = find(dif2 == 0 & obj.radars.ID ~= "");
        end
        
        function findClusters(obj)
            % findClusters - clusters the telemetry data and the sensory data
            %   to find the number of UAS flying in the Simulation at a
            %   given time
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
            
            % Pull the nesscary information from the telemetry and radar
            % lists
            
            [tel_rows, sen_rows] = obj.findRows();
            
            UASInfo = obj.telemetry(tel_rows, :);
            RadarInfo = obj.radars(sen_rows, :);
            
            % Ensuring that either group is filled with information
            if (~isempty(UASInfo) || ~isempty(RadarInfo))
                % Cluster the data points
                datapts = [UASInfo.pos; RadarInfo.pos];
                % Find Reservations that Corrspond to this time
                [rows, ~] = find(obj.lbsd.getReservations.entry_time_s ...
                    <= obj.time + 100*eps & ...
                    obj.lbsd.getReservations.exit_time_s >= obj.time - ...
                    100*eps);
                res = obj.lbsd.getReservations();
                res = res(rows, :);
                [idx, corepts] = dbscan(datapts, 2, 1);
                
                % Find the corresponding lane and update charts
                obj.ProjectToLanes(UASInfo, RadarInfo, datapts, idx,...
                    corepts, res);
                
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
        function ProjectToLanes(obj, UASInfo, RadarInfo, datapts,...
                idx, corepts, res)
            % ProjectToLanes - This is a helper method that will find the
            % associated lanes with each data groups and then update the lane
            % data stuctures.
            % Input:
            %   obj (atoc handle): This atoc object
            %   UASInfo (table): The telemetry data for this simulation step
            %   RadarInfo (table): The sensory data for this simulation step
            %   idx (nx1): The cluster indices for all the data points
            %   corepts (nx1): Indicates if the idx array points are the
            %       corepts in the clusters
            % Ouput:
            % Call:
            %   obj.ProjectToLanes(UASInfo, RadarInfo, idx, corepts);
            %
            
            % Grab all the unique cluster indices
            uniIdx = unique(idx);
            
            % NoLaneYet - bool
            NoLane = false;
            
            % Lane_id saving
            lane_id = "";
            
            % Loop through all the unique Indices
            for group = 1:length(uniIdx)
                % Grab the data group
                [rows, ~] = find(idx == uniIdx(group));
                cluster = datapts(rows, :);
                
                % Check which UAS belongs to this group
                [rows, ~] = find(ismember(cluster, UASInfo.pos,...
                    'rows') == 1);
                % If uas found - grab res data
                if(~isempty(rows))
                    uasID = UASInfo.ID(rows(1));
                    [row, ~] = find(res.uas_id == uasID & ...
                        res.entry_time_s <= obj.time & ...
                        res.exit_time_s >= obj.time);
                    % Reservation Found
                    if(~isempty(row))
                        % Update Lane Structure
                        lane_id = res.lane_id(row);
                    else
                        % If no res data found - Add to Anamoly List(update
                        % existing one if in list) - NoLaneYet = false
                        NoLane = true;
                    end
                else
                    % If no uas found - Anamoly List - NoLaneYet = True
                    NoLane = true;
                end
                
                % No Reserveration/Not Transmitting Information
                if(NoLane)
                    % Grab the core point location
                    [row, ~] = find(corepts(rows, :) == 1);
                    uasPoint = cluster(row, :);
                    
                    % Grab all the lane information
                    lane_ids = obj.lbsd.getLaneIds();
                    minDis = Inf;
                    % Loop through all the lanes to find the associated
                    % lane
                    for index = 1:length(lane_ids)
                        ids = obj.lbsd.getLaneVertexes(lane_ids(index));
                        pos = obj.lbsd.getVertPositions(ids);
                        mid = (pos(2,:) - pos(1, :))/2;
                        dis = norm(uasPoint - mid);
                        if(dis < minDis)
                            minDis = dis;
                            lane_id = lane_ids(index);
                        end
                    end
                end
                
                % Remove UAS Information
                cluster(rows, :) = [];
                idx = find(ismember(cluster, RadarInfo.pos, 'rows') == 1);
                obj.UpdateLaneInformation(lane_id, RadarInfo(idx, :));
            end
        end
        function pos = getPosition(obj, laneIndex)
            % getPosition - Obtains the starting and ending vertexes
            %   positions
            % Input
            %   laneIndex (string): The lane index
            ids = obj.lbsd.getLaneVertexes(laneIndex);
            idx = obj.lbsd.getVertPositions(ids(1, :));
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
            dis = (dotProd/(normLane));
        end
        function del_speed = calculateSpeedDifference(obj, src, ...
                UASInfo, lane_flights, lanePos)
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
            
            if (~isempty(tel_info)) % Has seen this drone before
                prev_pos = tel_info.pos(end, :);
                prev_time = tel_info.time(end);
                rows = lane_flights.uas_id == src.id & ...
                    lane_flights.entry_time_s <= obj.time ...
                    & lane_flights.exit_time_s >= obj.time;
                prev_info = lane_flights(rows, :);
                entry_time = prev_info.entry_time_s;
                exit_time = prev_info.exit_time_s;
                if(~isempty(prev_info)) % Has > 1 information on the drone
                    % Planned Speed
                    del_time = (obj.time - prev_time);
                    prev_time = (prev_time - entry_time);
                    prevPlan = lanePos(1:3) + ...
                        (prev_time)*(lanePos(4:6) - lanePos(1:3));
                    curPlan = lanePos(1:3) + ...
                        (del_time)*(lanePos(4:6) - lanePos(1:3));
                    
                    scheduled_speed = norm(curPlan - prevPlan)/del_time;
                    
                    % Drone Information
                    current_pos = [src.gps.lon, src.gps.lat, src.gps.alt];
                    del_dis = norm((prev_pos - current_pos));
                    speedUAS = del_dis/(del_time);
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
            exitTime = lane_flights(rows, :).exit_time_s;
            time = (obj.time - entryTime)/(exitTime - entryTime);
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
    end
    %% Rogue Detection
    % This section deals with identify any Rogue Behaviors of UAS' during
    % the life of the simulation, and then notify when certain behaviors
    % are observed
    %
    methods
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
        function laneDensity(obj, lane_ids, timeInterval)
        % laneDensity - Displays the given lane density for a specific lane
        % over a specific time interval. If no timeInterval is specified
        % than it will look at the whole simulation
        % Input:
        %   lane_ids (1xn stirng array) = Lane ids
        %   timeInterval (1x2 float array): specific time to look at
        %       column 1: The starting time
        %       column 2: The ending time
        % Output:
        %   density vs time graph
        % Call:
        %   atoc.laneDensity(["1", "2"], []) % Over the entire simulation
        %   atoc.laneDensity(["1", "2"], [0, 10]) % density between 0-10
        %       minutes
        
            % Loop through Lane_ids
                % Create a new graph
                % Pull out the desirable times
                % Plot This graph
        end
        function OverallLaneComparison(obj, lane_ids, timeInterval)
        % OverallLaneCOmparison - Displays the average density comparison
        % between the specified lanes, if lane_ids is empty does the
        % comparison between all of the lanes in the airway system.
        % Input:
        %   obj (atoc handle object)
        %   lane_ids (1xn string array): Lane ids for comparison
        %   timeInterval (1x2 float array): specific time to look at
        %       column 1: The starting time
        %       column 2: The ending time
        % Output:
        %   Pie graph comparison between average density over the
        %   simulation
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
                    pts = lane_flights{:, {'entry_time_s', 'exit_time_s'}};
                    for p = 1:size(pts, 1)
                        x = [pts(p, 1) pts(p, 2)];
                        y = [0 lane_length];
                        plot(x, y, 'DisplayName', ...
                            strcat("Planned UAS ID ", lane_flights{p, {'uas_id'}}));
                        hold on;
                    end
                    
                    uniqueID = unique(UASData.ID);
                    for id = 1:length(uniqueID)
                        if(~(uniqueID(id) == ""))
                            [rows, ~] = find(UASData.ID == uniqueID(id));
                            pts = UASData{rows, {'time', 'projection'}};
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
                        xmin = 0 - (max(abs(tnew(:, :).del_speed)));
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
end