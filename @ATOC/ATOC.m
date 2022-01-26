classdef ATOC < handle
    % ATOC - Monitors UAS Activity During Simulation
    %   Monitors: Lane Density, Lane Deviations, Rogue Flights
    %   Tracks: UAS Telemetry Data, Radar Sensors

    properties
        lbsd  % Reseveration Data (planned flight data)
        masterList % All Simulation Information
        radars % Store Radar Sensory Data
        telemetry % Store UAS Telemetry Data
        time % Keep track of time
        indexer % Keep track of the row entry in masterlist
        rogue_uas % A list to keep track of the rogue uas behaviors
    end

    %% Constructor and Event Handling
    methods
        % Constructor
        function obj = ATOC(lbsd)
            % ATOC - Constructor to Monitor UAS activity
            % Input:
            %   lbsd (LBSD Handle): LBSD handle from simulation
            % Output:
            %   obj (atoc handle): ATOC instance object
            % Call:
            %   atoc = ATOC(lbsd);

            % Store LBSD Handle
            obj.lbsd = lbsd;

            % Master Information about the flights
            masterList(100) = struct('time', [], 'lane_id', [], ...
                'uas_id', [], 'res_id', [], 'telemetry', [], 'sensory', [], ...
                'del_dis', [], 'del_speed', [], 'proj', [], 'Rogue', []);
            obj.masterList = masterList;
            obj.rogue_uas = [];

            % Create tempory telemetry & radar data structures
            obj.createRadarTelemetryData();
            % Set the atoc time
            obj.time = 0;
            % Set up the indexer
            obj.indexer = 1;
        end
        
        % Event Handler
        function handle_events(obj, src, event)
            % handle_events - handles any listening event during simulation
            % Input:
            %   obj (ATOC handle): atoc instance object
            %   src : an instance object that created the event
            %   event (event handle) - The action that triggered an action
            % Output:
            %

            if event.EventName == "Tick"
                % This event is to handle the analysis that happens with each
                % simulation step.

                % Update the master list
                UpdateMasterList(obj);
                % Update atoc time
                obj.time = obj.time + src.tick_del_t;
            end
            if event.EventName == "NewReservation"
                % This event is used to handle any new reservations that take
                % place during a simulation.
                obj.lbsd = src;
            end
            if event.EventName == "telemetry"
                % This event is used to store all of the telemetry data during
                % a simulation step
                obj.telemetry(end+1) = struct('ID', src.id,...
                    'pos', [src.gps.lon, src.gps.lat, src.gps.alt], ...
                    'speed', src.nominal_speed, 'time', obj.time);

            end
            if event.EventName == "Detection"
                % This event is used to store all of the sensory information
                % during a simulation step

                for item = 1:size(src.targets, 2)
                    obj.radars(end+1) = struct('ID', src.ID, ...
                        'pos', [src.targets(item).x, ...
                        src.targets(item).y, src.targets(item).z], ...
                        'speed', src.targets(item).s, 'time', obj.time);
                end
            end
        end
        
        % Initalizes the Radar/Telemetry Temporary List
        function createRadarTelemetryData(obj)
            % createRadarTelemetryData - Creates the data structure for the
            %   sensor and telemetry data produced from the simulation
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
            % Output:
            % Call:
            %   atoc.CreateRadarTelemetryData();

            obj.radars = struct('ID', [], 'pos', [], 'speed', [], ...
                'time', []);
            obj.telemetry = struct('ID', [], 'pos', [], 'speed', [],...
                'time', []);
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

    % Update Data Structures
    methods (Access = private)
        % Driver Method to Update the MasterList and Analyze Flights
        function UpdateMasterList(obj)
            % UpdateMasterList - the driver method to perform all of the
            % analysis for each flight behavior within a given simulation
            % step.
            % Input:
            %   obj (atoc handle)
            % Output:
            %

            % Check to see if there is any flights happening
            if(~isempty(obj.telemetry.ID) || ~isempty(obj.radars.ID))
                res = obj.lbsd.getReservations();

                % Grab the UAS from the group.
                datapts = [obj.telemetry.pos; obj.radars.pos];
                [idx, corepts] = dbscan(datapts, 2, 1);

                % find the dist_matrix
                [rows, ~] = find(corepts == 1);
                points = datapts(rows, :);
                D = pdist(points);
                dist = squareform(D);
                uni_idx = unique(idx);

                % Add Unigroups to MasterList
                for index = 1:length(uni_idx)
                    obj.AddEntry(obj.AnalyzeFlight(res, dist, uni_idx(index), ...
                        datapts));
                end
            end

            % Clear previous information
            obj.createRadarTelemetryData();
        end
        
        % Helper Method that Adds Entry To MasterList
        function AddEntry(obj, lane_id, uas_id, res_id, telemetryInfo,...
                sensory, del_dis, del_speed, projection, rogue)
            % AddEntry - This is a helper function that adds an entry into the
            % Master List
            % input:
            %   lane_id (string): the lane number the uas is in
            %   uas_id (string): the id of the uas
            %   res_id (string): the reservation id for the uas
            %   telemetryInfo (array): The reported Telemetry of the particular
            %       uas
            %   sensory (array): The reported sensory informaiton from the
            %       radars
            %   del_dis (float): The distance away from the reservation lane
            %   del_speed (float): The difference in speed from the reservation
            %   projeciton (float): The distance along the given lane
            %   rogue (boolean): Indicates if any rogue behavior is detected.
            % Output:
            %

            % Update the master list
            obj.masterList(obj.indexer, :) = struct('time', obj.time, ...
                'lane_id', lane_id, 'uas_id', uas_id,...
                'res_id', res_id, 'telemetry', telemetryInfo, ...
                'sensory', sensory, 'del_dis', del_dis,...
                'del_speed', del_speed, 'proj', projection,...
                'Rogue', rogue);
            obj.indexer = obj.indexer + 1;
        end
    end
    
    % Helper Calculation functions
    methods(Access = private)
        % Main Method to Analyze Flights
        function [lane_id, uas_id, res_id, telemetryInfo, sensory, ...
                del_dis, del_speed, projection, rogue] = ...
                obj.AnalyzeFlight(obj, res, dist, uniIdx, datapts)
            % AnalyzeFlight - This function is a helper function that performs
            % the neccesary operations needed to update the masterlist in the
            % ATOC class.
            % Input:
            % Output:
            %   lane_id (string): the lane number the uas is in
            %   uas_id (string): the id of the uas
            %   res_id (string): the reservation id for the uas
            %   telemetryInfo (array): The reported Telemetry of the particular
            %       uas
            %   sensory (array): The reported sensory informaiton from the
            %       radars
            %   del_dis (float): The distance away from the reservation lane
            %   del_speed (float): The difference in speed from the reservation
            %   projeciton (float): The distance along the given lane
            %   rogue (boolean): Indicates if any rogue behavior is detected.
            % Set up parameters
            
            % Set up Parameters
            telemetryInfo = [];
            res_id = "";
            uas_id = "";
            headway = 5;
            rogue = 0;

            % Grab the data group
            [rows, ~] = find(idx == uniIdx(group));
            cluster = datapts(rows, :);

            % Check which UAS belongs to this group
            [uas_index, ~] = find(ismember(obj.telemetry.pos,cluster, ...
                'rows') == 1);
            [sen_index, ~] = find(ismember(cluster,obj.telemetry.pos,...
                'rows') == 1);

            % Grab the associated Sensory Informaiton
            sensory = cluster;
            sensory(sen_index) = [];

            % Telemetry Data Not Being Transmitted
            if(isempty(uas_index))
                % Check rogue_uas list - if already in the list
                % If not add the rogue_uas to the list, give id
                % - project to lane, uas_id
                % Use UAS from rogue list - see if there is a
                % reservation for this rogue uas and set res.
                % If not in list - create new uas
                uas = [];
                obj.rogue_uas = [obj.rogue_uas; uas];
                rogue = 1;
            elseif (~isempty(res))% Grab existing reservation
                [rows, ~] = find(res.uas_id == ...
                    obj.telemetry.id(uas_index) & ...
                    res.entry_time_s <= obj.time & ...
                    res.exit_time_s >= obj.time);
                res = res(rows, :);
            end

            % There was no reservation data
            if(size(res, 1) ~= 1)
                % Check rogue_uas list - if already in the list. If not add
                % the rogue_uas to the list, give id,project to lane,
                % uas_id
                rogue = 1;
                lane_id = obj.findClosestLane(obj.telemetry(uas_index));
            else
                lane_id = res.lane_id;
            end

            % Find the previous material
            [rows, ~] = find(obj.masterList.uas_id == uas_id);
            pre_info = obj.masterList(rows(end), :);

            % Perform Analysis
            [del_dis, del_speed, projection, aRogue] = ...
                obj.PerformAnalysis(obj.telemetry(uas_index), ...
                pre_info, res);

            % Check headway distances
            [rDis, ~] = find(dist <= headway & dist > 0);
            if(~isempty(rDis))
                rogue = 1;
            end

            rogue = rogue | aRogue;
        end
        
        % Finds The closest lane based on uas position
        function lane_id = findClosestLane(obj, uasPoint)
            % findClosestLane - This is a helper function that will find the
            % closest lane for a uas that doesn't have a reservation.
            % Grab all the lane information
            lane_ids = obj.lbsd.getLaneIds();
            minDis = Inf;
            % Loop through all the lanes to find the associated
            % lane
            for index = 1:length(lane_ids)
                ids = obj.lbsd.getLaneVertexes(lane_ids(index));
                pos = obj.lbsd.getVertPositions(ids);
                mid = (pos(2,:) + pos(1, :))/2;
                dis = norm(uasPoint - mid);
                if(dis < minDis)
                    minDis = dis;
                    lane_id = lane_ids(index);
                end
            end
        end
        
        % Main Function to Perform the flight analysis
        function [del_dis, del_speed, projection, aRogue] = ...
                obj.PerformAnalysis(uas_pos, pre_info, res, lane_id)
        % PerformAnalysis - This function analyzes the deivation in the
        % planed flight behaviors
        % Input:
        %   uas_pos (1x3 array): x,y,z coordinates of the uas
        %   pre_info (1x4 array): The previous informaiton that was
        %       gathered for the particular uas
        %   res (array): The reservation information for the particular uas
        % Ouput:
        %   del_dis (float): Distance difference from reservation
        %   del_speed (float): Speed difference from reservation
        %   projection (float): How far into the lane is the uas
        %   aRogue (boolean): Whether there was adnormal behavior 
        %
            del_dis = 0;
            del_speed = 0;
            aRogue = 0;
            lanes = obj.getPosition(lane_id);
            
            % Grab Reservation information
            if(~isempty(res))
                % Distance Difference
                planned_pos = obj.PlannedPosition(res, lanes);
                del_dis = norm(uas_pos - planned_pos);

                % Check if del_dis is over a certain point
                    % If so aRogue = 1

                % Speed Difference
                schedule_speed = obj.Schedule_speed(res, pre_info.time,...
                    planned_pos, lanes);
                del_t = obj.time - pre_info.time;
                d_dis = norm(uas_pos - pre_info.telemetry.pos);
                del_speed = (d_dis/del_t) - schedule_speed;

                % Check if the speed is over a certain point
                    % If so aRogue = 1
            end

            % Project to the current lane
            del_v = uas_pos - lanes(1, :);
            dir_v = lanes(2, :) - lanes(1, :);
            dotProd = dot(del_v, dir_v);
            normLane = norm(dir_v);
            projection = (dotProd/(normLane));

        end
        
        % Grabs the lane x,y,z coordinates
        function pos = getPosition(obj, laneIndex)
            % getPosition - Obtains the starting and ending vertexes
            %   positions
            % Input
            %   laneIndex (string): The lane index
            ids = obj.lbsd.getLaneVertexes(laneIndex);
            idx = obj.lbsd.getVertPositions(ids(1, :));
            pos = [idx(1, :) idx(2, :)];
        end
        
        % Calcuates the Planned position
        function planned_pos = PlannedPosition(obj, res, lanes)
        % PlannedPosition - This function is used to calculate the planned
        % position for the given time. 
        % Input:
        %   obj (atoc handle)
        %   res (reservation informaiton)
        %   time (float): previous time stamp
        %   lanes (2x3 array): lane x,y,z coordinates of endpoints
        % Output:
        %   planned_pos(1x3 array): x,y,z of planned position
        %

            % Direction Vector
            dir_v = lanes(2, :) - lanes(1, :);
            del_t = obj.time - res.entry_time_s;
            planned_pos = lanes(1, :) + del_t*dir_v;
        end
        
        % Calculates the planned speed according to the time change
        function schedule_speed = Schedule_speed(obj, res, time, ...
                plan_pos,lanes)
        % Schedule_speed - This function is used to calculate the planned
        % speed of the uas given the change in time. 
        % Input:
        %   obj (atoc handle)
        %   res (reservation data)
        %   time (float): previous time stamp
        %   plan_pos (1x3): The planned position given the time
        %   lanes (2x3 array): Lanes x,y,z endpoint coordinates
        % Output:
        %   schedule_speed (float): The schedule speed of the uas 
        %   
            del_t = time - res.entry_time_s;
            dir_v = lanes(2, :) - lanes(1, :);
            prev_pos = lanes(1, :) - del_t*dir_v;
            del_dis = norm(prev_pos - plan_pos);
            schedule_speed = del_dis/(obj.time - time);
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

            for index = 1:length(lane_ids)
                density = obj.laneData(lane_ids(index)).density;
                if(~isempty(timeInterval))
                    [rows, ~] = find(density.time >= timeInteveral(1) & ...
                        density.time <= timeInteveral(2));
                    density = density(rows, :);
                end
                figure;
                plot(density.time, density.number, 'DisplayName', ...
                    strcat("Lane ", lane_ids(index), ": Lane Density"));
            end
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
            num_density = zeros(length(lane_ids), 1);
            for index = 1:length(lane_ids)
                density = obj.laneData(lane_ids(index)).density;
                if(~isempty(timeInterval))
                    [rows, ~] = find(density.time >= timeInteveral(1) & ...
                        density.time <= timeInteveral(2));
                    density = density(rows, :);
                end
                num_density(index) = sum(density.number);
            end
            pie(num_density, lane_ids);
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
        function flightTrajectory(obj, uas, timeInterval)
            % Grab the reservations from the lbsd
            % Grab the specific time interval that is associated with uas
            % and time
            % Loop through all of the lanes
            % Plot each lane ontop of one another with the projection,
            % and sensory information.
        end
    end
end