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
        trackMen % Tracker monitor handle
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
                'del_dis', [], 'del_speed', [], 'proj', [], ...
                'Classification', []);
            obj.masterList = masterList;
            obj.trackMen = TrackMonitor();

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
                
%                 % Grab Reserveration Data for the past simulation step
%                 res = obj.lbsd.getReservations();
%                 [rows, ~] = find(res.entry_time_s <= obj.time & ...
%                     res.exit_time_s >= obj.time);
%                 res = res(rows, :);
%                 
%                 % Check if there is any flight information
%                 if(~isempty(obj.telemetry(end).ID) ...
%                         || ~isempty(obj.radars(end).ID) || ~isempty(res))
%                     % Send Informaiton to the track monitor system
%                     obj.trackMen.GatherData(obj.telemetry, obj.radars, res);
%     
%                     % Update Master list based on Track Monitor Informaiton
%                    UpdateMasterList(obj);
%                 end
                
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
                obj.telemetry{end + 1, {'ID', 'pos', 'speed', 'time'}} ...
                = [src.id, [src.gps.lon, src.gps.lat, src.gps.alt], ...
                [src.gps.vx, src.gps.vy, src.gps.vz], obj.time];
            end
            if event.EventName == "Detection"
                % This event is used to store all of the sensory information
                % during a simulation step

                for item = 1:size(src.targets, 2)
                    obj.radars{end + 1, {'ID', 'pos', 'speed', 'time'}}...
                        = [src.ID, [src.targets(item).x, ...
                        src.targets(item).y, src.targets(item).z],...
                        [src.targets(item).s], obj.time];
                end
            end
        end
        function createRadarTelemetryData(obj)
            % createRadarTelemetryData - Creates the data structure for the
            %   sensor and telemetry data produced from the simulation
            % Input:
            %   obj (ATOC Handle) - ATOC instance object
            % Output:
            % Call:
            %   atoc.CreateRadarTelemetryData();
            
            tnew = table();
            tnew.ID = "";
            tnew.pos = zeros(1, 3);
            tnew.speed = zeros(1,3);
            tnew.time = 0;
            obj.radars = tnew;
            obj.telemetry = tnew;
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

            % Steps:
            %   1. Pull information from the track monitior
            %   2. Update MasterList
            %       a. Add telemetry Information
            %           1. uas_id, speed, pos
            %       b. Add Sensory Information
            %           1. radar_id, target position, speed
            %       c. Classification - tracker
            %           1. UAS, Hobbist, Analomy, Normal
            %       d. Reserveration information
            %           1. lane_id, res_id
            %       e. Analysis of Flight
            %           1. Del_speed/dis, projection to lane, etc

%             % Grab classifed flight behaviors
%             flightBehaviors = obj.trackMen.classifiedFlights;
% 
%             % Add information to MasterList
%             for behavior = 1:size(flightBehaviors, 1)
%                 obj.AddEntry(flightBehaviors(behavior));
%             end

            % Clear previous information
            obj.createRadarTelemetryData();
        end
        % Helper Method that Adds Entry To MasterList
        function AddEntry(obj, oneFlight)
            % AddEntry - This is a helper function that adds an entry into the
            % Master List
            % input:
            %   oneFlight (struct) - contains all of the information for a
            %   particular flight behavior during a simulation step
            %

            % Update the master list
            obj.masterList(obj.indexer) = struct('time', obj.time, ...
                'lane_id', oneFlight.lane_id, 'uas_id', oneFlight.uas_id,...
                'res_id', oneFlight.res_id, 'telemetry', oneFlight.tel, ...
                'sensory', oneFlight.sen, 'del_dis', oneFlight.dis,...
                'del_speed', oneFlight.speed, 'proj', oneFlight.proj,...
                'Classification', oneFlight.classify);
            obj.indexer = obj.indexer + 1;
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