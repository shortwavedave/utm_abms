classdef TrackMonitor < handle
    % TrackMonitor - This class classifies any flight behaviors within
    % simulation at any given simulation step. It also manages all of the
    % trackers.

    properties
        trackers % A list of all of the current trackers flighing
        update_listers % A list of all of the tracker listeners
        flights % A list of all of the current flights
        del_t % Change in time
        rowIndex % Keeps track of the number of items in the structure
        time % Keeps track of the time
        lbsd % Lane system handle
    end

    properties (Access=private)
        laneModel % KD - Tree of the lane system.
        removed % a parallel array to keep track of the removed flights
    end

    events
        UpdateModel
    end
    %% Public Driver Functionss
    % This section has all of the public functions that are used as a
    % driver to analyze and classify flight behaviors for the UAS. 
    %
    methods
        function obj = TrackMonitor()
            % TrackMonitor - This is the constructor to create a track
            % monitor object to manage trackers.
            obj.trackers = [];
            obj.removed = zeros(1, 100);
            obj.update_listers = [];
            obj.del_t = 0;
            telemetry = struct("ID", "", "pos", [0,0,0], "speed", [0,0,0], ...
                "time", 0);
            obj.flights = struct('lane_id', "", 'uas_id', "",'res_id', "", ...
                'telemetry', telemetry, 'sensory', telemetry, 'del_dis', 0,...
                'del_speed', 0, 'proj', 0, 'tracker_id', "", ...
                'Classification', "normal");
            obj.flights = repmat(obj.flights, 100, 1);
            obj.rowIndex = 1;
            obj.time = 0;
        end
        function initializeLaneStructor(obj, lbsd)
            % initializeLaneStructor - Creates a KD tree from the lane
            % system that is used for flight behavior analysis.
            % Input:
            %   lbsd (lbsd handle) the current lane structure
            % Output:
            % Call:
            %   monitor.initializeLaneStructor(lbsd);

            % Grab all of the lanes from the lbsd object
            obj.lbsd = lbsd;
            lane_ids = lbsd.getLaneIds();
            lanes = zeros(size(lane_ids,1), 6);

            for index = 1:size(lane_ids, 1)
                ids = lbsd.getLaneVertexes(lane_ids(index));
                p1 = lbsd.getVertPositions(ids(1));
                p2 = lbsd.getVertPositions(ids(2));
                lanes(index, :) = [p1, p2];
            end

            obj.laneModel = TrackMonitor.LEM_lanes2model(lanes, 2);  
        end
        function subscribe_to_updates(obj, subscriber)
            % subscribe_to_updates: This function is used for the trackers
            % to subscribe to update after each simulation step.
            % Inputs:
            %   obj (trackMonitor) - TrackMonitor handle
            %   subscriber (tracker) - tracker handle
            lh = obj.addlistener('UpdateModel', subscriber);
            obj.update_listers = [obj.update_listers; lh];
        end
        function AnalyzeFlights(obj, UASInfo, RadarInfo, res, del_t)            
            % GatherData - This function is used to gather informaiton from
            % each simulation step. The main purpose of this funciton is to
            % identify and classify flight patterns happening in the
            % simulation.
            % Input:
            %   UASInfo (struct) - Telemetry data gathered current step
            %   RadarInfo (struct) - Sensory information gathered current
            %       step
            %   res (struct) - reservation information pertaining to the
            %   current step
            
            % Update time trackers
            obj.del_t = del_t;

            % Find Associative Trackers
            obj.FindAssociativeTrackers(UASInfo, RadarInfo,res);

            % ClassifyFlightBehaviors
            obj.ClassifyFlightBehaviors();
            
            % Remove Finished Flights
            obj.removeDoneFlights();

            % Update Models - For each tracker if changed update.
            notify(obj, 'UpdateModel');

            % Update time 
            obj.time = obj.time + obj.del_t;
        end
        function flightInformation = retrieveFlightInformation(obj)
            % retrieveFlightInformation - retrieves the information of the
            % flight information at the given time stamp.
            flightInformation = obj.flights(1:obj.rowIndex);
        end
    end

    %% Classification Helper Methods
    % This section is all of the helper methods that are used to classify
    % the flight of the behavior of the uas
    %
    methods(Access = private)
        function ClassifyFlightBehaviors(obj)
            % ClassifyFlightBehaviors - Classifies the flight behaviors of
            % all the information that has been received during the
            % simulation step.
            % Input:
            %   obj (track Monitor Handle)

            % Loop through each of the trackers
            for trackIndex = 1:size(obj.trackers,1)
                curTracker = obj.trackers(trackIndex);
                % Analyze every six steps
                timeToAnalyze = size(curTracker.traj, 1) > 99 && ...
                    mod(size(curTracker.traj, 1), 1000) == 0;
                trackerFinished = (curTracker.disconnected && ...
                    (size(curTracker.traj, 1)>1));
                if(trackerFinished || (curTracker.active && timeToAnalyze))
                    traj = [curTracker.traj(:, 1:3), curTracker.time];
                    M = TrackMonitor.LEM_traj_measures(obj.laneModel, ...
                        traj, norm(curTracker.traj(end, 4:6)), obj.del_t);

                    if(TrackMonitor.LEM_check_hobbist2(traj)) % Trying to Mix
                        obj.updateFlightBehavior(curTracker.ID, "Hobbist Two");
                        continue;
                    end

                    if(TrackMonitor.LEM_check_rogue(traj)) % Working on it.
                        obj.updateFlightBehavior(curTracker.ID, "Rogue One");
                        continue;
                    end
                    
                    if(TrackMonitor.LEM_check_normal(M))
                        obj.updateFlightBehavior(curTracker.ID, "normal");
                        continue;
                    end

                    if(TrackMonitor.LEM_check_hobbist1(traj))
                        obj.updateFlightBehavior(curTracker.ID, "Hobbist One");
                        continue;
                    end

                    if(TrackMonitor.LEM_check_hobbist3(traj))
                        obj.updateFlightBehavior(curTracker.ID, "Hobbist Three");
                        continue;
                    end

                    obj.updateFlightBehavior(curTracker.ID, "Rogue Two");
                end
            end
        end
        function updateFlightBehavior(obj, tracker_id, behavior)
            % updateFlightBehavior - Updates the flight behavior for a
            % specific uas based on their tracker information. 
            % Input:
            %   obj (Track Monitor Handle)
            %   tracker_id (string): the tracker id
            %   behavior (string): the new flight classification

            [row, ~] = find([obj.flights.tracker_id] == tracker_id);
            if(~isempty(row))
                obj.flights(row(end)).Classification = behavior;
            end
        end
        function AddClassifyFlights(obj,...
                tel_info, sen_info, track_id,lane_id, res_id, ...
                del_dis, del_speed, projection)
            % addClassifyFlights - Adds flights to the main list that
            % gathers the information from the flights during the
            % simulation.
            % Input:
            %   tel_info (struct): the uas information
            %   sen_info (struct): the sensory information
            %   track_id (string): the tracker id
            %   lane_id (string): The lane id that the uas is closest to
            %   res_id (string): the reservation id for the uas
            %   del_dis (float): the change of distance from planned and
            %      actual
            %   del_speed (float): the change in speed from planned and
            %      actual
            %   projection (float): the projection distance of how far the
            %      uas is in the lane. 
            
            [row, ~] = find([obj.flights.tracker_id] == track_id);
            if(~isempty(row))
                update = row(1);
            else
                update = obj.rowIndex;
                obj.rowIndex = obj.rowIndex + 1;
            end

            obj.flights(update).lane_id = lane_id;
            obj.flights(update).uas_id = tel_info.ID;
            obj.flights(update).res_id = res_id;
            obj.flights(update).telemetry = tel_info;
            obj.flights(update).sensory = sen_info;
            obj.flights(update).del_dis = del_dis;
            obj.flights(update).del_speed = del_speed;
            obj.flights(update).proj = projection;
            obj.flights(update).tracker_id = track_id;
            if(isempty(obj.flights(update).Classification))
                obj.flights(update).Classification = "normal";
            end
        end
    end
    %% Tracker Maintance
    % This section has all of the helper methods that are used to maintain
    % the tracker information. 
    % 
    methods(Access = private)
        function FindAssociativeTrackers(obj, UASInfo, RadarInfo,res)
            % FindAssociativeTrackers - Links the telemetry information and
            % sensory information to the correct tracker object in the list
            % otherwise it creates a new tracker object.
            % Input
            %   obj (tracker monitor handle)
            %   UASInfo (struct): Telemetry Information
            %   RadarInfo (struct): Sensory Information
            %   res (Table): The reserveration information

            % Cluster the given data points
            [hasData, datapts] = TrackMonitor.hasInformation(UASInfo, RadarInfo);
            if(~hasData), return; end

            [idx, ~] = dbscan(datapts, 3, 1);

            uniqueID = unique(idx);
            for group = 1:size(uniqueID)
                uniIDx = uniqueID(group);
                [rows, ~] = find(uniIDx ==  idx);
                cluster = datapts(rows, :);

                [tel_info, sen_info] = ...
                    TrackMonitor.GetRelatedData(cluster, UASInfo, RadarInfo);
                track_id = obj.FindTrackerObject(tel_info, sen_info);
                
                [lane_id, res_id, del_dis, del_speed, projection] ...
                    = obj.PerformAnalysisDriver(tel_info, res, track_id);
                
                for i = 1:size(tel_info,1)
                    obj.AddClassifyFlights(tel_info(i), ...
                        sen_info, track_id, lane_id, res_id, del_dis, del_speed, ...
                        projection);
                end
            end

        end
        function track_id = FindTrackerObject(obj, tel_info, sen_info)
            % FindTrackerObject - Finds an associated Tracker for the given
            % information otherwise it creates a new tracker object.
            % Input:
            %   obj (track monitor handle)
            %   tel_info (struct): Telemetry Information
            %   sen_info (struct): Sensory Information
            % Output:
            %   Track_id (string): tracker Identification
            track_id = [];

            if(~isempty(tel_info(end).ID))
                itemPos = tel_info(end).pos;
                itemSpeed = tel_info(end).speed;
            elseif(~isempty(sen_info(1).ID))
                if(size(sen_info, 1) > 1)
                    itemPos = mean([sen_info.pos]);
                    itemSpeed = mean([sen_info.speed]);
                else
                    itemPos = sen_info.pos;
                    itemSpeed = sen_info.speed;
                end
            else
                return
            end

            % Find the tracker object
            for index = 1:size(obj.trackers)
                t = obj.trackers(index);
                pos = t.pos(1:3);
                % Found the correct tracker
                dif = norm(transpose(pos) - itemPos(end, :));
                if(dif < 4)
                    t.RecieveObservationData(tel_info, sen_info);
                    track_id = t.ID;
                    break;
                end
            end

            % New UAS Needs New Tracker
            if(isempty(track_id))
                pos = [transpose(itemPos); transpose(itemSpeed)];
                new_tracker = Tracker(pos);
                new_tracker.ID = num2str(size(obj.trackers, 1));
                new_tracker.active = true;
                obj.trackers = [obj.trackers; new_tracker];
                obj.subscribe_to_updates(@new_tracker.start_update);
                track_id = new_tracker.ID;
            end
        end
        function lane_id = findClosestLane(obj, uasPoint)
            % findClosestLane - This is a helper function that will find the
            % closest lane for a uas that doesn't have a reservation.
            % Grab all the lane information
            
            lane_ids = obj.lbsd.getLaneIds();
            minDis = Inf;
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
        function removeDoneFlights(obj)
            % removedDoneFlights - this is the main method that removes the
            % flight information of flights that are finished flying. 
            % Input:
            %   obj (track monitor handle)
            %
            if(length(obj.removed) < length(obj.trackers))
                obj.removed = [obj.removed, zeros(1, 100)];
            end

            for index = 1:length(obj.trackers)
                if(~obj.trackers(index).active &&...
                        ~obj.trackers(index).disconnected && ...
                        ~obj.removed(index))
                    obj.removeFlight(obj.trackers(index).ID, index);
                end
            end
        end
        function removeFlight(obj, id, index)
        % removeFlight - used to remove the specific flight information
        % from the flight list
        % Input:
        %   obj (track monitor handle)
        %   id (string): the tracker id
        %
            [row, ~] = ...
                find([obj.flights.tracker_id] == id);
            if(~isempty(row))
                obj.flights(row) = [];
                obj.rowIndex = obj.rowIndex - 1;
                obj.removed(index) = 1;
            end
        end
    end

    %% Analysis Helper Functions
    % This section has all of the helper functions that perform flight
    % analysis for individual uas. 
    methods(Access = private)
        function [lane_id, res_id, del_dis, del_speed, projection] = ...
                PerformAnalysisDriver(obj, tel_info, res, tracker_id)
            % PerformAnalysis - Analysis of the flight behavior with
            % regards with changing in speed, distance, headway distances,
            % and projections. 
            % Input:
            %   tel_info (struct): Telemetry information for a particular
            %       flight
            %   sen_info (struct): Sensory information for a particular
            %       flight
            %   res (struct) : reservation information
            % Output:
            %   lane_id (string): the closest lane
            %   res_id (string): the corsponding reservation information
            %   del_dis (float): The change in distance from the planned
            %   del_speed (float): The change in speed from the planned
            %   projection (float): The distance away from the actual
            %   distance. 

            lane_id = "";
            res_id = "";
            del_dis = 0;
            del_speed = 0;
            projection = 0;

            if(isempty(tel_info(end).ID))
                return;
            end
            
            uas_id = tel_info(end).ID;
            if(~isempty(res))
                [rows, ~] = find(res.uas_id == uas_id & ...
                    res.entry_time_s <= obj.time &...
                    res.exit_time_s >= obj.time);
            else
                rows = [];
            end
            
            if(~isempty(rows))
                res = res(rows(end), :);
                res_id = res.id;
                lane_id = res.lane_id;
            else
                lane_id = obj.findClosestLane(tel_info(end).pos);
            end
            rows = [];
            for index = 1:size(obj.trackers, 1)
                if(obj.trackers(index).ID == tracker_id)
                    rows = index;
                    break;
                end
            end
            if(isempty(rows))
                pre_info = [];
            else
                if(isempty(obj.trackers(rows(end)).traj))
                    pre_info = obj.trackers(rows(end)).pos(1:3)';
                else
                    pre_info = obj.trackers(rows(end)).traj(end, 1:3);
                end
            end

            [del_dis, del_speed, projection] = obj.PerformAnalysis(...
                tel_info(end).pos, pre_info, res, lane_id);

        end
        function [del_dis, del_speed, projection] = ...
                PerformAnalysis(obj, uas_pos, pre_info, res, lane_id)
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
            %

            ids = obj.lbsd.getLaneVertexes(lane_id);
            lanes = obj.lbsd.getVertPositions(ids(1, :));

            [del_dis, del_speed] = obj.findChangeInSpeedAndDistance(res, ...
                pre_info(1:3), uas_pos, lanes);

            % Project to the current lane
            del_v = uas_pos - lanes(1, :);
            dir_v = lanes(2, :) - lanes(1, :);
            dotProd = dot(del_v, dir_v);
            normLane = norm(dir_v);
            projection = (dotProd/(normLane));

        end
        function [del_dis, del_speed] = findChangeInSpeedAndDistance(...
                obj, res, pre_info,uas_pos, lanes)
            % findChangeInSpeedAndDistance - finds the change in distance
            % and speed from the planned and the actual
            % Input:
            %   obj (track monitor handle)
            %   res (table): reserveration data
            %   pre_info (1x3) the previous position
            %   uas_pos (1x3) the current position
            %   lanes (2 x 3) the lane positions that the uas is in.
            % Output:
            %   del_dis (float): the difference between actual and planned
            %       distant
            %   del_speed (float): the difference between actual and
            %      planned speed
            % Call:
            %   obj.findChangeInSpeedAndDistance(res, [x1, x2, x3],
            %            [x1, x2, x3; y1, y2, y3]
            del_dis = 0;
            del_speed = 0;

            if(~isempty(res))
                dir_v = lanes(2, :) - lanes(1, :);
                dir_v = dir_v/norm(dir_v);
                delt = obj.time - res.entry_time_s;
                planned_pos = lanes(1, :) + delt*dir_v;

                del_dis = norm(uas_pos - planned_pos);

                if(~isempty(pre_info) && obj.time ~= 0)
                    prev_time = obj.time - obj.del_t;
                    prev_uas_pos = pre_info;

                    delt = prev_time - res.entry_time_s;
                    dir_v = lanes(2, :) - lanes(1, :);
                    dir_v = dir_v/norm(dir_v);

                    prev_pos = lanes(1, :) + delt*dir_v;
                    dis = norm(prev_pos - planned_pos);
                    schedule_speed = dis/(obj.time - prev_time);
                    delt = obj.del_t;
                    d_dis = norm(uas_pos - prev_uas_pos);
                    del_speed = (d_dis/delt) - schedule_speed;
                end
            end
        end
    end

    %% Static Methods
    % This section deals with the all of the functions that came from
    % Professor Henderson and are being intergrated.
    methods(Static)
        model = LEM_lanes2model(lanes,del_x);
        isHobbist = LEM_check_hobbist1(traj);
        isHobbist = LEM_check_hobbist2(traj);
        isHobbist = LEM_check_hobbist3(traj);
        [r1, r2] = LEM_check_rogue(traj);
        [radius, center] = LEM_3pts2circle(p1, p2, p3);
        [p,s] = CV_total_LS(x,y);
        M = LEM_traj_measures(model, traj, speed, del_t);
        model = LEM_traj2model(traj);
        function normal = LEM_check_normal(M)
            % LEM_check_normal - Checks to see if the measures of the
            %   trajectory are within normal limits
            % Input:
            %   M (k-1 x 4 array): traj measures
            %       (:,1): distance from traj point to closest model point
            %       (:,2): cosine of angle between traj & model directions
            %       (:,3): lane change measure (number of bad lane changes)
            %       (:,4): lane sequence
            %       (:,5): stationary 
            % Output:
            %   normal (boolean) - Indicates if the flight is normal
            % Call:
            %   normal = TrackMonitor.LEM_check_normal(M)
            % Author:
            %     T. Henderson
            %     UU
            %     Spring 2021
            %
            DIST_THRESH = 1.4;
            COS_THRESH = 0.8;
            normal = 0;
            if median(M(:,1))<DIST_THRESH && median(M(:,2))>COS_THRESH
                normal = 1;
            end
        end
        function [tel_info, sen_info] = GetRelatedData(cluster, ...
                UASInfo, RadarInfo)
            % GetRelatedData - Finds the telemetry and sensory data based
            % on the clustered group.
            % Input:
            %   obj (Track Monitor Handle)
            %   cluster (array) - All of the telemetry and sensory
            %       information
            %   UASInfo (Struct) - telemetry table.
            %   RadarInfo (Struct) - Sensory table.
            %
            tel_info = struct("ID", "", "pos", [0,0,0], "speed", [0,0,0], ...
                "time", 0);
            sen_info = struct("ID", "", "pos", [0,0,0], "speed", [0,0,0], ...
                "time", 0);
            sen_info = repmat(sen_info, 100, 1);
            index = 1;

            for data = 1:size(cluster, 1)
                uas_index = [];
                sen_index = [];
                if(~isempty(UASInfo(1).ID))
                    pts = [UASInfo.pos];
                    pts = reshape(pts, [], 3);
                    [uas_index, ~] = find(ismember(pts,cluster(data, :), ...
                        'rows') == 1);
                elseif(~isempty(RadarInfo(1).ID))
                    pts = extractfield(RadarInfo,'pos');
                    pts = reshape(pts, [], 3);
                    [sen_index, ~] = find(ismember(pts, ...
                        cluster(data, :), 'rows') == 1);
                end
                if(~isempty(uas_index))
                    tel_info = UASInfo(uas_index, :);
                elseif(~isempty(sen_index))
                    sen_info(index:length(sen_info)) = RadarInfo(sen_index);
                    index = index + length(sen_info);
                end
            end
            
            sen_info(index+1:end) = [];
        end
        function [hasData, datapts] = hasInformation(UASInfo, RadarInfo)
            % hasInformation - Checks if there is information for flight
            % classification. 
            % Input:
            %   obj (track monitor handle)
            %   UASInfo (table): all of telemetry information
            %   RadarInfo (table): all of the radarInformation
            % Output:
            %   hasData (boolean): indicates if there was information
            %   datapts (n x 3): all of the information gathered for
            %      clustering
            datapts = [zeros(size(UASInfo, 1), 3);...
                zeros(size(RadarInfo, 1), 3)];
            i = 1;
            hasData = false;
            for index = 1:size(UASInfo, 1)
                if(~isempty(UASInfo(index).ID))
                    datapts(i, :) = UASInfo(index).pos;
                    i = i+1;
                end
            end
            for index = 1:size(RadarInfo, 1)
                if(~isempty(RadarInfo(index).ID))
                    datapts(i, :) = RadarInfo.pos;
                    i = i + 1;
                end
            end

            if(i == 1)
                return;
            else
                datapts(i:end, :) = [];
            end
            hasData = true;
        end
    end
end
