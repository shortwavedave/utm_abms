classdef TrackMonitor < handle
    % TrackMonitor - This class classifies any flight behaviors within
    % simulation at any given simulation step. It also manages all of the
    % trackers.

    properties
        tackers % A list of all of the current trackers flighing
        update_listers % A list of all of the tracker listeners
        flights % A list of all of the current flights
        del_t % Change in time
    end

    properties (Access=private)
        laneModel % KD - Tree of the lane system.
    end

    events
        UpdateModel
    end

    methods
        function obj = TrackMonitor()
            % TrackMonitor - This is the constructor to create a track
            % monitor object to manage trackers.
            obj.tackers = [];
            obj.update_listers = [];
            obj.del_t = 0;
            obj.flights = struct("uas_id", [], "telemetry", [], ...
                "sensory", [], "tracker_id", [], "classification", []);
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
            %   telemetry (struct) - Telemetry data gathered current step
            %   sensory (struct) - Sensory information gathered current
            %       step
            %   res (struct) - reservation information pertaining to the
            %   current step
            obj.del_t = del_t;

            % Find Associative Trackers
            obj.FindAssociativeTrackers(UASInfo, RadarInfo,res,1);

            obj.ClassifyFlightBehaviors(res);
            
            % Update Models - For each tracker if changed update.
            notify(obj, 'UpdateModel');
        end
    end

    %% Needs to Be Modified
    % Taken from ATOC Class Will have to Update for this class.
    methods(Access = private)
        function ClassifyFlightBehaviors(obj, res)
            % ClassifyFlightBehaviors - Classifies the flight behaviors of
            % all the information that has been received during the
            % simulation step.

            % Loop through each of the trackers
            for trackIndex = 1:size(obj.tackers,1)
                curTracker = obj.tackers(trackIndex);
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

%                     if(false)
                    obj.updateFlightBehavior(curTracker.ID, "Rogue Two");
%                     end
                end
            end
        end

        function updateFlightBehavior(obj, track_id, behavior)
            % updateFlightBehavior - used to update the classification of
            % the flights based on the simulation
            row = find(strcmp({obj.flights.tracker_id}, ...
                'tracker_id')==str2num(track_id));
            if(~isempty(row))
                obj.flights(row).classification = behavior;
            end
        end

        function FindAssociativeTrackers(obj, UASInfo, RadarInfo,res,indexer)
            % FindAssociativeTrackers - Links the telemetry information and
            % sensory information to the correct tracker object in the list
            % otherwise it creates a new tracker object.
            % Input
            %   obj (tracker monitor handle)
            %   UASInfo (Table): Telemetry Information
            %   RadarInfo (Table): Sensory Information

            % Cluster the given data points
            [hasData, datapts] = obj.hasInformation(UASInfo, RadarInfo);
            if(~hasData), return; end

            [idx, ~] = dbscan(datapts, 3, 1);

            uniqueID = unique(idx);
            for group = 1:size(uniqueID)
                % Find the group associated with the telemetry
                uniIDx = uniqueID(group);
                [rows, ~] = find(uniIDx ==  idx);
                cluster = datapts(rows, :);

                [tel_info, sen_info] = obj.GetRelatedData(cluster, UASInfo, ...
                    RadarInfo);
                track_id = obj.FindTrackerObject(tel_info, sen_info,res);
                for i = 1:height(tel_info)
                    indexer = obj.AddClassifyFlights(indexer, tel_info(i, :), ...
                        sen_info, track_id);
                end
            end

        end
        
        function [hasData, datapts] = hasInformation(obj, UASInfo, RadarInfo)
            % hasInformation - Checks if there is information for flight
            % classification. 
            datapts = [];
            hasData = false;
            if(RadarInfo.ID(end) ~= "" && UASInfo.ID(end) ~= "")
                datapts = [UASInfo.pos; RadarInfo.pos];
            elseif(UASInfo.ID(end) ~= "")
                datapts = [UASInfo.pos];
            elseif(RadarInfo.ID(end) ~= "")
                datapts = [RadarInfo.pos];
            else
                return
            end
            hasData = true;
        end
        function indexer = AddClassifyFlights(obj,indexer,...
                tel_info, sen_info, track_id)
            % addClassifyFlights - Adds flights to the main list that
            % gathers the information from the flights during the
            % simulation.
            
            row = find(strcmp({obj.flights.tracker_id}, ...
                'tracker_id')==str2num(track_id));
            if(~isempty(row))
                update = row(1);
            else
                update = indexer;
                indexer = indexer + 1;
            end

            info = tel_info;
            obj.flights(update).uas_id = info.ID;
            obj.flights(update).telemetry.pos = info.pos;
            obj.flights(update).telemetry.speed = info.speed;
            obj.flights(update).telemetry.time = info.time;
            obj.flights(update).sensory = table2struct(sen_info);
            obj.flights(update).tracker_id = track_id;
            if(isempty(obj.flights(update).classification))
                obj.flights(update).classification = "normal";
            end
            
        end

        function track_id = FindTrackerObject(obj, tel_info, sen_info, res)
            % FindTrackerObject - Finds an associated Tracker for the given
            % information otherwise it creates a new tracker object.
            % Input:
            %   obj (track monitor handle)
            %   tel_info (table): Telemetry Information
            %   sen_info (table): Sensory Information
            % Output:
            %   Track_id (string): tracker Identification

            track_id = [];

            if(~isempty(tel_info))
                itemPos = tel_info.pos;
                itemSpeed = tel_info.speed;
            elseif(~isempty(sen_info))
                itemPos = sen_info.pos(end, :);
                itemSpeed = sen_info.speed(end, :);
            else
                return
            end

            % Find the tracker object
            for index = 1:size(obj.tackers)
                t = obj.tackers(index);
                pos = t.pos(1:3);
                % Found the correct tracker
                dif = norm(transpose(pos) - itemPos(end, :));
                if(dif < 4)
                    % Check reserveration data
                    % If at end of reserveration data set t to
                    % inactive.
                    t.RecieveObservationData(tel_info, sen_info);
                    track_id = t.ID;
                    break;
                end
            end

            % New UAS Needs New Tracker
            if(isempty(track_id))
                pos = [transpose(itemPos); transpose(itemSpeed)];
                new_tracker = Tracker(pos);
                new_tracker.ID = num2str(size(obj.tackers, 1));
                new_tracker.active = true;
                obj.tackers = [obj.tackers; new_tracker];
                obj.subscribe_to_updates(@new_tracker.start_update);
                track_id = new_tracker.ID;
            end
        end

        function [tel_info, sen_info] = GetRelatedData(obj, cluster, ...
                UASInfo, RadarInfo)
            % GetRelatedData - Finds the telemetry and sensory data based
            % on the clustered group.
            % Input:
            %   obj (Track Monitor Handle)
            %   cluster (array) - All of the telemetry and sensory
            %       information
            %   UASInfo (table) - telemetry table.
            %   RadarInfo (table) - Sensory table.
            %
            tel_info = [];
            sen_info = [];
            for data = 1:size(cluster, 1)
                [uas_index, ~] = find(ismember(UASInfo.pos,cluster(data, :), ...
                    'rows') == 1);
                if(~isempty(uas_index))
                    tel_info = UASInfo(uas_index, :);
                else
                    [sen_index, ~] = find(ismember(RadarInfo.pos, ...
                        cluster(data, :), 'rows') == 1);
                    if(~isempty(sen_index))
                        if(isempty(sen_info))
                            sen_info = RadarInfo(sen_index, :);
                        else
                            ID = RadarInfo.ID(sen_index);
                            pos = RadarInfo.pos(sen_index, :);
                            speed = RadarInfo.speed(sen_index, :);
                            time = RadarInfo.time(sen_index);
                            sen_info{end+1, {'ID', 'pos', 'speed', 'time'}} ...
                                = [ID, pos, speed, time];
                        end
                    end
                end
            end
            if(isempty(sen_info))
                sen_info = table();
            elseif(isempty(tel_info))
                tel_info = table();
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
    end
    %% Rogue Detection
    % This section deals with identify any Rogue Behaviors of UAS' during
    % the life of the simulation, and then notify when certain behaviors
    % are observed
    %

    %     methods
    %
    %         % Main Method to Analyze Flights
    %         function [lane_id, uas_id, res_id, telemetryInfo, sensory, ...
    %                 del_dis, del_speed, projection, rogue] = ...
    %                 AnalyzeFlight(obj, res, dist, idx, uniIdx, datapts)
    %             % AnalyzeFlight - This function is a helper function that performs
    %             % the neccesary operations needed to update the masterlist in the
    %             % ATOC class.
    %             % Input:
    %             % Output:
    %             %   lane_id (string): the lane number the uas is in
    %             %   uas_id (string): the id of the uas
    %             %   res_id (string): the reservation id for the uas
    %             %   telemetryInfo (array): The reported Telemetry of the particular
    %             %       uas
    %             %   sensory (array): The reported sensory informaiton from the
    %             %       radars
    %             %   del_dis (float): The distance away from the reservation lane
    %             %   del_speed (float): The difference in speed from the reservation
    %             %   projeciton (float): The distance along the given lane
    %             %   rogue (boolean): Indicates if any rogue behavior is detected.
    %             % Set up parameters
    %
    %             % Set up Parameters
    %             telemetryInfo = [];
    %             res_id = "";
    %             uas_id = "";
    %             headway = 5;
    %             rogue = 0;
    %
    %             % Grab the data group
    %             [rows, ~] = find(idx == uniIdx);
    %             cluster = datapts(rows, :);
    %
    %             tel_pos = zeros(length(obj.telemetry) - 1, 3);
    %             for index = 1:size(tel_pos,1)
    %                 tel_pos(index, :) = obj.telemetry(index + 1).pos;
    %             end
    %             % Check which UAS belongs to this group
    %             [uas_index, ~] = find(ismember(tel_pos,cluster, ...
    %                 'rows') == 1);
    %             [sen_index, ~] = find(ismember(cluster,tel_pos,...
    %                 'rows') == 1);
    %
    %             % Grab the associated Sensory Informaiton
    %             sensory = cluster;
    %             sensory(sen_index, :) = [];
    %
    %             % Telemetry Data Not Being Transmitted
    %             if(isempty(uas_index))
    %                 % Check rogue_uas list - if already in the list
    %                 % If not add the rogue_uas to the list, give id
    %                 % - project to lane, uas_id
    %                 % Use UAS from rogue list - see if there is a
    %                 % reservation for this rogue uas and set res.
    %                 % If not in list - create new uas
    %                 uas = [];
    %                 obj.rogue_uas = [obj.rogue_uas; uas];
    %                 rogue = 1;
    %             elseif (~isempty(res))% Grab existing reservation
    %                 uas_index = uas_index(end);
    %                 telemetryInfo = struct(obj.telemetry(uas_index +1));
    %                 uas_id = obj.telemetry(uas_index + 1).ID;
    %                 [rows, ~] = find(res.uas_id == uas_id & ...
    %                     res.entry_time_s <= obj.time & ...
    %                     res.exit_time_s >= obj.time);
    %                 if(~isempty(rows))
    %                     res = res(rows(end), :);
    %                 end
    %             end
    %
    %             % There was no reservation data
    %             if(size(res, 1) ~= 1)
    %                 % Check rogue_uas list - if already in the list. If not add
    %                 % the rogue_uas to the list, give id,project to lane,
    %                 % uas_id
    %                 rogue = 1;
    %
    %                 lane_id = obj.findClosestLane(obj.telemetry(uas_index).pos);
    %             else
    %                 lane_id = res.lane_id;
    %                 res_id = res.id;
    %                 headway = res.hd;
    %             end
    %
    %             % Find the previous material
    %             cols = [obj.masterList.uas_id];
    %             pre_info = [];
    %             if(~isempty(cols))
    %                 [rows, ~] = find(cols == uas_id);
    %                 pre_info = obj.masterList(rows(end), :);
    %             end
    %
    %             % Perform Analysis
    %             [del_dis, del_speed, projection, aRogue] = ...
    %                 obj.PerformAnalysis(tel_pos(uas_index(end), :), ...
    %                 pre_info, res, lane_id);
    %
    %             % Check headway distances
    %             [rDis, ~] = find(dist <= headway & dist > 0);
    %             if(~isempty(rDis))
    %                 rogue = 1;
    %             end
    %
    %             rogue = rogue | aRogue;
    %         end
    %
    %         % Finds The closest lane based on uas position
    %         function lane_id = findClosestLane(obj, uasPoint)
    %             % findClosestLane - This is a helper function that will find the
    %             % closest lane for a uas that doesn't have a reservation.
    %             % Grab all the lane information
    %             lane_ids = obj.lbsd.getLaneIds();
    %             minDis = Inf;
    %             % Loop through all the lanes to find the associated
    %             % lane
    %             for index = 1:length(lane_ids)
    %                 ids = obj.lbsd.getLaneVertexes(lane_ids(index));
    %                 pos = obj.lbsd.getVertPositions(ids);
    %                 mid = (pos(2,:) + pos(1, :))/2;
    %                 dis = norm(uasPoint - mid);
    %                 if(dis < minDis)
    %                     minDis = dis;
    %                     lane_id = lane_ids(index);
    %                 end
    %             end
    %         end
    %
    %         % Main Function to Perform the flight analysis
    %         function [del_dis, del_speed, projection, aRogue] = ...
    %                 PerformAnalysis(obj, uas_pos, pre_info, res, lane_id)
    %             % PerformAnalysis - This function analyzes the deivation in the
    %             % planed flight behaviors
    %             % Input:
    %             %   uas_pos (1x3 array): x,y,z coordinates of the uas
    %             %   pre_info (1x4 array): The previous informaiton that was
    %             %       gathered for the particular uas
    %             %   res (array): The reservation information for the particular uas
    %             % Ouput:
    %             %   del_dis (float): Distance difference from reservation
    %             %   del_speed (float): Speed difference from reservation
    %             %   projection (float): How far into the lane is the uas
    %             %   aRogue (boolean): Whether there was adnormal behavior
    %             %
    %             del_dis = 0;
    %             del_speed = 0;
    %             aRogue = 0;
    %             ids = obj.lbsd.getLaneVertexes(lane_id);
    %             lanes = obj.lbsd.getVertPositions(ids(1, :));
    %
    %             % Grab Reservation information
    %             if(~isempty(res))
    %                 % Distance Difference
    %                 dir_v = lanes(2, :) - lanes(1, :);
    %                 del_t = obj.time - res.entry_time_s;
    %                 planned_pos = lanes(1, :) + del_t*dir_v;
    %
    %                 del_dis = norm(uas_pos - planned_pos);
    %
    %                 % Check if del_dis is over a certain point
    %                 % If so aRogue = 1
    %
    %                 % Speed Difference
    %                 if(~isempty(pre_info))
    %                     prev_time = pre_info(end).time;
    %                     prev_uas_pos = pre_info(end).telemetry;
    %                     if(isempty(prev_uas_pos))
    %                         prev_uas_pos = zeros(1,3);
    %                     else
    %                         prev_uas_pos = prev_uas_pos.pos;
    %                     end
    %
    %                     if(isempty(prev_time))
    %                         prev_time = 0;
    %                     end
    %                     del_t = prev_time - res.entry_time_s;
    %                     dir_v = lanes(2, :) - lanes(1, :);
    %                     prev_pos = lanes(1, :) + del_t*dir_v;
    %                     del_dis = norm(prev_pos - planned_pos);
    %                     schedule_speed = del_dis/(obj.time - prev_time);
    %                     del_t = obj.time - prev_time;
    %                     d_dis = norm(uas_pos - prev_uas_pos);
    %                     del_speed = (d_dis/del_t) - schedule_speed;
    %                 end
    %
    %                 % Check if the speed is over a certain point
    %                 % If so aRogue = 1
    %             end
    %
    %             % Project to the current lane
    %             del_v = uas_pos - lanes(1, :);
    %             dir_v = lanes(2, :) - lanes(1, :);
    %             dotProd = dot(del_v, dir_v);
    %             normLane = norm(dir_v);
    %             projection = (dotProd/(normLane));
    %
    %         end
    %         function RemovedFromATOC()
    %             % Check to see if there is any flights happening
    %             if(~isempty(obj.telemetry(end).ID) || ~isempty(obj.radars(end).ID))
    %                 res = obj.lbsd.getReservations();
    %                 datapts = zeros((length(obj.telemetry)+ ...
    %                     length(obj.radars) - 2), 3);
    %                 counter = 1;
    %                 for i = 2:length(obj.telemetry)
    %                     if (~isempty(obj.telemetry(i).pos))
    %                         datapts(counter, :) = obj.telemetry(i).pos;
    %                         counter = counter + 1;
    %                     end
    %                 end
    %                 for i = 2:length(obj.radars)
    %                     datapts(counter, :) = obj.radars(i).pos;
    %                     counter = counter + 1;
    %                 end
    %                 % Grab the UAS from the group.
    %                 [idx, corepts] = dbscan(datapts, 3, 1);
    %                 % find the dist_matrix
    %                 [rows, ~] = find(corepts == 1);
    %                 points = datapts(rows, :);
    %                 D = pdist(points);
    %                 dist = squareform(D);
    %                 uni_idx = unique(idx);
    %                 % Add Unigroups to MasterList
    %                 for index = 1:length(uni_idx)
    %                     [lane_id, uas_id, res_id, telemetryInfo, sensory, ...
    %                         del_dis, del_speed, projection, rogue] = ...
    %                         obj.AnalyzeFlight(res, dist, idx, uni_idx(index), ...
    %                         datapts);
    %                     obj.AddEntry(lane_id,uas_id, res_id, telemetryInfo, ...
    %                         sensory, del_dis, del_speed, projection, rogue);
    %                 end
    %             end
    %
    %         end
    %     end
end
