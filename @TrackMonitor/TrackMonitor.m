classdef TrackMonitor < handle
    % TrackMonitor - This class classifies any flight behaviors within 
    % simulation at any given simulation step. It also manages all of the 
    % trackers.

    properties
        tackers % A list of all of the current trackers flighing
        update_listers % A list of all of the tracker listeners
        classifiedFlights % Struct of Finished Classified behaviors
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
            obj.classifiedFlights = struct();
        end
        function subscribe_to_updates(obj, subscriber)
            % subscribe_to_updates: This function is used for the trackers
            % to subscribe to update after each simulation step. 
            % Inputs:
            %   obj (trackMonitor) - TrackMonitor handle
            %   subscriber (tracker) - tracker handle
            lh = obj.addlistener('UpdateModel', subscriber);
            obj.update_listers = [obj.update_listers, lh];
        end
        function GatherData(obj, UASInfo, RadarInfo, res)
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

            % steps:
            %   1. Cluster the informaiton into groups
            %   2. Identify which cluster belongs to which group
            %       a. link - telemetry, sensory, reservation
            %   3. Check if cluster has tracker
            %       a. if not - add, if so - update position "feed new
            %       position"
            %   4. Classify Behavior
            %       a. Hobbist, analomy behaviors, etc.
            %   5. Send information back to ATOC
            %       a. via broadcast - ATOC can access trackers
            %   6. Tell all trackers to update model

            % Clear previous flight information

            % Cluster the given data points
            datapts = [UASInfo.pos; RadarInfo.pos];
            [idx, corepts] = dbscan(datapts, 3, 1);

            % Each groupd
                % Find Associated Tracker
                % Classify the behavior
            
            % Update Models - For each tracker if changed update.

        end
    end

    %% Needs to Be Modified
    % Taken from ATOC Class Will have to Update for this class.
    methods(Access = private)
        % Main Method to Analyze Flights
        function [lane_id, uas_id, res_id, telemetryInfo, sensory, ...
                del_dis, del_speed, projection, rogue] = ...
                AnalyzeFlight(obj, res, dist, idx, uniIdx, datapts)
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
            [rows, ~] = find(idx == uniIdx);
            cluster = datapts(rows, :);

            tel_pos = zeros(length(obj.telemetry) - 1, 3);
            for index = 1:size(tel_pos,1)
                tel_pos(index, :) = obj.telemetry(index + 1).pos;
            end
            % Check which UAS belongs to this group
            [uas_index, ~] = find(ismember(tel_pos,cluster, ...
                'rows') == 1);
            [sen_index, ~] = find(ismember(cluster,tel_pos,...
                'rows') == 1);

            % Grab the associated Sensory Informaiton
            sensory = cluster;
            sensory(sen_index, :) = [];

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
                uas_index = uas_index(end);
                telemetryInfo = struct(obj.telemetry(uas_index +1));
                uas_id = obj.telemetry(uas_index + 1).ID;
                [rows, ~] = find(res.uas_id == uas_id & ...
                    res.entry_time_s <= obj.time & ...
                    res.exit_time_s >= obj.time);
                if(~isempty(rows))
                    res = res(rows(end), :);
                end
            end

            % There was no reservation data
            if(size(res, 1) ~= 1)
                % Check rogue_uas list - if already in the list. If not add
                % the rogue_uas to the list, give id,project to lane,
                % uas_id
                rogue = 1;
                
                lane_id = obj.findClosestLane(obj.telemetry(uas_index).pos);
            else
                lane_id = res.lane_id;
                res_id = res.id;
                headway = res.hd;
            end

            % Find the previous material
            cols = [obj.masterList.uas_id];
            pre_info = [];
            if(~isempty(cols))
                [rows, ~] = find(cols == uas_id);
                pre_info = obj.masterList(rows(end), :);
            end

            % Perform Analysis
            [del_dis, del_speed, projection, aRogue] = ...
                obj.PerformAnalysis(tel_pos(uas_index(end), :), ...
                pre_info, res, lane_id);

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
        %   aRogue (boolean): Whether there was adnormal behavior 
        %
            del_dis = 0;
            del_speed = 0;
            aRogue = 0;
            ids = obj.lbsd.getLaneVertexes(lane_id);
            lanes = obj.lbsd.getVertPositions(ids(1, :));
            
            % Grab Reservation information
            if(~isempty(res))
                % Distance Difference
                dir_v = lanes(2, :) - lanes(1, :);
                del_t = obj.time - res.entry_time_s;
                planned_pos = lanes(1, :) + del_t*dir_v;
                
                del_dis = norm(uas_pos - planned_pos);

                % Check if del_dis is over a certain point
                    % If so aRogue = 1

                % Speed Difference
                if(~isempty(pre_info))
                    prev_time = pre_info(end).time;
                    prev_uas_pos = pre_info(end).telemetry;
                    if(isempty(prev_uas_pos))
                        prev_uas_pos = zeros(1,3);
                    else
                        prev_uas_pos = prev_uas_pos.pos;
                    end

                    if(isempty(prev_time))
                        prev_time = 0;
                    end
                    del_t = prev_time - res.entry_time_s;
                    dir_v = lanes(2, :) - lanes(1, :);
                    prev_pos = lanes(1, :) + del_t*dir_v;
                    del_dis = norm(prev_pos - planned_pos);
                    schedule_speed = del_dis/(obj.time - prev_time);
                    del_t = obj.time - prev_time;
                    d_dis = norm(uas_pos - prev_uas_pos);
                    del_speed = (d_dis/del_t) - schedule_speed;
                end

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
            
        end
    end

    %% Rogue Detection
    % This section deals with identify any Rogue Behaviors of UAS' during
    % the life of the simulation, and then notify when certain behaviors
    % are observed
    %
    methods
        function RemovedFromATOC()
            % Check to see if there is any flights happening
           if(~isempty(obj.telemetry(end).ID) || ~isempty(obj.radars(end).ID))
               res = obj.lbsd.getReservations();
               datapts = zeros((length(obj.telemetry)+ ...
                   length(obj.radars) - 2), 3);
               counter = 1;
               for i = 2:length(obj.telemetry)
                   if (~isempty(obj.telemetry(i).pos))
                       datapts(counter, :) = obj.telemetry(i).pos;
                       counter = counter + 1;
                   end
               end
               for i = 2:length(obj.radars)
                   datapts(counter, :) = obj.radars(i).pos;
                   counter = counter + 1;
               end
               % Grab the UAS from the group.
               [idx, corepts] = dbscan(datapts, 3, 1);
               % find the dist_matrix
               [rows, ~] = find(corepts == 1);
               points = datapts(rows, :);
               D = pdist(points);
               dist = squareform(D);
               uni_idx = unique(idx);
               % Add Unigroups to MasterList
               for index = 1:length(uni_idx)
                   [lane_id, uas_id, res_id, telemetryInfo, sensory, ...
                       del_dis, del_speed, projection, rogue] = ...
                       obj.AnalyzeFlight(res, dist, idx, uni_idx(index), ...
                       datapts);
                   obj.AddEntry(lane_id,uas_id, res_id, telemetryInfo, ...
                       sensory, del_dis, del_speed, projection, rogue);
               end
           end

        end
    end
end
