classdef LBSD < handle
    %LBSD An instance of this class represents a Lane Based Strategic
    %   Deconfliction Supplemental Data Service Provider (SDSP). The user
    %   of this class is expected to create the lane_graph (which is a
    %   public property of this object), a directed graph as defined by 
    %   matlab (see https://www.mathworks.com/help/matlab/ref/digraph.html)
    %
    %   When constructing new lane graphs for this object, it is important
    %   to make the vertexes and edges indexed by a string, rather than just
    %   an integer. This makes the lane structure more flexible, allowing
    %   lanes to be added and deleted dynamically. The indexes can be set
    %   to strings by building a table with 'RowNames' specified.
    %   (see
    %   https://www.mathworks.com/help/matlab/matlab_prog/access-data-in-a-table.html).
    %   Additionally, the lane_graph must have XData, YData, ZData, as well
    %   as boolean Land and Launch columns. See the lane_graph generated by
    %   LBSD.genSampleLanes for an example.
    %   
    %   When the lane_graph property is set, this class
    %   automatically calculates delauney triangulations of land and launch
    %   nodes to facilitate nearest neigbor searching.
    %
    %   A sample graph can be generated by calling the static function 
    %   genSampleLanes. For example:
    %       
    %       lbsd = LBSD.genSampleLanes(10, 15)
    %
    %   For documentation of this function run:
    %
    %       help LBSD.genSampleLanes
    %
    %   For general documentation of this class, run:
    %
    %       help LBSD
    %
    %   Author:
    %       D. Sacharny
    %       UU
    %       Summer 2021
    %
    
    properties
        % A digraph representing the lane network. 
        lane_graph
    end
    
    properties (Access = protected)
        % A table of reservations. This table is preallocated for 
        % for performance reasons. To get the total number of reservations, 
        % use the getNumReservations method. This table contains the 
        % following columns:
        %   id %(string): unique identifier for this reservation
        %   lane_id %(string): The lane that is being reserved
        %   entry_time_s %(float): entry time from the lane in seconds 
        %   exit_time_s %(float): exit time from the lane in seconds 
        %   speed %(float): speed in m/s
        %   hd %(float): required headway distance 
        reservations
        % The next id of a reservation
        next_res_id = 1
        % The number of rows to preallocate in the reservations table
        preallocate = 100000
        % The next row to write to in the reservations table
        next_tbl_row = 1
        % A Warning indicator if we have exhausted all preallocated
        % reservations rows.
        notified_preallocation_warning = false
        % The row of latest reservation that was recorded in the
        % reservations table
        latest_res_row
        % Array of subscribers to the NewReservation event
        new_res_listeners = []
        % The delauney triangulation of launch vertexes. This is used in
        % order to perform a nearest neighbor search
        launch_delauney_tri
        % The delauney triangulation of land vertexes. This is used in
        % order to perform a nearest neighbor search
        land_delauney_tri
        % Table of launch nodes
        launch_table
        % Table of land nodes
        land_table
    end
    
    events
        % This event is triggered when a new reservation is recorded, i.e.,
        % when a successful call to makeReservation is made. The event
        % handler can obtain the reservation that triggered this request by
        % calling getLatestRes.
        NewReservation
    end
    
    methods
        function obj = LBSD()
            %LBSD Construct an instance of this class
            obj.clearReservations();
        end

        %% Reservation Methods
        function subscribeToNewReservation(obj, subscriber)
            % subscribeToNewReservation Set an event listener to trigger  
            %   when a new reservation is made.
            % On input
            %   obj - an instance of the LBSD class
            %   subscriber - a function handle to trigger
            % Call:
            %   lbsd.subscribeToNewReservation(@atoc.handleNewRes);
            lh = obj.addlistener('NewReservation', subscriber);
            obj.new_res_listeners = [obj.new_res_listeners, lh];
        end
        
        function clearNewResSubscriptions(obj)
            % clearNewResSubscriptions Delete all subscriptions to the
            % NewReservation event
            for lh = obj.new_res_listeners
                delete(lh);
            end
        end
        
        function res = getLatestRes(obj)
            % getLatestRes Get the latest reservation that was made
            % On Output:
            %   res: single row table, or empty table if no reservations
            %   have been made.
            if isempty(obj.latest_res_row)
                res = obj.reservations(obj.reservations.id == -1);
            else
                res = obj.reservations(obj.latest_res_row,:);
            end
        end
        
        function num_res = getNumReservations(obj)
            % getNumReservations Get the reservations for a lane
            % On Input:
            %   lane_id: (string) the lane id
            % On Output:
            %   lane_res: a table containing reservations
            num_res = obj.next_tbl_row - 1;
        end
        
        function res = getReservations(obj)
            % getReservations Get all reservations
            % On Output:
            %   res: a table containing reservations
            res = obj.reservations(1:obj.next_tbl_row - 1,:);
        end
        
        function lane_id = getLaneIdFromResId(obj, res_id)
            % getLaneIdFromResId Get the lane_id of a reservation
            % On Input:
            %   res_id: (string) the id of the reservation
            % On Output:
            %   lane_id: (string or an empty table) if the reservation id
            %       is present, then the lane_id is returned, otherwise it
            %       returns an empty table (this can be tested via:
            %           isempty(lane_id).
            % Call:
            %   lane_id = lbsd.getLaneIdFromResId("1")
            %   res_found = ~isempty(lane_id)
            lane_id = obj.reservations{...
                find(obj.reservations.id == res_id,1),'lane_id'};
        end
        
        function clearReservations(obj)
            % clearReservations Clear all reservations
            obj.reservations = table( 'Size',[obj.preallocate 6], ...
                'VariableNames', {'id','lane_id', ...
                'entry_time_s', 'exit_time_s', 'speed', 'hd'}, ...
                'VariableTypes',{'string','string','double','double', ...
                'double', 'double'} );
            obj.next_res_id = 1;
            obj.next_tbl_row = 1;
        end
        
        function [ok, res_ids, res_toa_s] = ...
                reserveLBSDTrajectory(obj, lane_ids, toa_s, h_d, r_e, r_l)
            % reserveLBSDTrajectory Reseave a sequence of lanes
            %	This method takes lane ids, time-of-arrival and departure
            %	for each lane, required headway distance, earliest launch
            %	time, and latest launch time, and returns an array of
            %	reservation ids and reserved time-of-arrival and
            %	departures.
            %   On Input:
            %       lane_ids - [nx1] string of lane identifiers
            %       toa_s - [(n+1)x1] float seconds arrival at each vertex
            %       h_d - float headway distance in meters
            %       r_e - float seconds earliest release (launch) time 
            %           desired
            %       r_l - float seconds latest release (launch) time
            %           desired
            %   On Output:
            %       res_ids - [nx1] string reservation ids for each lane
            %       res_toa_s - [(nx1)x1] float seconds reserved arrival at
            %           at each vertex in the reserved lane sequence
            ok = false;
            res_ids = [];
            res_toa_s = [];
            % Get all the reservations for each lane within the desired
            % intervals
            lane_dists = obj.getLaneLengths(lane_ids);
            % The average speed in the lane is the lane distance
            % divided by the time it takes to cross the lane
            lane_speeds = lane_dists ./ (toa_s(2:end)-toa_s(1:end-1));
            % Calculate the equivalent headway times
            hts = hd / lane_speeds;
            for i = 1:length(lane_ids)
                lane_id = lane_ids(i);
                ht = hts(i);
                % Buffer the release and exit times for the purpose of
                % considering relevant reservations that may conflict with
                % this proposed trajectory
                l_r_e = r_e + toa(i) - ht;
                l_r_l = r_l + toa(i) + ht;
                l_e_e = r_e + toa(i+1) - ht;
                l_e_l = r_l + toa(i+1) + ht;
                % Query the reservation table for all reservations that may
                % conflict
                lane_res = obj.getLaneResTimeBound(lane_id, l_r_e, l_r_l, ...
                    l_e_e, l_e_l);
                
            end
        end
        
        function [ok, res_id] = makeReservation(obj, lane_id, entry_time_s, ...
            exit_time_s, speed, hd)
            %makeReservation Create a reservation
            %   This method checks that the lane_id is valid and that the
            %   requested reservation does not overlap entry or exit time 
            %   within headway distance.
            %   On Input:
            %       lane_id %(string): The lane that is being reserved
            %       entry_time_s %(float): entry time from the lane in seconds 
            %       exit_time_s %(float): exit time from the lane in seconds 
            %       speed %(float): speed in m/s
            %       hd %(float): required headway distance 
            %   On Output:
            %       ok: true if the reservation was made successfully
            %       res_id: (string) reservation id. empty string if
            %       ok is false.
            %   Call:
            %       res = lbsd.makeReservation("1",0,10,10,10)
            %
            
            % First check that the lane_id exists
            if ~find(obj.lane_graph.Edges.Properties.RowNames == lane_id)
                ok = false;
                res_id = "";
            else
                % Create the candidate reservation row
                res = {string(obj.next_res_id), string(lane_id), ...
                        entry_time_s, exit_time_s, speed, hd};
                % Grab the reservations in this lane
                lane_res = obj.getLaneReservations(lane_id);
                if isempty(lane_res)
                    % No reservations in table, so go ahead and create a
                    % new one.
                    new_row_ind = obj.appendReservation(res);
                    obj.latest_res_row = new_row_ind;
                    res_id = obj.next_res_id;
                    obj.next_res_id = obj.next_res_id + 1;
                    ok = true;
                    notify(obj, 'NewReservation');
                else
                    % Check that entry time and exit times do not overlap
                    % by headway.
                    entry_time_conflict = find(...
                        abs(lane_res.('entry_time_s')-entry_time_s) < hd, ...
                        1);
                    exit_time_conflict = find(...
                        abs(lane_res.('exit_time_s')-exit_time_s) < hd, ...
                        1);
                    if isempty(entry_time_conflict) && ...
                            isempty(exit_time_conflict)
                        new_row_ind = obj.appendReservation(res);
                        obj.latest_res_row = new_row_ind;
                        res_id = obj.next_res_id;
                        obj.next_res_id = obj.next_res_id + 1;
                        ok = true;
                        notify(obj, 'NewReservation');
                    else
                        ok = false;
                        res_id = "";
                    end
                end
            end
        end
        
        function lane_res = getLaneReservations(obj, lane_id)
            % getLaneReservations Get the reservations for a lane
            % On Input:
            %   lane_id: (string) the lane id
            % On Output:
            %   lane_res: a table containing reservations
            lane_res = ...
                obj.reservations(obj.reservations.lane_id == lane_id,:);
        end
        
        function lane_res = getLaneResTimeBound(obj, lane_id, r_e, r_l, ...
                e_e, e_l)
            % getLaneResTimeBound Get the reservations for a lane between
            % two times. This amethod will return all lane reservations for
            % a lane_id that are in the time range ([r_e,r_l] | [e_e,e_l])
            % On Input:
            %   lane_id: (string) the lane id
            %   r_e: (float) earliest release time in seconds to consider
            %   r_l: (float) latest release time in seconds to consider
            %   e_e: (float) earliest exit time in seconds to consider
            %   e_l: (float) latest exit time in seconds to consider
            % On Output:
            %   lane_res: a table containing reservations
            c1 = obj.reservations.lane_id == lane_id;
            c2 = obj.reservations.entry_time_s >= r_e & ...
                    obj.reservations.entry_time_s <= r_l;
            c3 = obj.reservations.exit_time_s >= e_e & ...
                    obj.reservations.exit_time_s <= e_l;
            c = c1 & (c2 | c3);
            lane_res = obj.reservations(c, :);
        end
        
        
        
        function genRandReservations(obj, start_time, end_time, num_res, ...
                lane_ids, speed, headway)
            % genRandReservations create some random reservations
            % On Input:
            %   start_time: (float) the earliest reservation in seconds
            %   end_time: (float) the latest reservation in seconds
            %   num_res: (integer) the total number of reservations to try.
            %       Note: The final number of successful reservations may be
            %       less due to headway constraints.
            %   lane_ids: nx1 string array of lane ids to schedule on
            %   speed: (float) m/s
            %   headway: (float) m
            hd = headway;
            lengths = obj.getLaneLengths(lane_ids);
            entry_times = start_time + ...
                (end_time-start_time)*rand(1, num_res);
            for lane = lane_ids
                for res = 1:num_res
                    l = lengths(lane_ids==lane);
                    exit_time = entry_times(res)+l/speed;
                    [ok, ~] = obj.makeReservation(lane, entry_times(res), ...
                        exit_time, speed, hd);
                end
            end
        end
        
        %% Lane Methods
        function set.lane_graph(obj, g)
            % set.lane_graph Set the lane_graph property
            % 	In addition to checking all the required columns are there,
            % 	this method generates delauney triagulations for land and
            % 	launch nodes.
            % On Input:
            %   g - the lane graph - a directed graph
            has_x = any(startsWith(strtrim(...
                g.Nodes.Properties.VariableNames), 'XData'));
            has_y = any(startsWith(strtrim(...
                g.Nodes.Properties.VariableNames), 'YData'));
            has_z = any(startsWith(strtrim(...
                g.Nodes.Properties.VariableNames), 'ZData'));
            has_land = any(startsWith(strtrim(...
                g.Nodes.Properties.VariableNames), 'Land'));
            has_launch = any(startsWith(strtrim(...
                g.Nodes.Properties.VariableNames), 'Launch'));
            
            if ~has_x
                error('lane_graph must have XData column');
            end
            if ~has_y
                error('lane_graph must have YData column');
            end
            if ~has_z
                error('lane_graph must have ZData column');
            end
            if ~has_land
                error('lane_graph must have Land column');
            end
            if ~has_launch
                error('lane_graph must have Launch column');
            end
            
            obj.lane_graph = g;
            obj.recalcInternalStructs();
        end
        
        function h = plot(obj)
            %plot Plot the Lane System
            xdata = obj.lane_graph.Nodes.XData;
            ydata = obj.lane_graph.Nodes.YData;
            zdata = obj.lane_graph.Nodes.ZData;
            h = plot(obj.lane_graph,'XData',xdata,'YData',ydata, ...
                'ZData',zdata);
        end
        
        function highlight(obj, h, lane_ids, varargin)
            % highlight Highlight lanes in plotted graph
            % On Input:
            %   h - plot handle returned from LBSD.plot
            %   lane_ids - (nx1 string array) lane_ids
            %   varargin - (optional) variable arguments passed directly to
            %       matlab's highlight method
            %   (see
            %   https://www.mathworks.com/help/matlab/ref/...
            %   matlab.graphics.chart.primitive.graphplot.highlight.html)
            
            inds = find(ismember(...
                obj.lane_graph.Edges.Properties.RowNames, lane_ids));
            highlight(h,'Edges',inds,varargin{:});
        end
        
        function ids = getClosestLaunchVerts(obj, q)
            % getClosestLaunchVerts Get the Vertex IDs of the closest launch
            % nodes. 
            % On Input:
            %   q - nx2 query points. Each row is [x,y]
            % On Output:
            %   ids - (nx1 string array) ids of the closest launch vertex
            % Call:
            %   ids = lbsd.getClosestLaunchVerts([0 0; 5 5; -30 -15])
            xi = nearestNeighbor(obj.launch_delauney_tri, q);
            ids = string(obj.launch_table.Properties.RowNames(xi));
        end
        
        function ids = getClosestLandVerts(obj, q)
            % getClosestLandVerts Get the Vertex IDs of the closest land
            % nodes. 
            % On Input:
            %   q - nx2 query points. Each row is [x,y]
            % On Output:
            %   ids - (nx1 string array) ids of the closest launch vertex
            % Call:
            %   ids = lbsd.getClosestLandVerts([0 0; 5 5; -30 -15])
            xi = nearestNeighbor(obj.land_delauney_tri, q);
            ids = string(obj.land_table.Properties.RowNames(xi));
        end
        
        function [lane_ids, vert_ids, dist] = getShortestPath(obj, ...
                start_vert_id, end_vert_id)
            % getShortestPath Get a shortest path between two vertexes 
            % On Input:
            %   start_vert_id - (string) the id of the start vertex
            %   end_vert_id - (string) the id of the end vertex
            % On Output:
            %   lane_ids - (nx1 string array) ids of lanes from start to
            %       end.
            %   vert_ids - (nx1 string array) ids of the vertexes from
            %       start to end
            %   distance - the distance covered by the path
            % Call:
            %   [lane_ids, vert_ids, dist] = lbsd.getShortestPath("12","22")
            [vert_ids,dist,lane_ids] = shortestpath(obj.lane_graph, ...
                start_vert_id, end_vert_id);
            lane_ids = string(obj.lane_graph.Edges.Properties.RowNames(...
                lane_ids));
            vert_ids = vert_ids';
        end
        
        function ids = getLaunchVerts(obj)
            % getLaunchVerts Get the Vertex IDs of the launch nodes
            % On Output:
            %   ids - indices of vertexes
            ids = string(obj.launch_table.Properties.RowNames);
        end
        
        function ids = getLandVerts(obj)
            % getLandVerts Get the Vertex IDs of the land nodes
            % On Output:
            %   ids - string array
            ids = string(obj.land_table.Properties.RowNames);
        end
        
        function positions = getVertPositions(obj, ids)
            % getVertPositions Get the Positions of Vertexes in the lane 
            %   system by ID
            % On Input:
            %   ids - nx1 string array or ':' for all
            % On Output:
            %   positions - nx3 positions of vertexes
            positions = obj.lane_graph.Nodes{ids,{'XData','YData','ZData'}};
        end
        
        function ids = getLaneVertexes(obj, lane_id)
            % getLaneVertexes Get the vertex ids that define the lane
            % endpoints.
            % On Input:
            %   lane_id - (string) the id of the lane
            % On Output:
            %   ids - 2x1 string array of endpoint vertex ids
            rows = obj.lane_graph.Edges{lane_id,{'EndNodes'}}';
            ids = string(obj.lane_graph.Edges(rows,:).Properties.RowNames);
        end
        
        function lane_ids = getLaneIds(obj)
            % getLaneIds Get the vertex ids that define the lane
            % endpoints.
            % On Output:
            %   lane_ids - nx1 vector of strings of lane ids
            lane_ids = obj.lane_graph.Edges.Properties.RowNames;
            lane_ids = string(lane_ids);
        end
        
        function lengths = getLaneLengths(obj, lane_ids)
            % getLaneLengths Get the langths of lanes
            % On Input:
            %   lane_ids - nx1 (strings) the ids of the lanes
            % On Output:
            %   length - nx1 vector of floats of distances
            % Call:
            %   lengths = lbsd.getLaneLengths([1])
            num_ids = length(lane_ids);
            lengths = zeros(num_ids,1);
            for i = 1:num_ids
                lane_id = lane_ids(i);
                vert_ids = obj.lane_graph.Edges{ lane_id, {'EndNodes'} };
                vert_x = obj.getVertPositions(vert_ids);
                d = norm(vert_x(2,:) - vert_x(1,:));
                lengths(i) = d;
            end
        end
        
        function recalcInternalStructs(obj)
            % recalcInternalStructs Recalculate internal structures
            %   This function recalculates internal structures like land
            %   and launch tables and delauney triangulations. This
            %   method should be called anytime the lane_graph is modified.
            obj.recalcLaunchTable();
            obj.recalcLandTable();
            obj.recalcLaunchDelauney();
            obj.recalcLandDelauney();
        end
    end
    
    methods (Access = protected)
        function row_ind = appendReservation(obj, row)
            % appendReservation Append to the reservations table
            % On Input:
            %   row - cell array containing the reservation 
            % On Output:
            %   row_ind - the index of the new row in the reservations
            %   table.
            if obj.next_tbl_row < obj.preallocate
                obj.reservations(obj.next_tbl_row, :) = row;
                row_ind = obj.next_tbl_row;
                obj.next_tbl_row = obj.next_tbl_row + 1;
            else
                if ~obj.notified_preallocation_warning
                    warning("Exhausted Preallocated Reservations, Performance will now suffer");
                    obj.notified_preallocation_warning = true;
                end
                obj.reservations = [obj.reservations; row];
                row_ind = obj.next_tbl_row;
                obj.next_tbl_row = obj.next_tbl_row + 1;
            end
        end
        
                
        function recalcLaunchDelauney(obj)
            % recalcLaunchDelauney Recalculate the delauney triangulation
            % for launch nodes
            P = obj.launch_table{:, {'XData','YData'}};
            obj.launch_delauney_tri = delaunayTriangulation(P);
        end
        
        function recalcLandDelauney(obj)
            % recalcLandDelauney Recalculate the delauney triangulation
            % for land nodes
            P = obj.land_table{:, {'XData','YData'}};
            obj.land_delauney_tri = delaunayTriangulation(P);
        end
        
        function recalcLaunchTable(obj)
            % recalcLaunchTable Recalculate the launch node table
            obj.launch_table = obj.lane_graph.Nodes(...
                obj.lane_graph.Nodes.Launch==1,:);
        end
        
        function recalcLandTable(obj)
            % recalcLandTable Recalculate the land node table
            obj.land_table = obj.lane_graph.Nodes(...
                obj.lane_graph.Nodes.Land==1,:);
        end
        
    end
    
    methods (Static)
        lbsd = genSampleLanes(lane_length_m, altitude_m)
        
        function [H, f] = genReleaseObjective(rd)
            % genReleaseObjective Generate quadprog objective parameters
            % This is a quadratic objective that minimizes the time
            % distance between the desired release time and the constrained
            % solution.
            % On Input:
            %   rd - (float) desired release time
            % On Output:
            %   H - The H matrix for quadprog
            %   f - the f vector for quadprog
            % Call:
            %   [H, f] = genReleaseObjective(rd)
            H = [2 0; 0 0];
            f = [-2*rd 0]';
        end
        
        function [A, b] = genConflictConstraints(s, si, ri, ht, x0, xd)
            % genConflictConstraints Generate quadprog constraints for pair
            % conflict. It is assumed that the x vector is [x,r]', where x
            % is the position along a lane and r is the release time of the
            % desired reservation
            % On Input:
            %   s - (float) speed of the desired reservation
            %   si - (float) speed of the scheduled reservation
            %   ri - (float) release time of the scheduled reservation
            %   ht - (float) headway time required
            %   x0 - (float) start position (usually zero)
            %   xd - (float) end position (usually length of lane)
            % On Output:
            %   A - The A matrix for quadprog
            %   b - the b vector for quadprog
            % Call:
            %   [A, b] = LBSD.genConflictConstraints(s, si, ri, ht, x0, xd)
            A = [ -1/s  (s-si)/(s*si) ; ...
                   1/s  (si-s)/(s*si) ; ...
                   0    1;   ...
                   0   -1; ];
            b = [ -ht-(ri/si), -ht+(ri/si), xd, -x0 ]'; 
        end
    end
end