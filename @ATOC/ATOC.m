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
        telindex % Keep track of the row entry in telemetry
        senIndex % Keep track of the row entry in sensory
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
            masterList = struct('time', [], 'lane_id', [], ...
                'uas_id', [], 'res_id', [], 'telemetry', [], 'sensory', [], ...
                'del_dis', [], 'del_speed', [], 'proj', [], 'tracker_id', [], ...
                'Classification', []);
            
            obj.masterList = repmat(masterList, 100, 1);

            % Initialize Track Monitor object
            obj.trackMen = TrackMonitor();
            obj.trackMen.initializeLaneStructor(lbsd);

            % Create tempory telemetry & radar data structures
            info = struct('ID', [], 'pos', [], 'speed', [], 'time', []);
            obj.telemetry = repmat(info, 100, 1);
            obj.radars = repmat(info, 100,1);
            obj.telindex = 1;
            obj.senIndex = 1;

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

            if event.EventName == "Tick"
                % Grab Reserveration Data for the past simulation step
                res = obj.lbsd.getReservations();
                [rows, ~] = find(res.entry_time_s <= obj.time & ...
                    res.exit_time_s >= obj.time);
                res = res(rows, :);

                % Analyze flight behavior
                if(obj.telindex == 1)
                    obj.telindex = 2;
                end
                
                obj.trackMen.AnalyzeFlights(obj.telemetry(1:obj.telindex-1),...
                    obj.radars(1:obj.senIndex), res, src.tick_del_t);
                
                % Retrieve Flight Behavior
                flightInformation = ...
                    obj.trackMen.retrieveFlightInformation();

                obj.UpdateMasterList(flightInformation);

                % Update atoc time
                obj.time = obj.time + src.tick_del_t;

                % Clear previous information
                obj.telindex = 1;
                obj.senIndex = 1;

            end
            if event.EventName == "NewReservation"
                % This event is used to handle any new reservations that take
                % place during a simulation.
                obj.lbsd = src;
            end
            if event.EventName == "telemetry"
                % Handling all of the reported telemetry information
                obj.telemetry(obj.telindex) = struct('ID', src.id, ...
                    'pos',[src.gps.lon, src.gps.lat, src.gps.alt], ...
                    'speed', [src.gps.vx, src.gps.vy, src.gps.vz], ...
                    'time', obj.time);
                obj.telindex = obj.telindex + 1;
            end
            if event.EventName == "Detection"
                % Handling Radar Detection
                for item = 1:size(src.targets, 2)
                    obj.radars(obj.senIndex) = struct('ID', src.ID, ...
                        'pos', [src.targets(item).x,src.targets(item).y,...
                        src.targets(item).z], 'speed', [src.targets(item).s], ...
                        'time', obj.time); 
                    obj.senIndex = obj.senIndex + 1;
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

    % Update Data Structures
    methods (Access = private)
        % Driver Method to Update the MasterList and Analyze Flights
        function UpdateMasterList(obj, flightInformation)
            % UpdateMasterList - the driver method to perform all of the
            % analysis for each flight behavior within a given simulation
            % step.
            % Input:
            %   obj (atoc handle)
            % Output:
            %

          % Add information to MasterList
            for behavior = 1:size(flightInformation, 1)
                obj.AddEntry(flightInformation(behavior));
            end
        end
        function AddEntry(obj, oneFlight)
            % AddEntry - This is a helper function that adds an entry into the
            % Master List
            % input:
            %   oneFlight (struct) - contains all of the information for a
            %   particular flight behavior during a simulation step
            %

            % Update the master list
            if(oneFlight.uas_id == "")
                return;
            end
            
            row = [];
            if(~isempty(obj.masterList(1).tracker_id))
                [row, ~] = find([obj.masterList.tracker_id]' == oneFlight.tracker_id);
            end
            if(isempty(row))
                obj.masterList(obj.indexer) = struct('time', obj.time, ...
                    'lane_id', convertCharsToStrings(oneFlight.lane_id), ...
                    'uas_id', oneFlight.uas_id,...
                    'res_id', convertCharsToStrings(oneFlight.res_id), ...
                    'telemetry', oneFlight.telemetry, ...
                    'sensory', oneFlight.sensory, ...
                    'del_dis', oneFlight.del_dis,...
                    'del_speed', oneFlight.del_speed, ...
                    'proj', oneFlight.proj,...
                    'tracker_id', oneFlight.tracker_id, ...
                    'Classification', oneFlight.Classification);
                obj.indexer = obj.indexer + 1;
            else
                obj.updateEntry(oneFlight, row);
            end
        end
        function updateEntry(obj, flightInformation, index)
            % updateEntry: updates the flight information of an already
            % recorded flight 
            if(size(index, 1) > 1)
                index = index(end);
            end
            obj.masterList(index).time = [obj.masterList(index).time; ...
                obj.time];
            if(obj.masterList(index).lane_id(end) ~= flightInformation.lane_id)
                obj.masterList(index).lane_id = [obj.masterList(index).lane_id; ...
                    convertCharsToStrings(flightInformation.lane_id)];
            end
            if(obj.masterList(index).res_id(end) ~= flightInformation.res_id)
                obj.masterList(index).res_id = [obj.masterList(index).res_id,...
                    convertCharsToStrings(flightInformation.res_id)];
            end
            obj.masterList(index).sensory = [obj.masterList(index).sensory, ...
                flightInformation.sensory];
            obj.masterList(index).telemetry = [obj.masterList(index).telemetry, ...
                flightInformation.telemetry];
            obj.masterList(index).del_dis = [obj.masterList(index).del_dis, ...
                flightInformation.del_dis];
            obj.masterList(index).del_speed = [obj.masterList(index).del_speed, ...
                flightInformation.del_speed];
            obj.masterList(index).proj = [obj.masterList(index).proj, ...
                flightInformation.proj];
            obj.masterList(index).Classification = ...
                flightInformation.Classification;
        end
    end
end
