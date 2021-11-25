classdef SimMetrics < matlab.mixin.SetGet
    %SIMMETRICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % The time it took to initialize the LBSD
        init_time_lbsd_s = -1
        % The time it took to initialize the ATOC
        init_time_atoc_s = -1
        % The time it took to intialize the Radar system
        init_time_radar_s = -1
        % The time it took to initialize the UAS
        init_time_uas_s = -1
        % The time it took to initialize the UAS Trajectories
        init_time_traj_s = -1
        % The number of flights that could not be scheduled
        num_failed_flights = 0
        % Failed Flight IDs
        failed_flights_ids = []
        % The number of flights that were scheduled
        num_success_flights = 0
        % IDS of flights that were scheduled successfully
        success_flights_ids = []
        % Reservation time statistics .var .mean .median .max .min
        reservation_time
        % Delay statistics .var .mean .median .max .min
        delay_time
        % (struct array) .lane_id .num_uas .occ representing lane occupancy
        % The ratio of of the sum of the lengths of the headway times of
        % reservations to the length of the interval in which those 
        % reservations are present 
        lane_occs = []
        % Handle to the sim object
        h_sim
        test_config
    end
    
    methods
        function obj = SimMetrics()
            %SIMMETRICS Construct an instance of this class
            %   Detailed explanation goes here
            obj.reservation_time.max = -1;
            obj.reservation_time.min = -1;
            obj.reservation_time.mean = -1;
            obj.reservation_time.median = -1;
            obj.reservation_time.var = -1;
            
            obj.delay_time.max = -1;
            obj.delay_time.min = -1;
            obj.delay_time.mean = -1;
            obj.delay_time.median = -1;
            obj.delay_time.var = -1;
        end
    end
    
    methods (Static)
        [occ_var, occ] = occ_variance(metrics)
    end
end

