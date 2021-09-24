function res = LEM_sim1_LBSD_51x51(obj,num_flights,airways,t_min,t_max,...
    launch_time_spread,b)
% LEM_sim1_LBSD_41x51 - 51x51 grid simulation of lane based method
% On input:
%     num_flights (int): number of flights to schedule
%     airways (airways struct): airways info
%     t_min (float): simulation start time
%     t_max (float): simulation end time
%     launch_time_spread (float); possible launch interval spread
%     b (Boolean): if 1, then create movie (sim_LBSD_6x6.mp4); else no
% On output:
%     res (results struct): results info
%       .roads (roads struct): roads info
%       .airways (airways struct): airways info
%       .requests (requests struct): LBSD requests
%       .reservations (reservations struct): reservations info
%       .flights (flights struct): flights info
%       .failures_LBSD (n by 1 vector): 0 if LBSD flight success; else 1
%       .flights_cells (flights struct): cells flight info
%       .cells (cells struct): FAA-NASA cells corresponding to LBSD layout
%       .requests_cells (requests_cells struct): FAA-NASA flight requests 
%       .P (nx5 array): performance info for LBSD
%         col 1 (float): flight ID
%         col 2 (float): delay (actual_start - requested_min_start)
%         col 3 (float): flight duration
%         col 4 (float): deconflict time
%         col 5 (float): failed to schedule
%        .P_cells (empty)
% Call:
%     res = LEM_sim1_LBSD_51x51(500,0,100,10,0);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

FAILED = 0;

res = [];

hd = 1;   % headway distance
min_speed = 10;
max_speed = 10;
%num_flights = 100;
requests = LBSD.LEM_gen_requests_LBSD(obj,t_min,t_max,airways,num_flights,...
    launch_time_spread,[min_speed,max_speed]);
[reservations,flights] = LBSD.LEM_requests2reservations(obj,airways,...
    requests,hd);

del_t = 0.1;
for f = 1:num_flights
    f_out = LBSD.LEM_gen_traj(obj,flights(f),del_t);
    flights(f).traj = f_out.traj;
end

if b==1
    LBSD.LEM_run_flights(obj,airways,flights,1,1,'sim1_LBSD');
end

P = LBSD.LEM_performance(obj,flights);

roads.vertexes = airways.vertexes;
roads.edges = airways.edges;
res.roads = roads;
res.airways = airways;
res.requests = requests;
res.reservations = reservations;
res.flights = flights;
failures_LBSD = zeros(num_flights,1);
for f = 1:num_flights
    if flights(f).type==FAILED
        failures_LBSD(f) = 1;
    end
end
res.failures_LBSD = failures_LBSD;
res.mp4 = 'sim1_LBSD_51x51.mp4';
res.P = P;
res.P_cells = [];
