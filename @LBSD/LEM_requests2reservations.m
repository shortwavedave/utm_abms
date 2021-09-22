function [reservations,flights] = LEM_requests2reservations(obj,airways,...
    requests,hd)
% LEM_requests2resvations - reserve lanes for flights, if possible
% On input:
%     airways (airways struct): airways info
%     requests (requests struct): requests info
%     hd (float): headway distance
% On output:
%     reservations (reservations struct): reservations info
%       (k).flights (mx4 array): entry_time, exit_time, speed, lane_index
%     flights (flights struct): flights info
%       (k).type (int): 1 success; 0 failed
%       (k).decon_time (float): time to deconflict (sec)
%       (k).plan (px4 array): entry_time, exit_time, speed, lane_index
%       (k).route (qx9 array): x1,y1,z1,x2,y2,z2,t1,t2,speed
%       (k).telemetry (sx7 array): x,y,z,dx,dy,dz,speed
% Call:
%     [rr,ff] = LEM_requests2reservations(airways,requests,1);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

FAILED = 0;

num_requests = length(requests);
num_lanes = length(airways.lane_lengths);
for k = 1:num_lanes
    reservations(k).flights = [];
    reservations(k).hd = hd;
end
flights = [];

wb = waitbar(0,'LBSD Deconfliction');
for k = 1:num_requests
    waitbar(k/num_requests);
    [k,num_requests]
    [reservations,f1] = LBSD.LEM_gen_reservations(obj,airways,flights,...
        reservations,requests(k),1,hd);
    indexes = LBSD.LEM_find_conflict(obj,reservations,0.1);
    if ~isempty(indexes)
        display('Conflict in reservations');
    end
    flights(k).decon_time = f1.decon_time;
    flights(k).plan = f1.plan;
    flights(k).route = f1.route;
    flights(k).telemetry = f1.telemetry;
    flights(k).start_interval = requests(k).launch_interval;
    flights(k).speed = requests(k).speed;
    flights(k).path = requests(k).path;
    flights(k).path_vertexes = requests(k).path_vertexes;
    flights(k).launch_index = requests(k).launch_index;
    flights(k).land_index = requests(k).land_index;
    if isempty(f1.plan)
        flights(k).start_time = Inf;
        flights(k).end_time = Inf;
    else
        flights(k).start_time = f1.plan(1,1);
        flights(k).end_time = f1.plan(end,2);
    end
    if f1.type~=FAILED
        flights(k).type = f1.type;
    else
        flights(k).type = FAILED;
    end
end
close(wb);

tch = 0;
