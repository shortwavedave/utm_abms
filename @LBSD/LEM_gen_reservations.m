function [reservations_out,flights] = LEM_gen_reservations(obj,airways,...
    a_flights,reservations,request,n,hd)
% LEM_gen_reservations - generate new lane reservations
% On input:
%     airways (airways struct): airways info
%     a_flights (flight struct): flights info
%     reservations (reservations struct): lane reservations
%     request (request struct): request info
%       .request_time (float): time request arrives
%       .launch_interval (1x2 vector): [min start, max start]
%       .speed (float): desired speed
%       .path (1xn vector): lane sequence
%     t (float): current time
%     n (int): number of new flights
%     hd (float): headway distance
% On output:
%     reservations_out (reservations struct): updated lane reservations
%     flights (flights struct): new flight plans
% Call:
%     [r1,f1] = LEM_gen_reservations(airways,reservations,12.4,3);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

FAILED = 0;
PLANNED = 1;

reservations_out = reservations;
flights = [];

ht = hd/request.speed;
path = request.path;

num_flights = length(a_flights);
num_flight_plans = 0;
for k = 1:n
    v1 = request.path(1);
    v2 = request.path(end);
    t1 = request.launch_interval(1);
    t2 = request.launch_interval(2);
    speed = request.speed;
    tic
    [fp,reservations_out] = LBSD.LEM_reserve_fp(obj,reservations_out,...
        airways,t1,t2,speed,path,hd);
    dct = toc;
    if ~isempty(fp)
        num_flight_plans = num_flight_plans + 1;
        num_flights = num_flights + 1;
        num_steps = length(fp(:,4));
        for s = 1:num_steps
            index = find(reservations_out(fp(s,4)).flights(:,1)==-1);
            reservations_out(fp(s,4)).flights(index,1) = num_flights;
%            reservations_out(fp(s,4)).flights(end,1) = num_flights;
        end
        flights(num_flight_plans).type = PLANNED;
        flights(num_flight_plans).decon_time = dct;
        flights(num_flight_plans).plan = fp;
        flights(num_flight_plans).route = LBSD.LEM_plan2route(obj,fp,airways);
        lane_index = fp(1,4);
        lane_vertex1 = airways.lane_edges(lane_index,1);
        pt1 = airways.lane_vertexes(lane_vertex1,:);
        lane_vertex2 = airways.lane_edges(lane_index,2);
        pt2 = airways.lane_vertexes(lane_vertex2,:);
        dir1 = pt2 - pt1;
        dir1 = dir1/norm(dir1);
        flights(num_flight_plans).telemetry = [pt1 dir1 fp(1,3)];
    else
        num_flight_plans = num_flight_plans + 1;
        flights(num_flight_plans).type = FAILED;
        flights(num_flight_plans).decon_time = dct;
        flights(num_flight_plans).plan = [];
        flights(num_flight_plans).route = [];
        flights(num_flight_plans).telemetry = [];
    end
end
tch = 0;
