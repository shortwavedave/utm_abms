function requests = LEM_gen_requests_packed(obj,t_min,t_max,airways,...
    num_requests,del_t,speeds)
% LEM_gen_requests_packed - generate a packed set of flights
% On input:
%     t_min (float): earliest start time for flight
%     t_max (float): latest start time for flight
%     airways (airways struct): airways info
%     num_requests (int): number of requested flights
%     del_t (float): time interval between launches
%     speeds (1x2 vector): [min_speed, max_speed]
% On output:
%     requests (requests struct): request info
%      (k).request_time (float): time request is submitted
%      (k).launch_interval (1x2 vector): [earliest time, latest time]
%      (k).speed (float): desired average speed (uniform from speeds)
%      (k).lane_indexes (1xn vector): lane indexes of flight path
%      (k).launch_index (int): index in airways.launch_lane_vertexes
%      (k).land_index (int): index in airways.land_lane_vertexes
% Call:
%     req = LEM_gen_requests_LBSD(0,100,airways,1000,0.1,[20,30]);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

SPREAD = 1;

time_spread = t_max - t_min;
arrival_times = [t_min:del_t:t_max];
min_speed = min(speeds);
speed_spread = max(speeds) - min_speed;
times = zeros(num_requests,1);
launch_vertexes = airways.launch_lane_vertexes;
num_launch_vertexes = length(launch_vertexes);
land_vertexes = airways.land_lane_vertexes;
num_land_vertexes = length(land_vertexes);
arrival_time = -del_t;
for r = 1:num_requests
    arrival_time = arrival_time + del_t;
    requests(r).request_time = arrival_time;
    t1 = arrival_time;
    t2 = t1 + SPREAD;
    requests(r).launch_interval = [t1,t2];
    requests(r).speed = min_speed + rand*speed_spread;
    launch_index = num_launch_vertexes(1);
    launch_vertex = launch_vertexes(launch_index);
    land_index = num_land_vertexes(1);
    land_vertex = land_vertexes(land_index);
    path = LBSD.LEM_get_path(airways,launch_vertex,land_vertex,[]);
    requests(r).path = path;
    len_path = length(path);
    path_vertexes = zeros(len_path,6);
    for ind = 1:len_path
        path_vertexes(ind,:) = airways.lanes(path(ind),:);
    end
    requests(r).path_vertexes = path_vertexes;
    requests(r).launch_index = launch_index;
    requests(r).land_index = land_index;
end
tch = 0;
