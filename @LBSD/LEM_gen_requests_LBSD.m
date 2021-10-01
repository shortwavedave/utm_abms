function requests = LEM_gen_requests_LBSD(obj,t_min,t_max,airways,...
    num_requests,launch_interval,speeds)
% LEM_gen_requests_LBSD - generate a set of flight requests for lanes
% On input:
%     t_min (float): earliest start time for flight
%     t_max (float): latest start time for flight
%     airways (airways struct): airways info
%     num_requests (int): number of requested flights
%     launch_interval (float): time spread for posisble launch
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
%     req = LEM_gen_requests_LBSD(0,10,airways,10,4,[20,30]);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

time_spread = t_max - t_min;
arrival_times = sort(t_min + rand(num_requests,1)*time_spread);
min_speed = min(speeds);
speed_spread = max(speeds) - min_speed;
num_requests = length(arrival_times);
times = zeros(num_requests,1);
launch_vertexes = airways.launch_lane_vertexes;
num_launch_vertexes = length(launch_vertexes);
land_vertexes = airways.land_lane_vertexes;
num_land_vertexes = length(land_vertexes);

for r = 1:num_requests
    requests(r).request_time = arrival_times(r);
    t1 = requests(r).request_time + rand*(t_max-arrival_times(r));
    times(r) = t1;
    t2 = t1 + launch_interval;
    requests(r).launch_interval = floor([t1,t2]);
    requests(r).speed = min_speed + rand*speed_spread;
    launch_index = randi(num_launch_vertexes);
    launch_vertex = launch_vertexes(launch_index);
    done = 0;
    while done==0
        land_index = randi(num_land_vertexes);
        land_vertex = land_vertexes(land_index);
        if launch_vertex~=land_vertex
            done = 1;
        end
    end
    path = LBSD.LEM_get_path(obj,airways,launch_vertex,land_vertex,[]);
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
