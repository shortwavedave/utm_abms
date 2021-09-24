function flight_out = LEM_gen_traj(obj,flight,del_t)
%

flight_out = flight;
flight_out.traj = [];
speed = flight.speed;
route = flight.route;
if isempty(route)
    return
end

len_route = length(route(:,1));
traj = [];

for k = 1:len_route
    pt1 = route(k,1:3);
    pt2 = route(k,4:6);
    t1 = route(k,7);
    t2 = route(k,8);
    times = [t1:del_t:t2];
    if length(times)>1&times(end)<t2
        times(end+1) = t2;
    end
    num_times = length(times);
    dir = pt2 - pt1;
    dir = dir/norm(dir);
    for t = 1:num_times
        t_across = times(t) - t1;
        pt = pt1 + t_across*dir*speed;
        traj = [traj; t1+t_across,pt];
    end
end
flight_out.traj = traj;

