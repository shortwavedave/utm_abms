function flights_out = LEM_add_flight(flights_in,airways,fp,path,vertexes)
%

flights = flights_in;
lanes = airways.lanes;
[num_lanes,~] = size(lanes);
num_flights = length(flights);
num_flights = num_flights + 1;
flights(num_flights).type = 1;
flights(num_flights).decon_time = 0;
flights(num_flights).plan = fp;
[len_fp,~] = size(fp);
route = zeros(len_fp,9);
route(:,7:9) = fp(:,1:3);
for k = 1:len_fp
    route(k,1:6) = lanes(path(k),:);
end
flights(num_flights).route = route;
flights(num_flights).telemetry = [route(1,1:3),0,0,1,fp(1,3)];
flights(num_flights).start_interval = [fp(1,1),fp(end,2)];
flights(num_flights).speed = fp(1,3);
flights(num_flights).path = path;
flights(num_flights).path_vertexes = vertexes;
flights(num_flights).launch_index = path(1);
flights(num_flights).land_index = path(end);
flights(num_flights).start_time = fp(1,1);
flights(num_flights).end_time = fp(end,2);
flights(num_flights).traj = [route(1,1:3),fp(1,1)];
flights_out = flights;
