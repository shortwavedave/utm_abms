%% Query the input roads layer
input_road_layer = 'u_of_u_roads_elev';
xformer_url = 'http://localhost:8080/v1/graph';
roads = georq_get_roads(input_road_layer, false, xformer_url);

%% Query the input roads layer
rng(0);
num_sites = 10;
num_flights = 100;
make_movie = 1;
% Choose a couple random launch sites
launch_sites = unique(randi(length(roads.vertexes),num_sites,1));
% Every vertex in the network is a land site
land_sites = 1:length(roads.vertexes);

res = LEM_sim2_LBSD(roads, launch_sites, land_sites, num_flights, make_movie);
P = LEM_performance(res.flights);
disp('delay (min requested start time - actual start time)')
mean(P(:,2))
disp('deconfliction time (secs)')
mean(P(:,4))

%% Publish Airways to GeoRq
publish_url = 'http://localhost:8080/v1/lanes';
ok = publish_lanes(res.airways, 'u_of_u_airways', false)