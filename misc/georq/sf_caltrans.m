%% Query the input roads layer
input_road_layer = 'downtown_sf';
% input_road_layer = 'sf_reduced';
xformer_url = 'http://localhost:8080/v1/graph';
roads = georq_get_roads(input_road_layer, false, xformer_url);
locations = roads.vertexes(:,1:2);
elevs = LEM_get_elevation(locations,3857);
roads.vertexes =  [roads.vertexes, elevs];
%% Query the input roads layer
rng(0);
num_sites = 30;
num_flights = 100;
make_movie = 1;
% Choose a couple random launch sites
launch_sites = unique(randi(length(roads.vertexes),num_sites,1));
% Every vertex in the network is a land site
% land_sites = 1:length(roads.vertexes);
land_sites = unique(randi(length(roads.vertexes),num_sites,1));

MIN_LANE_LENGTH = 2;
MIN_ALTITUDE = 60;
MAX_ALTITUDE = 100;

num_vertexes = size(roads.vertexes,1);
disp(['Num vertexes: ', num2str(num_vertexes)])

% res = LEM_sim2_LBSD(roads, launch_sites, land_sites, num_flights, make_movie);
% P = LEM_performance(res.flights);
% disp('delay (min requested start time - actual start time)')
% mean(P(:,2))
% disp('deconfliction time (secs)')
% mean(P(:,4))
airways = LEM_gen_airways(roads,launch_sites,land_sites,MIN_LANE_LENGTH,...
    MIN_ALTITUDE,MAX_ALTITUDE);

%% Publish Airways to GeoRq
publish_url = 'http://localhost:8080/v1/lanes';
% ok = publish_lanes(airways, 'sf_reduced_airways_lo', false)
ok = publish_lanes(airways, 'downtown_sf_airways_v2', false)