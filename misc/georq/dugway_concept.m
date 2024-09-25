%% Query the input roads layer
input_road_layer = 'slc_hill_dugway_concept_explode';
xformer_url = 'http://localhost:8080/v1/graph';
roads = georq_get_roads(input_road_layer, false, xformer_url);
locations = roads.vertexes(:,1:2);
elevs = LEM_get_elevation(locations,3857);
roads.vertexes =  [roads.vertexes, elevs];
%% Get Hill Bounds
projection = projcrs(3857);
ll_bounds = [-113.5484839933653234,40.8556978739420629; ...
    -112.8143582005836549,41.1936724416081645];
[maxx,maxy] = projfwd(projection,ll_bounds(2,2), ll_bounds(2,1));
[minx,miny] = projfwd(projection,ll_bounds(1,2), ll_bounds(1,1));
roads_hill = LEM_gen_Delaunay_roads(minx,maxx,miny,maxy,500,200);
locations = roads_hill.vertexes(:,1:2);
elevs = LEM_get_elevation(locations,3857);
roads_hill.vertexes =  [roads_hill.vertexes(:,1:2), elevs];
% Get Dugway Bounds
ll_bounds = [-113.7045579808070954,39.9045936693975278;...
    -112.7970166464234580,40.6433093643145611];
[maxx,maxy] = projfwd(projection,ll_bounds(2,2), ll_bounds(2,1));
[minx,miny] = projfwd(projection,ll_bounds(1,2), ll_bounds(1,1));
roads_dugway = LEM_gen_Delaunay_roads(minx,maxx,miny,maxy,500,200);
locations = roads_dugway.vertexes(:,1:2);
elevs = LEM_get_elevation(locations,3857);
roads_dugway.vertexes =  [roads_dugway.vertexes(:,1:2), elevs];
%% Combine roads
roads2.vertexes = [roads.vertexes;roads_hill.vertexes;roads_dugway.vertexes];
hill_start_ind = size(roads.vertexes,1);
dugway_start_ind = hill_start_ind + size(roads_hill.vertexes,1);

roads2.edges = [roads.edges; roads_hill.edges + hill_start_ind; ...
    roads_dugway.edges + dugway_start_ind];

%% Query the input roads layer
rng(0);
num_sites = 30;
num_flights = 100;
make_movie = 1;
% Choose a couple random launch sites
launch_sites = unique(randi(length(roads2.vertexes),num_sites,1));
% Every vertex in the network is a land site
% land_sites = 1:length(roads.vertexes);
land_sites = unique(randi(length(roads2.vertexes),num_sites,1));

MIN_LANE_LENGTH = 2;
MIN_ALTITUDE = 60;
MAX_ALTITUDE = 100;

num_vertexes = size(roads2.vertexes,1);
disp(['Num vertexes: ', num2str(num_vertexes)])

% res = LEM_sim2_LBSD(roads, launch_sites, land_sites, num_flights, make_movie);
% P = LEM_performance(res.flights);
% disp('delay (min requested start time - actual start time)')
% mean(P(:,2))
% disp('deconfliction time (secs)')
% mean(P(:,4))
airways = LEM_gen_airways(roads2,launch_sites,land_sites,MIN_LANE_LENGTH,...
    MIN_ALTITUDE,MAX_ALTITUDE);

%% Publish Airways to GeoRq
publish_url = 'http://localhost:8080/v1/lanes';
% ok = publish_lanes(airways, 'sf_reduced_airways_lo', false)
ok = publish_lanes(airways, 'hill_dugway_concept', false)