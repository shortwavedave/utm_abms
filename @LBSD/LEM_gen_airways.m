function airways = LEM_gen_airways(obj,roads,launch_sites,land_sites,...
    min_lane_len,altitude1,altitude2)
% LEM_gen_airways - generate airway lanes from a road network
% On input:
%     roads (road struct): road info
%     launch_site (1xm vector): vertex indexes of launch locations
%     land_sites (1xn vector): vertex indexes of lan locations
% On output:
%     airways (airway struct): lane information
% Call:
%     lanes_SLC = LEM_gen_airways(roads_SLC,launch_SLC, land_SLC);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

airways = roads;
airways.vertexes(:,3) = 0;

num_vertexes = length(airways.vertexes(:,1));

airways.launch_vertexes = launch_sites;
airways.land_vertexes = land_sites;
airways.min_lane_len = min_lane_len;
airways.g_z_upper = altitude2; %534
airways.g_z_lower = altitude1; %467

airways = LBSD.LEM_gen_lanes(airways);
airways.vertexes = roads.vertexes;
airways = LBSD.LEM_add_ground_height(airways);
