function airways_out = LEM_gen_lanes(obj,airways)
% LEM_gen_corridors - generate lanes for given network
% On input:
%     airways (primitive airways struct): initial network info
%       .vertexes (nx3 array): road network vertexes
%       .edges (mx2 array): vertex index pair for edges
%       .launch_vertexes (1xp vector): vertex indexes of launch sites
%       .land_vertexes (1xq vector): vertex indexes of land sites
% On output:
%     airways_out (airways struct): airway lane info
%       .vertexes (nx3 array); road intersections or endpoints
%       .edges (mx2 array): edges between road vertexes
%       .launch_vertexes (1xp vector): road vertexes for launching
%       .land_vertexes (1xq vector): road vertexes for landing
%       .min_lane_len (flioat): minimum lane length
%       .g_z_upper (float): upper altitude for lanes
%       .g_z_lower (float): lower altitude for lanes
%       .roundabouts_up (roundabouts struct):
%       .roundabouts_dn (roundabouts struct):
%       .lanes (kx6 array): lanes (entry point, exit point)
%       .lane_vertexes (wx3 array): lane vertexes
%       .lane_edges (zx2 array): vertex indexes for edges
%       .G (digraph): graph of airways
%       .launch_lane_vertexes (1xs vector): lane vertexes for launching
%       .land_lane_vertexes (1xt vector): lane vertexes for landing
% Call:
%     airways1 = LEM_gen_lanes(airways0);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

LAUNCH_CODE = -1;
LAND_CODE = -2;

vertexes = airways.vertexes;
num_vertexes = length(vertexes(:,1));
edges = airways.edges;
num_edges = length(edges(:,1));
all_lanes = [];

% Create roundabouts
for v = 1:num_vertexes
    [r_up,r_dn]= LBSD.LEM_roundabout(obj,airways,v);
    all_lanes = [all_lanes;r_up.lanes;r_dn.lanes];
    airways.roundabouts_up(v).info = r_up;
    airways.roundabouts_dn(v).info = r_dn;
end

% Create launch lanes
launch_vertexes = airways.launch_vertexes;
launch_lanes = [];
for v = 1:num_vertexes
    if find(launch_vertexes==v)
        angles_nei = airways.roundabouts_dn(v).info.angles_nei;
        v_lanes = airways.roundabouts_dn(v).info.lanes;
        index = find(angles_nei==LAUNCH_CODE);
        launch_lanes = [launch_lanes;[v_lanes(index,1:2),vertexes(v,3),...
            v_lanes(index,1:3)]];
    end
end
all_lanes = [all_lanes;launch_lanes];

% Create land lanes
land_vertexes = airways.land_vertexes;
land_lanes = [];
for v = 1:num_vertexes
    if find(land_vertexes==v)
        angles_nei = airways.roundabouts_dn(v).info.angles_nei;
        v_lanes = airways.roundabouts_dn(v).info.lanes;
        index = find(angles_nei==LAND_CODE);
        land_lanes = [land_lanes;[v_lanes(index,1:3),...
            v_lanes(index,1:2),vertexes(v,3)]];
    end
end
all_lanes = [all_lanes; land_lanes];

% Create lanes between connected vertexes
e_lanes = [];
for e = 1:num_edges
    v1 = edges(e,1);
    v2 = edges(e,2);
    pt1 = vertexes(v1,:);
    pt2 = vertexes(v2,:);
    dir = pt2 - pt1;
    dir = dir/norm(dir);
    theta = LBSD.LEM_posori(obj,atan2(dir(2),dir(1)));
    angles_nei1 = airways.roundabouts_dn(v1).info.angles_nei;
    angles_nei2 = airways.roundabouts_dn(v2).info.angles_nei;
    index12 = find(angles_nei1==v2);
    index21 = find(angles_nei2==v1);
    pt1_dn = airways.roundabouts_dn(v1).info.lanes(index12,1:3);
    pt1_up = airways.roundabouts_up(v1).info.lanes(index12,1:3);
    pt2_dn = airways.roundabouts_dn(v2).info.lanes(index21,1:3);
    pt2_up = airways.roundabouts_up(v2).info.lanes(index21,1:3);
    if theta<pi
        e_lanes = [e_lanes; [pt1_up,pt2_up]; [pt2_dn,pt1_dn]];
    else
        e_lanes = [e_lanes; [pt1_dn,pt2_dn]; [pt2_up,pt1_up]];
    end
end
all_lanes = [all_lanes; e_lanes];

% Create roundabout up-down lanes
ud_lanes = [];
for v = 1:num_vertexes
    angles_nei = airways.roundabouts_dn(v).info.angles_nei;
    lanes = airways.roundabouts_dn(v).info.lanes;
    index_launch = find(angles_nei==LAUNCH_CODE);
    index_land = find(angles_nei==LAND_CODE);
    if ~isempty(index_launch)&~isempty(index_land)
        ud_lanes = [ud_lanes; [lanes(index_launch,1:3),...
            lanes(index_launch,1:2),airways.g_z_upper]];
        ud_lanes = [ud_lanes; [lanes(index_land,1:2),airways.g_z_upper,...
            lanes(index_land,1:3)]];
    elseif ~isempty(index_launch)
        ud_lanes = [ud_lanes; [lanes(index_launch,1:3),...
            lanes(index_launch,1:2),airways.g_z_upper]];
        index2 = mod(index_launch,length(lanes(:,1))) + 1;
        ud_lanes = [ud_lanes; [lanes(index2,1:2),airways.g_z_upper,...
            lanes(index2,1:3)]];
    elseif ~isempty(index_land)
        index1 = mod(index_land,length(lanes(:,1))) + 1;
        ud_lanes = [ud_lanes; [lanes(index1,1:3),...
            lanes(index1,1:2),airways.g_z_upper]];
        ud_lanes = [ud_lanes; [lanes(index_land,1:2),airways.g_z_upper,...
            lanes(index_land,1:3)]];
    else
        ud_lanes = [ud_lanes; [lanes(1,1:3),...
            lanes(1,1:2),airways.g_z_upper]];
        ud_lanes = [ud_lanes; [lanes(2,1:2),airways.g_z_upper,...
            lanes(2,1:3)]];
    end
end
all_lanes = [all_lanes; ud_lanes];

airways.lanes = all_lanes;
num_all_lanes = length(all_lanes(:,1));
lane_vertexes = [all_lanes(:,1:3);all_lanes(:,4:6)];
lane_vertexes = LBSD.LEM_elim_redundant(obj,lane_vertexes);
airways.lane_vertexes = lane_vertexes;
edges = zeros(num_all_lanes,2);
for e = 1:num_all_lanes
    pt1 = all_lanes(e,1:3);
    pt2 = all_lanes(e,4:6);
    index1 = find(pt1(1)==lane_vertexes(:,1)...
        &pt1(2)==lane_vertexes(:,2)&pt1(3)==lane_vertexes(:,3));
    index2 = find(pt2(1)==lane_vertexes(:,1)...
        &pt2(2)==lane_vertexes(:,2)&pt2(3)==lane_vertexes(:,3));
    edges(e,:) = [index1, index2];
end
airways.lane_edges = edges;
G = LBSD.LEM_airways2graph(obj,airways);
airways.G = G;

% Set launch lane vertexes
[num_launch_lanes,dummy] = size(launch_lanes);
launch_lane_vertexes = zeros(1,num_launch_lanes);
for k = 1:num_launch_lanes
    launch_lane_vertexes(k) = find(launch_lanes(k,1)==lane_vertexes(:,1)...
        &launch_lanes(k,2)==lane_vertexes(:,2)...
        &launch_lanes(k,3)==lane_vertexes(:,3));
end
airways.launch_lane_vertexes = launch_lane_vertexes;

% Set land lane vertexes
[num_land_lanes,dummy] = size(land_lanes);
land_lane_vertexes = zeros(1,num_land_lanes);
for k = 1:num_land_lanes
    land_lane_vertexes(k) = find(land_lanes(k,4)==lane_vertexes(:,1)...
        &land_lanes(k,5)==lane_vertexes(:,2)...
        &land_lanes(k,6)==lane_vertexes(:,3));
end
airways.land_lane_vertexes = land_lane_vertexes;

num_lanes = length(airways.lanes(:,1));
lane_lengths = zeros(num_lanes,1);
for s = 1:num_lanes
    lane_lengths(s) = norm(airways.lanes(s,4:6)-airways.lanes(s,1:3));
end
airways.lane_lengths = lane_lengths;

airways_out = airways;

tch = 0;
