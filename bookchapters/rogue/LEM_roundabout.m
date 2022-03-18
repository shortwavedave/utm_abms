function [r_up,r_dn] = LEM_roundabout(airways,v)
% LEM_roundabout - create a roundabout for a ground vertex
% On input:
%     airways (airway struct): airway info
%     v (int): ground vertex index
% On output:
%     r_up (roundabout struct): 
%       .info
%         .vertex (int): associated ground vertex
%         .nei_indexes (kx1 vector): ground vertex neighbor indexes
%         .angles (mx1 vector): angles to neighbors 
%         .angles_nei (mx1 vector): ground vertex index for each angle 
%             -1 for launch vertex; -2 for land vertex; -3 for fill vertex
%         .radius (float): circle radius for roundabout
%         .lanes (mx6 array): entry and exit points for lane
%         .r_pts (mx3 array): roundabout lane endpoints
% Call:
%     r1 = LEM_roundabout(airways,2);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

roundabout.vertex = v;

vertexes = airways.vertexes;
edges = airways.edges;
launch_vertexes = airways.launch_vertexes;
land_vertexes = airways.land_vertexes;
min_lane_len = airways.min_lane_len;
g_z_upper = airways.g_z_upper;
g_z_lower = airways.g_z_lower;

nei = unique([edges(find(edges(:,1)==v),2);edges(find(edges(:,2)==v),1)]);
roundabout.nei_indexes = nei;
num_nei = length(nei);
pts = vertexes(nei,:);
angles = zeros(num_nei,1);
for n = 1:num_nei
    dir = pts(n,:)- vertexes(v,:);
    angles(n) = LEM_posori(atan2(dir(2),dir(1)));
end
[angles,indexes] = sort(angles);
angles_nei = nei(indexes);

pts = pts(indexes,:);
anglesd = zeros(360,1);
anglesd(floor(angles*180/pi)+1) = 1;
if find(launch_vertexes==v)
    indexes = find(anglesd);
    max_angle = 0;
    max_index = 0;
    for ind = 1:length(indexes)-1
        s = indexes(ind+1) - indexes(ind);
        if s>max_angle
           max_angle = s;
           max_index = ind;
        end
    end
    if 360 - indexes(end) + indexes(1)>max_angle
        max_angle = 360 - indexes(end) + indexes(1);
        max_index = length(indexes);
    end
    new_angle = LEM_posori(angles(max_index) + pi*(max_angle/2)/180);
    if new_angle<angles(1)
        angles = [new_angle;angles];
        angles_nei = [-1;angles_nei];
    elseif new_angle>angles(end)
        angles = [angles;new_angle];
        angles_nei = [angles_nei;-1];
    else
        angles = [angles(1:max_index);new_angle;angles(max_index+1:end)];
        angles_nei = [angles_nei(1:max_index);-1;...
            angles_nei(max_index+1:end)];
    end
end
anglesd = zeros(360,1);
anglesd(floor(angles*180/pi)+1) = 1;
if find(land_vertexes==v)
    indexes = find(anglesd);
    max_angle = 0;
    max_index = 0;
    for ind = 1:length(indexes)-1
        s = indexes(ind+1) - indexes(ind);
        if s>max_angle
           max_angle = s;
           max_index = ind;
        end
    end
    if 360 - indexes(end) + indexes(1)>max_angle
        max_angle = 360 - indexes(end) + indexes(1);
        max_index = length(indexes);
    end
    new_angle = LEM_posori(angles(max_index) + pi*(max_angle/2)/180);
    if max_index==length(indexes)
        if new_angle<angles(1)
            angles = [new_angle;angles];
            angles_nei = [-2;angles_nei];
        else
            angles = [angles; new_angle];
            angles_nei = [angles_nei; -2];
        end
    else
        angles = [angles(1:max_index);new_angle;angles(max_index+1:end)];
        angles_nei = [angles_nei(1:max_index);-2;...
            angles_nei(max_index+1:end)];
    end
end
if length(angles)==1
    angles(2) = LEM_posori(angles(1) + 120*pi/180);
    angles(3) = LEM_posori(angles(2) + 120*pi/180);
    angles_nei(2) = -3;
    angles_nei(3) = -3;
end
if length(angles)==2
    angle1 = angles(2) - angles(1);
    angle2 = 2*pi - angles(2) + angles(1);
    if angle1>angle2
        angles(3) = angles(1) + angle1/2;
    else
        angles(3) = LEM_posori(angles(2) + angle2/2);
    end
    angles_nei(3) = -4;
end
[angles,indexes] = sort(angles);
angles_nei = angles_nei(indexes);

angle_diffs = diff(angles);
angle_diffs(end+1) = 2*pi - angles(end) + angles(1);
min_angle_diff = min(angle_diffs);
radius = (min_lane_len/2)/sin(min_angle_diff/2);
nei_pts = vertexes(nei,:);
min_dist = Inf;
for n = 1:num_nei
    d = norm(vertexes(v,:)-vertexes(nei(n),:));
    min_dist = min(min_dist,d);
end
radius = min(radius,min_dist/2);
num_angles = length(angles);
r_pts = zeros(num_angles,3);
r_pts(:,1) = vertexes(v,1) + radius*cos(angles);
r_pts(:,2) = vertexes(v,2) + radius*sin(angles);
%%r_pts(:,3) = vertexes(v,3);
r_pts(:,3) = g_z_upper;
num_r_pts = length(r_pts(:,1));
roundabout.angles = angles;
roundabout.angles_nei = angles_nei;
roundabout.radius = radius;
lanes = zeros(num_r_pts,6);
for p = 1:num_r_pts-1
    lanes(p,:) = [r_pts(p,1:3),r_pts(p+1,1:3)];
end
lanes(end,:) = [r_pts(num_r_pts,:),r_pts(1,:)];
roundabout.lanes = lanes;

r_up = roundabout;
r_dn = roundabout;
%%r_dn.lanes(:,3) = r_dn.lanes(:,3) + g_z_lower;
%%r_dn.lanes(:,6) = r_dn.lanes(:,6) + g_z_lower;
%%r_up.lanes(:,3) = r_up.lanes(:,3) + g_z_upper;
%%r_up.lanes(:,6) = r_up.lanes(:,6) + g_z_upper;
r_dn.lanes(:,3) = g_z_lower;
r_dn.lanes(:,6) = g_z_lower;
num_r_pts = length(r_pts(:,1));
%%r_dn.r_pts = r_dn.lanes(:,1:3);
%%r_up.r_pts = r_up.lanes(:,1:3);
r_dn.r_pts = [r_pts(:,1:2),ones(num_r_pts,1)*g_z_lower];
r_up.r_pts = [r_pts(:,1:2),ones(num_r_pts,1)*g_z_upper];

tch = 0;
