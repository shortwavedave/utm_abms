function traj = LEM_rogue_type2(airways,launch_site,land_site,...
    num_disrupt,min_speed,max_speed,del_t,ratio)
% LEM_rogue_type2 - disrupt lanes
% On input:
%     airways (airways struct): airway info
%     launch_site (3x1 vector): ground launch point
%     land_site (3x1 vector): ground land point
%     num_disrupt (int): number of lanes to interupt
%     min_speed (float): min speed to fly
%     max_speed (float): max speed to fly
%     del_t (float): time sample interval
%     ratio (float): determines radius of error circle for traj noise
%       radius is the inter-point distance * ratio
% On ouput:
%     traj (nx4 array): x,y,z,t
% Call:
%     traja = LEM_rogue_type2(a1,[x1;y1;z1],[x2;y2;z2],7,5,10,0.1,0.5);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

lanes = airways.lanes;
lanes(airways.launch_lane_vertexes,:) = [];
lanes(airways.land_lane_vertexes,:) = [];
[num_lanes,~] = size(lanes);
perms = randperm(num_lanes);
indexes = perms(1:num_disrupt);

c_pt = launch_site;
index = 0;
traj = [c_pt'];

for k = 1:num_disrupt
    speed = min_speed + rand*(max_speed-min_speed);
    index = index + 1;
    n_pt = lanes(indexes(index),1:3)';
    dist = norm(n_pt-c_pt);
    dir = n_pt - c_pt;
    dir = dir/norm(dir);
    pt = c_pt;
    while norm(pt-c_pt)<dist
        pt = pt + speed*dir*del_t;
        traj = [traj;pt'];
    end
    if norm(pt-c_pt)>dist
        traj(end,:) = n_pt;
    end
    c_pt = n_pt;
    n_pt = lanes(indexes(index),4:6)';
    dist = norm(n_pt-c_pt);
    dir = n_pt - c_pt;
    dir = dir/norm(dir);
    pt = c_pt;
    while norm(pt-c_pt)<dist
        pt = pt + speed*dir*del_t;
        traj = [traj;pt'];
    end
    if norm(pt-c_pt)>dist
        traj(end,:) = n_pt;
    end
end

% down
dist = norm(n_pt-land_site);
dir = land_site - n_pt;
dir = dir/norm(dir);
pt = traj(end,:)';
while norm(pt-n_pt)<dist
    pt = pt + speed*del_t*dir;
    traj = [traj;pt'];
end
if norm(pt-n_pt)>dist
    traj(end,:) = land_site;
end
num_pts = length(traj(:,1));
traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
trajn = LEM_noisy_traj(traj,ratio,speed,del_t);
traj = trajn;

tch = 0;
