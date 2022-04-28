function traj = LEM_hobby_type2(launch_site,height,radius,num_moves,...
    speed,del_t,min_wait,max_wait,ratio)
% LEM_hobby_type2 - up and random between sphere surface points
% On input:
%     launch_site (3x1 vector): ground aunch point
%     height (float): altitude for center of sphere
%     radius (float): radius of sphere
%     num_moves (int): number of random moves across sphere
%     speed (float): speed of UAS
%     del_t (float): time sample interval
%     min_wait (float): minimum time to hover
%     max_wait (float): maximum time to hover
%     ratio (float): determines radius of error circle for traj noise
%       radius is the inter-point distance * ratio
% On ouput:
%     traj (nx4 array): x,y,z,t
% Call:
%     traja = LEM_hobby_type2([x;y;z],10,5,10,1,0.1,10,50,0.5);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

% up
pt1 = launch_site;
pt2 = [pt1(1);pt1(2);height];
dist12 = norm(pt2-pt1);
dir = [0;0;1];
pt = pt1;
traj = [pt'];
while norm(pt1-pt)<dist12
    pt = pt + speed*dir*del_t;
    traj = [traj;pt'];
end
if norm(pt1-pt)>dist12
    traj(end,:) = pt2';
end

% Generate sphere moves
s_pts = LEM_sphere_pts(pt2,radius,num_moves);
c_pt = pt2;
for p = 1:num_moves
    n_pt = s_pts(p,:)';
    dir = n_pt - c_pt;
    dir = dir/norm(dir);
    dist_cn= norm(n_pt-c_pt);
    pt = c_pt;
    while norm(pt-c_pt)<dist_cn
        pt = pt + speed*del_t*dir;
        traj = [traj;pt'];
    end
    if norm(pt-c_pt)>dist_cn
        traj(end,:) = n_pt';
    end
    wait_time = min_wait + rand*(max_wait-min_wait);
    t = del_t;
    while t<wait_time
        t = t + del_t;
        traj = [traj;pt'];
    end
    c_pt = n_pt;
end

% down
dir = pt2 - c_pt;
dir = dir/norm(dir);
dist_c2= norm(c_pt-pt2);
pt = c_pt;
while norm(pt-c_pt)<dist_c2
    pt = pt + speed*del_t*dir;
    traj = [traj;pt'];
end
if norm(pt-c_pt)>dist_c2
    traj(end,:) = pt2';
end
dir = [0;0;-1];
pt = pt2;
while norm(pt2-pt)<dist12
    pt = pt + dir*del_t;
    traj = [traj;pt'];
end
if norm(pt2-pt)>dist12
    traj(end,:) = pt1';
end
num_pts = length(traj(:,1));
traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
% trajn = LEM_noisy_traj(traj,ratio,speed,del_t);
% traj = trajn;

tch = 0;
