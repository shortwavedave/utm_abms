function traj = LEM_rogue_type1(launch_site,land_site,height,speed,del_t,...
    ratio)
% LEM_rogue_type1 - up, over & down delivery
% On input:
%     launch_site (3x1 vector): ground launch point
%     land_site (3x1 vector): ground land point
%     height (float): altitude for center of sphere
%     speed (float): speed of UAS
%     del_t (float): time sample interval
%     ratio (float): determines radius of error circle for traj noise
%       radius is the inter-point distance * ratio
% On ouput:
%     traj (nx4 array): x,y,z,t
% Call:
%     traja = LEM_rogue_type1([x1;y1;z1],[x2;y2;z2],5,10,1,0.1,0.5);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

% up
pt1 = launch_site;
pt2 = [pt1(1);pt1(2);pt1(3)+height];
pt3 = land_site;
pt3(3) = pt2(3);
pt4 = land_site;

dist12 = norm(pt2-pt1);
dir = [0;0;1];
pt = pt1;
traj = [pt'];
while norm(pt1-pt)<dist12
    pt = pt + dir*del_t;
    traj = [traj;pt'];
end
if norm(pt1-pt)>dist12
    traj(end,:) = pt2';
end

% across
dist23 = norm(pt3-pt2);
dir = pt3 - pt2;
pt = traj(end,:)';
while norm(pt2-pt)<dist23
    pt = pt + speed*dir*del_t;
    traj = [traj;pt'];
end
if norm(pt2-pt)>dist23
    traj(end,:) = pt3';
end

% down
dist34 = norm(pt4-pt3);
dir = pt4 - pt3;
dir = dir/norm(dir);
pt = traj(end,:)';
while norm(pt-pt3)<dist34
    pt = pt + speed*del_t*dir;
    traj = [traj;pt'];
end
if norm(pt3-pt)>dist34
    traj(end,:) = pt4';
end
num_pts = length(traj(:,1));
traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
trajn = LEM_noisy_traj(traj,ratio,speed,del_t);
traj = trajn;

tch = 0;
