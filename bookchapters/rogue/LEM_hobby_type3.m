function traj = LEM_hobby_type3(center,launch_site,height,speed,del_t,...
    ratio)
% LEM_hobby_type3 - spiral up and down
% On input:
%     center (3x1 vector): center of spiral radius
%     launch_site (3x1 vector): ground launch point
%     height (float): altitude for center of sphere
%     speed (float): speed of UAS
%     del_t (float): time sample interval
%     ratio (float): determines radius of error circle for traj noise
%       radius is the inter-point distance * ratio
% On ouput:
%     traj (nx3 array): x,y,z,t
% Call:
%     traja = LEM_hobby_type3([cx;cy;cz],[x;y;z],10,1,0.1,0.5);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

traj = [launch_site'];
radius = norm(center-launch_site);
thetad = 0;
x = launch_site(1);
y = launch_site(2);
z = launch_site(3);
while z<height
    thetad = mod(thetad + 1,360);
    x = center(1) + radius*cosd(thetad);
    y = center(2) + radius*sind(thetad);
    z = z + speed*del_t;
    traj = [traj;x,y,z];
end
if z>height
    traj(end,3) = height;
end

while z>launch_site(3)
    thetad = mod(thetad + 1,360);
    x = center(1) + radius*cosd(thetad);
    y = center(2) + radius*sind(thetad);
    z = z - speed*del_t;
    traj = [traj;x,y,z];
end
if traj(end,3)<launch_site(3)
    traj(end,3) = launch_site(3);
end

num_pts = length(traj(:,1));
traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
trajn = LEM_noisy_traj(traj,ratio,speed,del_t);
traj = trajn;

tch = 0;
