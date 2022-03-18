function traj = LEM_lanes2traj(airways,path,speed,del_t)
%

lanes = airways.lanes;
len_path = length(path);
traj = [];

for p = 1:len_path
    pt1 = lanes(path(p),1:3);
    pt2 = lanes(path(p),4:6);
    dist = norm(pt2-pt1);
    dir = pt2 - pt1;
    dir = dir/norm(dir);
    pt = pt1;
    while norm(pt-pt1)<dist
        pt = pt + speed*dir*del_t;
        traj = [traj;pt];
    end
    if norm(pt-pt1)>=dist
        traj(end,:) = [];
    end
end
traj(end+1,:) = lanes(path(end),4:6);
num_pts = length(traj(:,1));
traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
