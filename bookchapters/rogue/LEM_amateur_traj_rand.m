function traj = LEM_amateur_traj_rand(launch_site,speed,radius,...
    sphere_height,num_bounces,del_t,max_t)
%

ZERO_DIST = 0.01;

traj = [launch_site];

cur_t = 0;

% Go ~50 feet up
t_stage1 = sphere_height/speed;
while cur_t<min(max_t,t_stage1)
    cur_t = cur_t + del_t;
    traj = [traj;traj(end,1),traj(end,2),traj(end,3)+speed*del_t];
end
if abs(cur_t-max_t)<del_t
    return
end
clf
hold on

%go to random point on sphere centered at launch_site+[0,0,50]
bounce = 0;
while cur_t<max_t&bounce<num_bounces
    bounce = bounce + 1
    pt1 = traj(end,:);
    % pick random point
    theta = 2*pi*rand;
    x = launch_site(1) + rand*radius*cos(theta);
    y = launch_site(2) + rand*radius*sin(theta);
    z = sqrt(radius^2-(x-launch_site(1))^2-(y-launch_site(2))^2);
    if rand<0.5
        z = -z;
    end
    z = z + sphere_height;
    pt2 = [x,y,z];
    plot3(pt2(1),pt2(2),pt2(3),'k.');
    dir = pt2 - pt1;
    dir = dir/norm(dir);
    pt = pt1;
    done = 0;
    while cur_t<max_t&done==0
        cur_t = cur_t + del_t;
        pt = pt + del_t*speed*dir;
        traj = [traj;pt];
        if norm(pt-pt2)<ZERO_DIST|norm(pt-pt1)>norm(pt2-pt1)
            done = 1;
        end
    end
end
pt1 = traj(end,:);
pt2 = [launch_site(1:2),sphere_height];
dir = pt2 - pt1;
dir = dir/norm(dir);
