function res = LEM_classify_tests(airways,model,N,ratio,traj_len)
%

rng('default');

speed = 1;
min_speed = 1;
max_speed = 3;
del_t = 0.1;
res = zeros(N,6);

num_launch = length(airways.launch_lane_vertexes);
num_land = length(airways.land_lane_vertexes);
x_min = min(airways.lanes(:,1));
x_max = max(airways.lanes(:,1));
y_min = min(airways.lanes(:,2));
y_max = max(airways.lanes(:,2));
z_min = min(airways.lanes(:,3));
z_max = max(airways.lanes(:,3));
for s = 1:N
    [s,N]
    % Nominal
    launch = airways.launch_lane_vertexes(randi(num_launch));
    land = airways.land_lane_vertexes(randi(num_land));
    [path,vpath] = LEM_get_path(airways,launch,land,[]);
    t = LEM_path2traj(airways,path,speed,del_t);
    t = LEM_noisy_traj(t,ratio,speed,del_t);
    [traj_len,~] = size(t);
    res(s,1) = LEM_NAB_classify(airways,model,t,speed,del_t,traj_len);
    % Hobby 1
    x = x_min + (x_max-x_min)*rand;
    y = y_min + (y_max-y_min)*rand;
    z = z_min;
    alt = z_min + (z_max-z_min)*rand;
    radius = 50;
    num_moves = randi(20);
    t = LEM_hobby_type1([x;y;z],alt,radius,num_moves,speed,del_t,...
        ratio);
    [traj_len,~] = size(t);
    res(s,2) = LEM_NAB_classify(airways,model,t,speed,del_t,traj_len);
    % Hobby 2
    x = x_min + (x_max-x_min)*rand;
    y = y_min + (y_max-y_min)*rand;
    z = z_min;
    alt = z_min + (z_max-z_min)*rand;
    radius = 50;
    num_moves = randi(20);
    t = LEM_hobby_type2([x;y;z],alt,radius,num_moves,speed,del_t,...
        10,20,ratio);
    [traj_len,~] = size(t);
    res(s,3) = LEM_NAB_classify(airways,model,t,speed,del_t,traj_len);
    % Hobby 3
    x = x_min + (x_max-x_min)*rand;
    y = y_min + (y_max-y_min)*rand;
    z = z_min;
    alt = z_min + (z_max-z_min)*rand;
    radius = 50;
    num_moves = randi(20);
    t = LEM_hobby_type3([x;y;z],[x+5;y;z],alt,speed,del_t,ratio);
    [traj_len,~] = size(t);
    res(s,4) = LEM_NAB_classify(airways,model,t,speed,del_t,traj_len);
    % Rogue 1
    x1 = x_min + (x_max-x_min)*rand;
    y1 = y_min + (y_max-y_min)*rand;
    z1 = z_min;
    x2 = x_min + (x_max-x_min)*rand;
    y2 = y_min + (y_max-y_min)*rand;
    z2 = z_min;
    alt = z_max - 10*rand;
    t = LEM_rogue_type1([x1;y1;z1],[x2;y2;z2],alt,speed,del_t,ratio);
    [traj_len,~] = size(t);
    res(s,5) = LEM_NAB_classify(airways,model,t,speed,del_t,traj_len);
    % Rogue 2
    launch = airways.launch_lane_vertexes(randi(num_launch));
    land = airways.land_lane_vertexes(randi(num_land));
    launch_site = airways.lanes(launch,1:3)';
    land_site = airways.lanes(land,4:6)';
    x1 = x_min + (x_max-x_min)*rand;
    y1 = y_min + (y_max-y_min)*rand;
    z1 = z_min;
    x2 = x_min + (x_max-x_min)*rand;
    y2 = y_min + (y_max-y_min)*rand;
    z2 = z_min;
    alt = z_max - 10*rand;
    num_disupt = randi(10) + 9;
    t = LEM_rogue_type2(airways,launch_site,land_site,num_disupt,...
        min_speed,max_speed,del_t,ratio);
    [traj_len,~] = size(t);
    res(s,6) = LEM_NAB_classify(airways,model,t,speed,del_t,traj_len);
    tch = 0;
end