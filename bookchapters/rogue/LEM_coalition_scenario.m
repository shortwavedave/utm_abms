function res = LEM_coalition_scenario
%

rng('default');

% Create 3 levels of airways
num_ISR = 5;
num_manned = 5;
num_UAS = 5;
num_bogeys = 5;
num_artillery = 500;

INDEX_ISR = 1;
INDEX_MANNED = 2;
INDEX_UAS = 3;
INDEX_BOGEY = 4;
INDEX_ARTILLERY = 5;

t_min = 0;
t_max = 240;
speed = 20;

% ISR flights
roads_ISR = LEM_gen_grid_roads(0,5000,0,5000,1000,1000);
airways_ISR = LEM_gen_airways(roads_ISR,[1],[2],50,18000,18200);
flights = [];
num_flights = 0;

num_lanes_ISR = length(airways_ISR.lanes);
for k = 1:num_lanes_ISR
    reservations_ISR(k).flights= [];
end

% Manned Flights
roads_manned = LEM_gen_grid_roads(0,5000,0,5000,500,500);
airways_manned = LEM_gen_airways(roads_manned,[1],[2],30,4000,4200);
num_lanes_manned = length(airways_manned.lanes);
for k = 1:num_lanes_manned
    reservations_manned(k).flights= [];
end

% UAS flights
roads_UAS = LEM_gen_grid_roads(0,5000,0,5000,300,300);
airways_UAS = LEM_gen_airways(roads_manned,[1:10],[11:20],10,600,620);
num_lanes_UAS = length(airways_UAS.lanes);
for k = 1:num_lanes_UAS
    reservations_UAS(k).flights= [];
end

% Generate Coalition Force Flights

% Generate Coalition Member 1 Requests
% ISR: 5 platforms
p1 = LEM_get_path(airways_ISR,251,220,[]);
p2 = LEM_get_path(airways_ISR,220,250,[]);
p3 = LEM_get_path(airways_ISR,250,35,[]);
p4 = LEM_get_path(airways_ISR,35,220,[]);
p5 = LEM_get_path(airways_ISR,220,35,[]);
p6 = LEM_get_path(airways_ISR,35,252,[]);
path = [p1,p2,p3,p4,p5,p6];
path_ISR = path;
%path_ISR = [251,     6,     3,     1,    41,    39,    85,    83,...
%   129,   127,   173,   171,   216,   219,   220,   220,...
%   218,   215,   223,   221,   229,   227,   235,   233,...
%   241,   239,  247,   245,   246,   249,   250,   250,...
%   248,   249,   212,   213,   168,   169,   124,   125,...
%    80,   81,    36,    33,    34,    37,    38,    31,...
%    32,    25,    26,    19,   20,    12,    13,    14,     5,     6];
hd = 10;
for r = 1:num_ISR
    requests_ISR(r).request_time = 0;
    requests_ISR(r).launch_interval = [0,240];
    requests_ISR(r).speed = speed;
    requests_ISR(r).path = path_ISR;
    len_path = length(path_ISR);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_ISR.lanes(path_ISR(p),:)];
    end
    requests_ISR(r).path_vertexes = vertexes;
    requests_ISR(r).launch_index = path_ISR(1);
    requests_ISR(r).land_index = path_ISR(end);
    [fp,reservations_ISR] = LEM_reserve_fp(reservations_ISR,airways_ISR,...
        t_min,t_max,speed,path_ISR,hd);
    num_flights = num_flights + 1;
    flights = LEM_add_flight(flights,airways_ISR,fp,path_ISR,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_ISR;
    flights(end).team = 1;
    flights(end).delay = 0;
end

% manned flights

%path = [891,6,3,1,71,69,155,153,239,237,323,321,407,405,491,489,575,573,...
%    659,657,743,741,826,827,825,833,831,839,837,845,843,851,849,857,...
%    855,863,861,869,867,875,873,881,879,887,885,886,889,890,888,889,...
%    822,823,738,739,654,655,570,571,486,487,402,403,318,319,234,235,150,...
%    151,66,63,64,67,68,61,62,55,56,49,50,43,44,37,38,31,32,25,...
%    26,19,20,12,13,14,5,6];
manned2 = [170,370,650];
manned3 = [825,890];
manned4 = [65];
manned5 = [892];
for r = 1:num_manned
    requests_manned(r).request_time = 0;
    requests_manned(r).launch_interval = [0,240];
    requests_manned(r).speed = 20;
    v1 = 891;
    v2 = manned2(randi(3));
    v3 = manned3(randi(2));
    v4 = manned4;
    v5 = manned5;
    leg1 = LEM_get_path(airways_manned,v1,v2);
    leg2 = LEM_get_path(airways_manned,v2,v3);
    leg3 = LEM_get_path(airways_manned,v3,v4);
    leg4 = LEM_get_path(airways_manned,v4,v5);
    path = [leg1,leg2,leg3,leg4];
    requests_manned(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_manned.lanes(path(p),:)];
    end
    requests_manned(r).path_vertexes = vertexes;
    requests_manned(r).launch_index = path(1);
    requests_manned(r).land_index = path(end);
    [fp,reservations_manned] = LEM_reserve_fp(reservations_manned,...
        airways_manned,t_min+1000,t_max,speed,path,hd);
    flights = LEM_add_flight(flights,airways_manned,fp,path,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_MANNED;
    flights(end).team = 1;
    flights(end).delay = 0;
end

%path = [925,6,3,1,87,88,85,189,187,273,271,357,355,441,439,525,523,609,...
%    607,693,691,777,775,860,861,859,867,865,873,871,879,877,885,883,891,...
%    889,897,895,903,901,909,907,915,913,921, 919,920,923,856,857,772,773,...
%    688,689,604,605,520,521,436,437,352,353,268,269,184,185,82,83,84,76,...
%    77,78,68,69,70,60,61,62,52,53,54,44,45,46,36,37,38,28,29,30,20,21,...
%    22,12,13,14,5,6];
UAS1 = [925:934];
UAS2 = [380,410,650];
UAS3 = [860,920];
UAS4 = [80];
UAS5 = [935:944];
for r = 1:num_UAS
    requests_UAS(r).request_time = 0;
    requests_UAS(r).launch_interval = [0,240];
    requests_UAS(r).speed = 20;
    v1 = UAS1(randi(10));
    v2 = UAS2(randi(3));
    v3 = UAS3(randi(2));
    v4 = UAS4;
    v5 = UAS5(randi(10));
    leg1 = LEM_get_path(airways_UAS,v1,v2);
    leg2 = LEM_get_path(airways_UAS,v2,v3);
    leg3 = LEM_get_path(airways_UAS,v3,v4);
    leg4 = LEM_get_path(airways_UAS,v4,v5);
    path = [leg1,leg2,leg3,leg4];
    requests_UAS(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_UAS.lanes(path(p),:)];
    end
    requests_UAS(r).path_vertexes = vertexes;
    requests_UAS(r).launch_index = path(1);
    requests_UAS(r).land_index = path(end);
    [fp,reservations_UAS] = LEM_reserve_fp(reservations_UAS,...
        airways_UAS,t_min+1500,t_max,speed,path,hd);
    flights = LEM_add_flight(flights,airways_UAS,fp,path,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_UAS;
    flights(end).team = 1;
    flights(end).delay = 0;
end

% Generate Coalition Member 2 Requests
path = path_ISR;
hd = 10;
for r = 1:num_ISR
    requests_ISR(r).request_time = 0;
    requests_ISR(r).launch_interval = [0,240];
    requests_ISR(r).speed = 20;
    requests_ISR(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_ISR.lanes(path(p),:)];
    end
    requests_ISR(r).path_vertexes = vertexes;
    requests_ISR(r).launch_index = path(1);
    requests_ISR(r).land_index = path(end);
    [fp,reservations_ISR] = LEM_reserve_fp(reservations_ISR,airways_ISR,...
        t_min+100,t_max,speed,path_ISR,hd);
    flights = LEM_add_flight(flights,airways_ISR,fp,path_ISR,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_ISR;
    flights(end).team = 2;
    flights(end).delay = 0;
end
% manned flights

%path = [891,6,3,1,71,69,155,153,239,237,323,321,407,405,491,489,575,573,...
%    659,657,743,741,826,827,825,833,831,839,837,845,843,851,849,857,...
%    855,863,861,869,867,875,873,881,879,887,885,886,889,890,888,889,...
%    822,823,738,739,654,655,570,571,486,487,402,403,318,319,234,235,150,...
%    151,66,63,64,67,68,61,62,55,56,49,50,43,44,37,38,31,32,25,...
%    26,19,20,12,13,14,5,6];
manned2 = [170,370,650];
manned3 = [825,890];
manned4 = [65];
manned5 = [892];
for r = 1:num_manned
    requests_manned(r).request_time = 0;
    requests_manned(r).launch_interval = [0,240];
    requests_manned(r).speed = 20;
    v1 = 891;
    v2 = manned2(randi(3));
    v3 = manned3(randi(2));
    v4 = manned4;
    v5 = manned5;
    leg1 = LEM_get_path(airways_manned,v1,v2);
    leg2 = LEM_get_path(airways_manned,v2,v3);
    leg3 = LEM_get_path(airways_manned,v3,v4);
    leg4 = LEM_get_path(airways_manned,v4,v5);
    path = [leg1,leg2,leg3,leg4];
    requests_manned(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_manned.lanes(path(p),:)];
    end
    requests_manned(r).path_vertexes = vertexes;
    requests_manned(r).launch_index = path(1);
    requests_manned(r).land_index = path(end);
    [fp,reservations_manned] = LEM_reserve_fp(reservations_manned,...
        airways_manned,t_min+1200,t_max,speed,path,hd);
    flights = LEM_add_flight(flights,airways_manned,fp,path,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_MANNED;
    flights(end).team = 2;
    flights(end).delay = 0;
end

%path = [925,6,3,1,87,88,85,189,187,273,271,357,355,441,439,525,523,609,...
%    607,693,691,777,775,860,861,859,867,865,873,871,879,877,885,883,891,...
%    889,897,895,903,901,909,907,915,913,921, 919,920,923,856,857,772,773,...
%    688,689,604,605,520,521,436,437,352,353,268,269,184,185,82,83,84,76,...
%    77,78,68,69,70,60,61,62,52,53,54,44,45,46,36,37,38,28,29,30,20,21,...
%    22,12,13,14,5,6];
UAS1 = [925:934];
UAS2 = [380,410,650];
UAS3 = [860,920];
UAS4 = [80];
UAS5 = [935:944];
for r = 1:num_UAS
    requests_UAS(r).request_time = 0;
    requests_UAS(r).launch_interval = [0,240];
    requests_UAS(r).speed = 20;
    v1 = UAS1(randi(10));
    v2 = UAS2(randi(3));
    v3 = UAS3(randi(2));
    v4 = UAS4;
    v5 = UAS5(randi(10));
    leg1 = LEM_get_path(airways_UAS,v1,v2);
    leg2 = LEM_get_path(airways_UAS,v2,v3);
    leg3 = LEM_get_path(airways_UAS,v3,v4);
    leg4 = LEM_get_path(airways_UAS,v4,v5);
    path = [leg1,leg2,leg3,leg4];
    requests_UAS(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_UAS.lanes(path(p),:)];
    end
    requests_UAS(r).path_vertexes = vertexes;
    requests_UAS(r).launch_index = path(1);
    requests_UAS(r).land_index = path(end);
    [fp,reservations_UAS] = LEM_reserve_fp(reservations_UAS,...
        airways_UAS,t_min+1600,t_max,speed,path,hd);
    flights = LEM_add_flight(flights,airways_UAS,fp,path,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_UAS;
    flights(end).team = 2;
    flights(end).delay = 0;
end

% Generate Coalition Member 3 Requests
path = path_ISR;
hd = 10;
for r = 1:num_ISR
    requests_ISR(r).request_time = 0;
    requests_ISR(r).launch_interval = [0,240];
    requests_ISR(r).speed = 20;
    requests_ISR(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_ISR.lanes(path(p),:)];
    end
    requests_ISR(r).path_vertexes = vertexes;
    requests_ISR(r).launch_index = path(1);
    requests_ISR(r).land_index = path(end);
    [fp,reservations_ISR] = LEM_reserve_fp(reservations_ISR,airways_ISR,...
        t_min+200,t_max,speed,path_ISR,hd);
    flights = LEM_add_flight(flights,airways_ISR,fp,path_ISR,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_ISR;
    flights(end).team = 3;
    flights(end).delay = 0;
end

% manned flights

%path = [891,6,3,1,71,69,155,153,239,237,323,321,407,405,491,489,575,573,...
%    659,657,743,741,826,827,825,833,831,839,837,845,843,851,849,857,...
%    855,863,861,869,867,875,873,881,879,887,885,886,889,890,888,889,...
%    822,823,738,739,654,655,570,571,486,487,402,403,318,319,234,235,150,...
%    151,66,63,64,67,68,61,62,55,56,49,50,43,44,37,38,31,32,25,...
%    26,19,20,12,13,14,5,6];
manned2 = [170,370,650];
manned3 = [825,890];
manned4 = [65];
manned5 = [892];
for r = 1:num_manned
    requests_manned(r).request_time = 0;
    requests_manned(r).launch_interval = [0,240];
    requests_manned(r).speed = 20;
    v1 = 891;
    v2 = manned2(randi(3));
    v3 = manned3(randi(2));
    v4 = manned4;
    v5 = manned5;
    leg1 = LEM_get_path(airways_manned,v1,v2);
    leg2 = LEM_get_path(airways_manned,v2,v3);
    leg3 = LEM_get_path(airways_manned,v3,v4);
    leg4 = LEM_get_path(airways_manned,v4,v5);
    path = [leg1,leg2,leg3,leg4];
    requests_manned(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_manned.lanes(path(p),:)];
    end
    requests_manned(r).path_vertexes = vertexes;
    requests_manned(r).launch_index = path(1);
    requests_manned(r).land_index = path(end);
    [fp,reservations_manned] = LEM_reserve_fp(reservations_manned,...
        airways_manned,t_min+1400,t_max,speed,path,hd);
    flights = LEM_add_flight(flights,airways_manned,fp,path,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_MANNED;
    flights(end).team = 3;
    flights(end).delay = 0;
end

%path = [925,6,3,1,87,88,85,189,187,273,271,357,355,441,439,525,523,609,...
%    607,693,691,777,775,860,861,859,867,865,873,871,879,877,885,883,891,...
%    889,897,895,903,901,909,907,915,913,921, 919,920,923,856,857,772,773,...
%    688,689,604,605,520,521,436,437,352,353,268,269,184,185,82,83,84,76,...
%    77,78,68,69,70,60,61,62,52,53,54,44,45,46,36,37,38,28,29,30,20,21,...
%    22,12,13,14,5,6];
UAS1 = [925:934];
UAS2 = [380,410,650];
UAS3 = [860,920];
UAS4 = [80];
UAS5 = [935:944];
for r = 1:num_UAS
    requests_UAS(r).request_time = 0;
    requests_UAS(r).launch_interval = [0,240];
    requests_UAS(r).speed = 20;
    v1 = UAS1(randi(10));
    v2 = UAS2(randi(3));
    v3 = UAS3(randi(2));
    v4 = UAS4;
    v5 = UAS5(randi(10));
    leg1 = LEM_get_path(airways_UAS,v1,v2);
    leg2 = LEM_get_path(airways_UAS,v2,v3);
    leg3 = LEM_get_path(airways_UAS,v3,v4);
    leg4 = LEM_get_path(airways_UAS,v4,v5);
    path = [leg1,leg2,leg3,leg4];
    requests_UAS(r).path = path;
    len_path = length(path);
    vertexes = [];
    for p = 1:len_path
        vertexes = [vertexes;airways_UAS.lanes(path(p),:)];
    end
    requests_UAS(r).path_vertexes = vertexes;
    requests_UAS(r).launch_index = path(1);
    requests_UAS(r).land_index = path(end);
    [fp,reservations_UAS] = LEM_reserve_fp(reservations_UAS,...
        airways_UAS,t_min+1700,t_max,speed,path,hd);
    flights = LEM_add_flight(flights,airways_UAS,fp,path,vertexes);
    flights(end) = LEM_gen_traj(flights(end),0.1);
    flights(end).id = INDEX_UAS;
    flights(end).team = 3;
    flights(end).delay = 0;
end

% Generate Bogeys
for k = 1:num_bogeys
    x = 3000 + 2000*rand;
    y = 5000*rand;
    z = 0;
    launch_site = [x;y;z];
    land_site = [x;y;z];
    bogey = LEM_rogue_type2(airways_manned,launch_site,land_site,10,20,...
        20,0.1,1);
    bogey(:,4) = bogey(:,4) + 1500;
    flights(end+1).traj = bogey;
    flights(end).team = 4;
    flights(end).id = INDEX_BOGEY;
    flights(end).delay = 0;
end

% Generate artillery
model_ISR = LEM_lanes2model(airways_ISR.lanes,10);
model_manned = LEM_lanes2model(airways_manned.lanes,10);
model_UAS = LEM_lanes2model(airways_UAS.lanes,10);

for k = 1:num_artillery
    [k num_artillery]
    x0 = 1000+ 1000*rand;
    y0 = 1000 + 1000*rand;
    z0 = 0;
    x1 = 3000 + 1000*rand;
    y1 = 2000 + 1000*rand;
    z1 = 0;
    theta = LEM_artillery_angle(x1-x0,y1-y0,z1-z0,827);
    atraj = LEM_artillery_traj(x0,y0,z0,x1,y1,z1,827,theta,0.1);
    time_offset = 2000*rand;
    atraj(:,4) = atraj(:,4) + time_offset;
    atraj = LEM_schedule_artillery(model_ISR,model_manned,model_UAS,...
        airways_ISR,airways_manned,airways_UAS,atraj,theta,hd,t_max,...
        reservations_ISR,reservations_manned,reservations_UAS);
    flights(end+1).traj = atraj;
    flights(end).team = 5;
    flights(end).id = INDEX_ARTILLERY;
    flights(end).pt0 = [x0;y0;z0];
    flights(end).pt1 = [x1;y1;z1];
    flights(end).theta = theta;
    flights(end).delay = atraj(1,4) - time_offset;
end

tch = 0;
