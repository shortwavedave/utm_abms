function [flight_plan,reservations] = LEM_reserve_fp_rogue(obj,reservations,...
    lane_lengths,t1,t2,speed,path,hd)
% LEM_reserve_fp - find flight plan, if possible
% On input:
%     reservations (reservations struct): flight reservation info
%       (k).flights (n_k x 4 array)  [flight_id t_in t_out speed]
%     airways (airway struct): airway info
%     t1 (float): initial possible launch time
%     t2 (float): final possible launch time
%     speed (float): request vehicle's speed
%     path (kx1 vector): lane indexes
%     hd (float): headway distance
% On output:
%     flight_plan (nx4 array): flight plan
%       (i,:): [ti1, ti2, speed, lane_i]
%     reservations (reservations struct): updated reservation info
% Call:
%     [fp,reservations] = LEM_reserve_fp(reservations,airways1,1,30,p,10);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

flight_plan = [];
ht = hd/speed;

len_path = length(path);

possible0 = [t1,t2];

%get possible start inteval (full interval for first flight)
possible = obj.LEM_LSD_rogue(possible0,speed,path,lane_lengths,...
    reservations,ht);

if isempty(possible)
    return
end

t_start = possible;

flight_plan = zeros(len_path,4);
t1 = t_start(1);
for c = 1:len_path
    flight_plan(c,1) = t1;
    t2 = t1 + lane_lengths(c)/speed;
    flight_plan(c,2) = t2;
    flight_plan(c,3) = speed;
    flight_plan(c,4) = path(c);
    t1 = t2;
end

for c = 1:len_path
    reservations(path(c)).flights = [reservations(path(c)).flights;...
        -1,flight_plan(c,1:3)];
    flights = reservations(path(c)).flights;
    [vals,indexes] = sort(flights(:,2));
    flights = flights(indexes,:);
    reservations(path(c)).flights = flights;
end

tch = 0;
