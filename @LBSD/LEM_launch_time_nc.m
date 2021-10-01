function t = LEM_launch_time_nc(obj,reservations,path,t1,t2,lane_lengths,...
    hd,speed)
% LEM_gen_reservations - generate new lane reservations
% On input:
%     reservations (reservations struct): lane reservations
%     path (nx2 array): lane index, lane speed for path
%     t1 (float): min possible launch time
%     t2 (float): max possible launch time
%     lane_lengths (nx1 vector): lengths of lanes
%     hd (float): headway distance
% On output:
%     t (float): assigned lauch time
% Call:
%     t = LEM_launch_time(reservations,path,1,31,lengths,6);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%     modified Fall 2021
%

t = [];

[len_path,~] = size(path);
if len_path<1
    return
end

excluded = [];
d = 0;

for k = 1:len_path
    lane_number = path(k,1);
    lane_len = lane_lengths(lane_number);
    flights = reservations(lane_number).flights;
    [num_flights,~] = size(flights);
    for f = 1:num_flights
        f_t1 = flights(f,2);
        f_t2 = flights(f,3);
        f_speed = flights(f,4);
        ht = max(hd/speed,hd/f_speed);
        f11 = f_t1 - ht;
        f21 = f_t2 - ht;
        f12 = f_t1 + ht;
        f22 = f_t2 + ht;
        if speed>f_speed
            e1 = f11;
            e2 = f22 - lane_len/speed;
        elseif speed<f_speed
            e1 = f12 - lane_len/speed;
            e2 = f21;
        else
            e1 = f11;
            e2 = f21;
        end
        b1 = e1;
        b2 = e2;
        for k2 = k-1:-1:1
            b1 = b1 - lane_lengths(k2)/path(k2,2);
            b2 = b2 - lane_lengths(k2)/path(k2,2);
        end
        if ~(b2<=t1|t2<=b1) % overlaps with [t1,t2]
            excluded = LBSD.LEM_excluded(obj,excluded,t1,t2,b1,b2);
        end
    end
end

if isempty(excluded)
    t = t1;
else
    [num_int,~] = size(excluded);
    for k = 1:num_int
        int1 = excluded(k,:);
        e_min = min(int1);
        e_max = max(int1);
        if e_min<=t1&e_max>=t2
            t = [];
        elseif e_min<=t1&e_max<t2
            t = e_max;
        else
            t = t1;
        end
        if ~isempty(t)
            return
        end
    end
end

tch = 0;
