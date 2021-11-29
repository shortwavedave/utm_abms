function [d, n] = getLaneOccupancy(obj, lane_id, t0, tf)
%GETLANEDENSITY Summary of this function goes here
%   Detailed explanation goes here
%     r = Reservations.getReservationTable(obj.reservations);
    r = obj.reservations;
    r_i = find(r.lane_id == lane_id ...
        & r.entry_time_s >= t0 ...
        & r.entry_time_s <= tf);
    n = length(r_i);
    if n > 0
        h_ds = r.hd(r_i);
        speeds = r.speed(r_i);
        h_ts = h_ds ./ speeds;
        d = (sum(h_ts)+h_ts(1))/(tf-t0);
%         h_d = r.hd(1);
%         s = r.speed(1);
%         h_t = h_d/s;
%         max_possible = floor((tf-t0)/h_t);
%         d = n/max_possible;
    else
        d = 0;
    end
end

