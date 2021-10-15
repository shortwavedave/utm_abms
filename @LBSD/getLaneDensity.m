function [d, n] = getLaneDensity(obj, lane_id, t0, tf)
%GETLANEDENSITY Summary of this function goes here
%   Detailed explanation goes here
r = obj.reservations(...
    obj.reservations.lane_id == lane_id ...
    & obj.reservations.entry_time_s >= t0 ...
    & obj.reservations.entry_time_s <= tf, :);
n = height(r);
d = n/(tf-t0);
end

