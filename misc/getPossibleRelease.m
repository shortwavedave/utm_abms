function [r2_min,r2_max] = getPossibleRelease(v1,r1,hd1,hd2,v2)
%GETPOSSIBLERELEASE Summary of this function goes here
%   Detailed explanation goes here
    r2_min = ((v1*r1)-(v1-v2)*max(hd1,hd2)^2)/v2;
    r2_max = ((v1*r1)+(v1-v2)*max(hd1,hd2)^2)/v2;
end

