function [isHobbist] = LEM_check_hobbist2(traj)
%LEM_CLASSIFICATION_HOBBISTTWO Summary of this function goes here
%   Detailed explanation goes here
% HobbistTwoCheck - Checks if the trajectory is a hobbist type
% 2 or not.
% Input:
%   traj (nx4 array): x,y,z,t
% Output:
%   isHobbist (boolean): Indicates if this is the traj is a
%   Hobbist Type 2.
% Call:
%   isHobbist = HobbistTwoCheck(traj)
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

isHobbist = false;
Hover_DIST_Thresh = .8;

[num_pts, ~] = size(traj);
errors = 10*ones(num_pts, 1);
for p = 6:num_pts-5
    pts = traj(p-5:p+5,1:3);
    ptsm = mean(pts);
    e = zeros(11,1);
    for k = 1:11
        dx = pts(k,1) - ptsm(1);
        dy = pts(k,2) - ptsm(2);
        dz = pts(k,3) - ptsm(3);
        e(k) = sqrt(dx^2+dy^2+dz^2);
    end
    errors(p) = max(e);
end

indexes = find(errors < Hover_DIST_Thresh, 1);

if ~isempty(indexes)
    isHobbist = true;
end
