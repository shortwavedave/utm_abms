function [isHobbist] = LEM_Classification_HobbistTwo(traj)
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
Hover_DIST_Thresh = .08;

[num_pts, ~] = size(traj);
errors = 10*ones(num_pts, 1);
for p = 1:5:num_pts-5
    pts = traj(p:p+5,1:3);
    ptsm = mean(pts);
    e = zeros(5,1);

    for k = 1:5
        dx = pts(k,1) - ptsm(1); % del_x = actual - mean
        dy = pts(k,2) - ptsm(2); % del_y = actual - mean
        dz = pts(k,3) - ptsm(3); % del_z = actual - mean
        e(k) = sqrt(dx^2+dy^2+dz^2); % Euclidean distance from mean
    end
    errors(p) = max(e);
    indexes = find(errors < Hover_DIST_Thresh, 1);
    if ~isempty(indexes)
        isHobbist = true;
    end
end
