function [r1, r2] = LEM_check_rogue(traj)
% LEM_check_rogue1 - checks if the trajectory falls under the
%    classification of rogue 1. 
% Input:
%   traj (nx4): x, y, z, t
% Output:
%   b (boolean): Indicates if the trajectory is Rogue 1
% Call
%   b = LEM_check_rogue1(traj)
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

DIST_THRESH = 2;
MIN_FIT = 0.90;
MIN_REMAINING = 20;

% Default is false
r1 = 0;
r2 = 0;

% Requires a certain amount of trajectory information
[num_pts,~] = size(traj);
if num_pts<3*MIN_REMAINING
    return
end

% Find the x spread and y spread 
x_spread = max(traj(:,1)) - min(traj(:,1));
y_spread = max(traj(:,2)) - min(traj(:,2));
y = traj(:,3);

% grab the maximum spread
if x_spread>y_spread
    x = traj(:,1);
else
    x = traj(:,2);
end

segs = zeros(num_pts,1);
% Calculates the total least squares between the x/y and z
% p1 - Coefficients of best fit line ax + by + c = 0
% s1 - error measure
[p1,s1] = TrackMonitor.CV_total_LS(x(1:20),y(1:20));

for p = 1:num_pts
    % calculate the d based on p1. 
    d = p1(1)*x(p) + p1(2)*y(p) + p1(3);
    % If the predicted value is less than 2
    if abs(d)<DIST_THRESH
        segs(p) = 1;
    end
end

% Find any segments that are still zero
indexes = find(segs==0);

% Not Rogue 1 Flight
if length(indexes)<MIN_REMAINING
    return
end

% Grab the first zero segment
ind1 = indexes(1);
% Calculates the total least squares between the x/y and z
% p2 - Coefficients of best fit line ax + by + c = 0
% s2 - error measure
[p2,s2] = TrackMonitor.CV_total_LS(x(ind1:ind1+20),y(ind1:ind1+20));

for p = ind1:num_pts
    d = p2(1)*x(p) + p2(2)*y(p) + p2(3);
    if abs(d)<DIST_THRESH
        segs(p) = 2;
    end
end

% Not Rogue 2 Flight - segment 2
indexes = find(segs==0);
if length(indexes)<MIN_REMAINING
    return
end

% Check for segment 3
ind1 = indexes(1);
[p3,s3] = TrackMonitor.CV_total_LS(x(ind1:ind1+20),y(ind1:ind1+20));
for p = ind1:num_pts
    d = p3(1)*x(p) + p3(2)*y(p) + p3(3);
    if abs(d)<DIST_THRESH
        segs(p) = 3;
    end
end

% Check each segment labeling/total number of points - if it is less than
% 90 percent, mark as Rogue 1. 
indexes = find(segs);
if length(indexes)/num_pts>MIN_FIT
    r1 = 1;
end
