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

DIST_THRESH = 4;
MIN_FIT = 0.90;
MIN_REMAINING = 3;

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
index = min(size(x, 1), 20);
[p1,s1] = TrackMonitor.CV_total_LS(x(1:index),y(1:index));

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
index = min(size(indexes, 1), 20);
if(ind1+index > size(x))
    index = size(x)-ind1;
end
if(ind1+index > size(y))
    index = size(y)-ind1;
end
[p2,s2] = TrackMonitor.CV_total_LS(x(ind1:ind1+index),y(ind1:ind1+index));

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
index = min(size(indexes, 1), 20);
index = index(1);
ind1 = indexes(1);
if(ind1+index >= size(x,1))
    index = size(x,1)-ind1;
end
if(ind1+index >= size(y,1))
    index = size(y,1)-ind1;
end
[p3,s3] = TrackMonitor.CV_total_LS(x(ind1:ind1+index),y(ind1:ind1+index));
for p = ind1:num_pts
    d = p3(1)*x(p) + p3(2)*y(p) + p3(3);
    if abs(d)<DIST_THRESH
        segs(p) = 3;
    end
end

% Check each segment labeling/total number of points - if it is less than
% 90 percent, mark as Rogue 1. 
indexes = find(segs);
dif = norm(traj(1, 1:3) - traj(end, 1:3));
if length(indexes)/num_pts>MIN_FIT
    if(dif > 5)
        r1 = 1;
    end
end
