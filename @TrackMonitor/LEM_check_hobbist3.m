function HobbistThree = LEM_check_hobbist3(traj)
% LEM_CLASSIFICATION_HOBBISTTHREE Checks to see if the trajectory falls
%   under the classification of hobbist three. 
% Input:
%   traj (nx4): x, y, z, t
% Output:
%   HobbistThree (boolean): Indicates if the trajectory is Hobbist three
% Author:
%     T. Henderson
%     UU
%     Spring 2021
% 

% Initializes variables
CIRCLE_THRESH = 0.9;
[num_pts, ~] = size(traj);
HobbistThree = false;
curv = zeros(num_pts,1);
dists = zeros(num_pts,1);
indexes = find(max(traj(:,1))==traj(:,1));
p1 = [traj(indexes(1),1:2),0];
indexes = find(max(traj(:,2))==traj(:,2));
p2 = [traj(indexes(1),1:2),0];
indexes = find(min(traj(:,1))==traj(:,1));
p3 = [traj(indexes(1),1:2),0];
[radius,center] = TrackMonitor.LEM_3pts2circle(p1,p2,p3);

for p = 1:num_pts-6
%    p1 = [traj(p,1:2),0];
%    p2 = [traj(p+3,1:2),0];
%    p3 = [traj(p+6,1:2),0];
%    radius = (1/2)*(norm(p1-p2)*norm(p2-p3)*norm(p3-p1))...
%        /norm(cross((p1-p2),(p2-p3)));
    curv(p) = 1/radius;
    dists(p) = norm(traj(p,1:2)-center);
end

n = length(find(dists-median(dists)<1));
if n/length(dists)>CIRCLE_THRESH
    HobbistThree = true;
end

end

