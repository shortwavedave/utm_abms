function c = LEM_NAB_classify(airways,model,traj,speed,del_t,traj_len)
%

DIST_THRESH = 1;
COS_THRESH = 0.8;
HOVER_DIST_THRESH = 0.08;
FLAT_THRESH = 0.2;
CIRCLE_THRESH = 0.9;
LL_DIST = 2;

% Default is Rogue 2.
c = 6;

M = LEM_traj_measures(airways,model,traj,speed,del_t);
M1 = imresize(M(:,1),[traj_len,1])';
M2 = imresize(M(:,2),[traj_len,1])';

% Check if the flight is normal
if median(M1)<DIST_THRESH & median(M2)>COS_THRESH
    c = 1;
    return
end

% Hobby 2: hovers
[num_pts,~] = size(traj);
errors = 10*ones(num_pts,1);
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

indexes = find(errors<HOVER_DIST_THRESH);
if ~isempty(indexes)
    c = 3;
    return
end

% Rogue 1
if LEM_check_rogue1(traj)
    c = 5;
    return
end

% Hobby 1
% diff - Difference and Approximate derivations
% abs - absolute value of the differences of the z values
% length - Length of largest array dimensions
s = sum(abs(diff(traj(1:20:num_pts,3)))<.1);
len = length(diff(traj(1:20:num_pts,3)));

% sum differences/size of differences in z cordinates
if s/len<FLAT_THRESH
    c = 2;
end

% Hobby 3
curv = zeros(num_pts,1);
dists = zeros(num_pts,1);
indexes = find(max(traj(:,1))==traj(:,1));
p1 = [traj(indexes(1),1:2),0];
indexes = find(max(traj(:,2))==traj(:,2));
p2 = [traj(indexes(1),1:2),0];
indexes = find(min(traj(:,1))==traj(:,1));
p3 = [traj(indexes(1),1:2),0];
[radius,center] = LEM_3pts2circle(p1,p2,p3);
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
    c = 4;
end

tch = 0;
