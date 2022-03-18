function b = LEM_check_rogue1(traj)
%

DIST_THRESH = 2;
MIN_FIT = 0.90;
MIN_REMAINING = 20;

b = 0;

[num_pts,~] = size(traj);
if num_pts<3*MIN_REMAINING
    return
end

x_spread = max(traj(:,1)) - min(traj(:,1));
y_spread = max(traj(:,2)) - min(traj(:,2));
y = traj(:,3);
if x_spread>y_spread
    x = traj(:,1);
else
    x = traj(:,2);
end

segs = zeros(num_pts,1);
[p1,s1] = CV_total_LS(x(1:20),y(1:20));
for p = 1:num_pts
    d = p1(1)*x(p) + p1(2)*y(p) + p1(3);
    if abs(d)<DIST_THRESH
        segs(p) = 1;
    end
end

indexes = find(segs==0);
if length(indexes)<MIN_REMAINING
    return
end
ind1 = indexes(1);
[p2,s2] = CV_total_LS(x(ind1:ind1+20),y(ind1:ind1+20));
for p = ind1:num_pts
    d = p2(1)*x(p) + p2(2)*y(p) + p2(3);
    if abs(d)<DIST_THRESH
        segs(p) = 2;
    end
end

indexes = find(segs==0);
if length(indexes)<MIN_REMAINING
    return
end
ind1 = indexes(1);
[p3,s3] = CV_total_LS(x(ind1:ind1+20),y(ind1:ind1+20));
for p = ind1:num_pts
    d = p3(1)*x(p) + p3(2)*y(p) + p3(3);
    if abs(d)<DIST_THRESH
        segs(p) = 3;
    end
end
indexes = find(segs);
if length(indexes)/num_pts>MIN_FIT
    b = 1;
end
