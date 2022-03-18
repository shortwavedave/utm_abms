function b = LEM_lane_OK(reservations,lanes,lane_index,traj_pt,hd)
%

b = 1;

x = traj_pt(1:3);
t = traj_pt(4);
flights = reservations(lane_index).flights;
if isempty(flights)
    return
end
indexes = find(flights(:,2)<=t&t<=flights(:,3));
if isempty(indexes)
    return
end
e1 = lanes(lane_index,1:3);
e2 = lanes(lane_index,4:6);
lane_length = norm(e2-e1);
dir = e2 - e1;
dir = dir/norm(dir);

for k = 1:length(indexes)
    f_index = indexes(k);
    t1 = flights(f_index,2);
    t2 = flights(f_index,3);
    speed = flights(f_index,4);
    fx = e1 + (t-t1)*speed*dir;
    if norm(fx-x)<hd
        b = 0;
        return
    end
end
