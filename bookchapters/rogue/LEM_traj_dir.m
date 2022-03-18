function d = LEM_traj_dir(traj);
% LEM_traj_dir - produce traj direction changes
% On input:
%     traj (kx4 array): x,y,z,t
% On output:
%     d (kx1 vector): angle difference between moves
% Call:
%     d = LEM_traj_dir(traj);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if isempty(traj)
    dir = [];
    return
end

t_model = LEM_traj2model(traj);
[num_pts,dummy] = size(t_model);
d = zeros(num_pts,1);
for p = 1:num_pts-1
    tv1 = t_model(p,4:6);
    tv2 = t_model(p+1,4:6);
    d(p) = dot(tv1,tv2)/(norm(tv1)*norm(tv2));
end
