function M = LEM_traj_measures(model,traj,speed,del_t)
% LEM_traj_measures - produce distance and directn measures
% On input:
%     model (model struct): model info
%       .xyzuvw (nx6 array): x,y,z,dx,dy,dz
%       .lane (nx1 vector): lane associated with point
%       .kdt (kdt tree): x,y,z data  [kdt = createns(model.xyzuvw(:,1:3))]
%     traj (kx4 array): x,y,z,t
% On output:
%     M (k-1 x 4 array): traj measures
%       (:,1): distance from traj point to closest model point
%       (:,2): cosine of angle between traj & model directions
%       (:,3): lane change measure (number of bad lane changes)
%       (:,4): lane sequence
%       (:,5): stationary 
%     M = LEM_traj_measures(airways,model,traj);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

DIST_THRESH = speed*del_t;

if isempty(traj)|isempty(model)
    M = [];
    return
end

% traj(:,1) = imfilter(traj(:,1),ones(1,10)/10);
% traj(:,2) = imfilter(traj(:,2),ones(1,10)/10);
% traj(:,3) = imfilter(traj(:,3),ones(1,10)/10);

kdt = model.kdt;
xyzuvw = model.xyzuvw;
t_model = TrackMonitor.LEM_traj2model(traj);
[num_pts,dummy] = size(t_model);
M = zeros(num_pts,2);
lanes = zeros(num_pts,1);
for p = 1:num_pts
    [p num_pts];
    IdxNN = knnsearch(kdt,t_model(p,1:3));
    lanes(p) = model.lane(IdxNN);
    tx = t_model(p,1:3);
    tv = t_model(p,4:6);
    x = xyzuvw(IdxNN,1:3);
    v = xyzuvw(IdxNN,4:6);
    M(p,1) = norm(x-tx);
    if isnan(M(p,1))
        M(p,1) = 10;
    end
    if p<num_pts-5
        dir = traj(p+5,1:3) - traj(p,1:3);
        tv = dir/norm(dir);
    end
    M(p,2) = dot(tv,v)/(norm(tv)*norm(v));
    if isnan(M(p,2))
        M(p,2) = 0;
    end
end

return

% edges = airways.lane_edges;
% errors = 0;
% changes = 0;
% for p = 1:num_pts-1
%     if lanes(p)~=lanes(p+1)
%         changes = changes + 1;
%         index = find(edges(:,1)==lanes(p)&edges(:,2)==lanes(p+1));
%         if isempty(index)
%             errors = errors + 1;
%         end
%     end
% end
% M(:,3) = errors/changes;

return
