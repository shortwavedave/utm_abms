function model = LEM_lanes2model(lanes,del_x)
% LEM_lanes2model - make a trajectory model for a set of air lanes
% On input:
%     lanes (nx6 array): x1,y1,z1,x2,y2,z2 lane endpoints
%     del_x (float): spacing for sample points along lanes
% On ouput:
%     model (modelstruct): model info
%       .xyzvwu (nx6 array): lane sample points & dirs
%       .lane (nx1 vector): lanes associated with points
%       .kdt (kdt tree): lane sample points in kd-tree
% Call:
%     model3 = LEM_lanes2model(airways.lanes,2);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

% Declare a model
model = [];

% Number of lanes
[num_lanes,dummy] = size(lanes);
if num_lanes<1
    return
end

% Lane lengths
lane_lengths = zeros(num_lanes,1);
for k = 1:num_lanes
    lane_lengths(k) = norm(lanes(k,4:6)-lanes(k,1:3));
end

total_length = sum(lane_lengths);
num_samples = ceil(total_length/del_x);
model = zeros(num_samples,7);
count = 0;

% Loop through all of the lanes
for k = 1:num_lanes-1
    % grab the two end points
    pt1 = lanes(k,1:3);
    pt2 = lanes(k,4:6);
    % Del_x
    dx = pt2 - pt1;
    dist = norm(dx);
    dir = dx/dist;
    d = 0; pt = pt1; x = []; y = []; z = [];
    % While current dis less than total change
    while d<=dist
        % r = ro + del_x * direction vector
        pt = pt + del_x*dir;
        % update current distance
        d = norm(pt1-pt);
        % Update the x,y,z of the model
        if d<=dist
            x = [x,pt(1)];
            y = [y,pt(2)];
            z = [z,pt(3)];
        end
    end
    % Update the current num_pts
    num_pts = length(x);
    % Add the points to the model
    for p = 1:num_pts
        count = count + 1;
        model(count,1:6) = [x(p),y(p),z(p),dir(1),dir(2),dir(3)];
        model(count,7) = k;
        tch = 0;
    end
    tch = 0;
end
% Updates the model
model = model(1:count,:);
model1.xyzuvw = model(:,1:6);
model1.lane = model(:,7);
% Create the KD-Tree
kdt = createns(model(:,1:3));
model1.kdt = kdt;
model = model1;

tch = 0;
