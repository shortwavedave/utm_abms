function route = LEM_plan2route(obj,plan,airways)
% LEM_plan2route - convert plan to route data
% On input:
%     plan (plan data): plan data
%     airways (airways struct): airways info
% On output:
%     route (nx9 array): route data: [x1 y1 z1 x2 y2 z2 t1 t2 speed]
% Call: 
%     r = LEM_plan2route(p,airways1);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

num_steps = length(plan(:,1));
route = zeros(num_steps,9);

for s = 1:num_steps
    t1 = plan(s,1);
    t2 = plan(s,2);
    speed = plan(s,3);
    lane_index = plan(s,4);
    v1 = airways.lane_edges(lane_index,1);
    pt1 = airways.lane_vertexes(v1,:);
    v2 = airways.lane_edges(lane_index,2);
    pt2 = airways.lane_vertexes(v2,:);
    route(s,:) = [pt1 pt2 t1 t2 speed];
end
