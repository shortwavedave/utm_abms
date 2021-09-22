function lanes = LEM_vertexes2lanes(obj,airways,indexes)
% LEM_vertexes2lanes - create lane sequence from vertex sequence
% On input:
%     airways (airways struct): airway info
%     indexes (1xn vector): list of lane vertexes
% On output:
%     lanes (1x(n-1) vector): list of lane indexes
% Call:
%     lanes = LEM_vertexes2lanes([70,28,24,21,48,49,52,73]);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

vertexes = airways.lane_vertexes;
a_lanes = airways.lanes;
num_indexes = length(indexes);
lanes = zeros(1,num_indexes-1);
for e = 1:num_indexes-1
    pt1 = vertexes(indexes(e),:);
    pt2 = vertexes(indexes(e+1),:);
    index = find(a_lanes(:,1)==pt1(1)&a_lanes(:,2)==pt1(2)...
        &a_lanes(:,3)==pt1(3)...
        &a_lanes(:,4)==pt2(1)&a_lanes(:,5)==pt2(2)&a_lanes(:,6)==pt2(3));
    if length(index)==1
        lanes(e) = index;
    else
        lanes(e) = index(1);
    end
end
