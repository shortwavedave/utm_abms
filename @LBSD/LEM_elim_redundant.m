function pts_out = LEM_elim_redundant(obj,pts)
% LEM_elim_redundant - produce unique set of points
% On input:
%     pts (nx3 array): 3D points
% On output:
%    pts_out (mx3 array): unique point set
% Call:
%    pts2 = LEM_elim_redundant(lane_vertexes);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

num_pts = length(pts(:,1));
redundant = zeros(num_pts,1);

for p1 = 1:num_pts-1
    if redundant(p1)==0
        for p2 = p1+1:num_pts
            if norm(pts(p1,:)-pts(p2,:))==0
                redundant(p2) = 1;
            end
        end
    end
end

indexes = find(redundant==0);
pts_out = pts(indexes,:);
