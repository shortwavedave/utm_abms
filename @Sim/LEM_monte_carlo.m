function [coverage,pts] = LEM_monte_carlo(lbsd, radars, num_samples)
% LEM_monte_carlo - estimates the total coverage of radars field.
% On Input:
%   airways (airway struct): Airway information
%   radars (radar Struct): Radar Information
% On Output:
%   coverage (float): the estimate percentage of radar coverage
% On Call:
%   coverage = LEM_monte_carlos(airways, radars);
% Author
%   Vista Marston & Thomas Henderson
%   UU
%   Summer 2021

lane_verts = lbsd.getVertPositions(':');
pts = zeros(num_samples,3);
hits = 0;
xmin = min(lane_verts(:,1));
xmax = max(lane_verts(:,1));
ymin = min(lane_verts(:, 2));
ymax = max(lane_verts(:, 2));
zmin = min(lane_verts(:, 3));
zmax = max(lane_verts(:, 3));
num_radars = length(radars);

for s = 1:num_samples
    x = xmin + rand*(xmax - xmin);
    y = ymin + rand*(ymax - ymin);
    z = zmin + rand*(zmax - zmin);
    for r = 1:num_radars
        w = [radars(r).dirVector(1), radars(r).dirVector(2), radars(r).dirVector(3)];
        r_pos = [radars(r).location(1), radars(r).location(2), radars(r).location(3)];
        range = radars(r).range;
        v = [x, y, z] - r_pos;
        dist = norm(v);
        uv = v / dist;
        alpha = Sim.LEM_posori(acos(dot(uv, w)));
        if alpha <= radars(r).apexAngle && dist < range
            hits = hits + 1;
            pts(hits,:) = [x,y,z];
            break;
        end
    end
end
coverage = hits/num_samples;
pts(hits+1:num_samples,:) = [];
