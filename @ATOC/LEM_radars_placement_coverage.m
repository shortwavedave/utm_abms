function radar = LEM_radars_placement_coverage(lbsd,range, noise, angle)
% LEM_radars_placement_coverage - creates a number of radars to cover
%       a given area and calculates the amount of coverage the radar field
%       has.
% On Input:
%     lbsd (LSBD handle): Lane Based information
%     range (float): the maximum range the radar field can have
%     noise (3x3): noise that each radar will have
%     angle (float): the radian degree from middle of radar field to the
%        edge.
% On Output
%     radars (radar struct): radar information
%       (k).x (float): x coord of radar
%       (k).y (float): y coord of radar
%       (k).z (float): z coord of radar
%       (k).dx (float): unit vector of the radar x position
%       (k).dy (float): unit vector of the radar y position
%       (k).dz (float): unit vector of the radar z position
%       (k).phi (float): the angle from the middle to the end of the radar
%               field
%       (k).max_range (float): the maximum range the radar field has
%       (k).id (float): radar identification number
%       (k).pos_noise (3x3): noise within the data
% On Call:
%      radar = LEM_radars_placement_coverage(lbsd, 500, eye(3), pi/4)
% Author
%     Vista Marston
%     UU
%     Summer 2021

lane_verts = lbsd.getVertPositions(':');
max_lane_height = max(lane_verts(:,3));
radius = max_lane_height*tan(angle);
side = sqrt(2*radius^2);
xmax = max(lane_verts(:, 1));
xmin = min(lane_verts(:, 1));
ymax = max(lane_verts(:, 2));
ymin = min(lane_verts(:, 2));
yrange = ymax - ymin;
xrange =  xmax - xmin;
num_radar = ceil((xrange*yrange)/(side^2));
radar(num_radar) = struct();
c = 1;
ycord = ymin + side/2;
xcord = xmin + side/2;

% Placing Radars
while xcord < xmax
    while ycord < ymax
        % Vertical Radar dir = [0,0,1]
        verticalRadar(xcord, ycord, lane_verts, angle, range, noise, c);
        c = c + 1;
        horizontalRadar(xcord, ycord, side, lane_verts, angle, range,...
            noise, c);
        % Update Position
        c = c + 1;
        ycord = ycord + side;
    end
    xcord = xcord + side;
    ycord = ymin + side/2;
end

if (isempty(fieldnames(radar)))
    % Vertical Radar dir = [0,0,1]
    verticalRadar(xmax/2, ymax/2, lane_verts, angle, range, noise, c);
    c = c + 1;
    horizontalRadar(xmin, ymax, xmax/2, lane_verts, angle, range, ...
        noise, c, radar);
end

    function verticalRadar(xcord, ycord, lane_vertexes, angle, range, noise, c)
        % verticalRadar - creates radar that are vertically aligned
        % Input -
        %   xcord (float) - x coord
        %   ycord (float) - y coord
        %   lane_vertexes (nx3) - coordinates of lane vertexes [x,y,z]
        %   range (float): the maximum range the radar field can have
        %   noise (3x3): noise that each radar will have
        %   angle (float): the radian degree from middle of radar field to the
        %        edge.
        %   c (float) - number of radar
        radar(c).x = xcord;
        radar(c).y = ycord;
        % Min - grabs the ground of where the UAS are flying from
        radar(c).z = min(lane_verts(:,3));
        radar(c).dx = 0;
        radar(c).dy = 0;
        radar(c).dz = 1;
        radar(c).phi = angle;
        radar(c).max_range = range;
        radar(c).id = c;
        radar(c).pos_noise = noise;
    end
    function horizontalRadar(xcord, ycord,side, lane_vertexes, angle, range, noise, c)
        % horizontalRadar - creates radar that are horizontal aligned
        % Input -
        %   xcord (float) - x coord
        %   ycord (float) - y coord
        %   side (float) - side length of the radar
        %   lane_vertexes (nx3) - coordinates of lane vertexes [x,y,z]
        %   range (float): the maximum range the radar field can have
        %   noise (3x3): noise that each radar will have
        %   angle (float): the radian degree from middle of radar field to the
        %        edge.
        %   c (float) - number of radar
        % Radar that is 45 degrees off the horizontal line in pos x dir
        radar(c).x = xcord - side/2;
        radar(c).y = ycord - side/2;
        radar(c).z = min(lane_verts(:,3));
        x = radar(c).x + range*cosd(45);
        z = radar(c).z + range*sind(45);
        v = [x, radar(c).y, z] - [radar(c).x, radar(c).y, radar(c).z];
        uv = v/norm(v);
        radar(c).dx = uv(1);
        radar(c).dy = uv(2);
        radar(c).dz = uv(3);
        radar(c).phi = angle;
        radar(c).max_range = range;
        radar(c).id = c;
        radar(c).pos_noise = noise;
    end
end
