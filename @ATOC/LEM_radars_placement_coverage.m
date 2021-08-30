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


% Gathering required information
lane_ids = lbsd.getLaneIds();
vertexes = zeros(length(lane_ids), 3);

for lane = 1:length(lane_id)
    id = lane_id(lane);
    vertexes(lane, :) = lbsd.getVertPositions(id);
end

radius = (mean(vertexes(:,3))*tan(angle));
side = sqrt(2*radius^2);
xmax = max(vertexes(:, 1));
xmin = min(vertexes(:, 1));
ymax = max(vertexes(:, 2));
ymin = min(vertexes(:, 2));
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
        radar(c) = verticalRadar(xcord, ycord, res, angle, range, noise, c);
        c = c + 1;
        radar(c) = horizontalRadar(xcord, ycord, side, angle, range,...
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
    radar(c) = verticalRadar(xmax/2, ymax/2, res, angle, range, noise, c);
    c = c + 1;
    radar(c) = horizontalRadar(xmin, ymax, xmax/2, res, angle, range, ...
        noise, c);
end

    function radar = verticalRadar(xcord, ycord, angle, range, noise, c)
        % verticalRadar - creates radar that are vertically aligned
        % Input -
        %   xcord (float) - x coord
        %   ycord (float) - y coord
        %   res (results struct): results info
        %      .roads (roads struct): roads info
        %      .airways (airways struct): airways info
        %      .requests (requests struct): LBSD requests
        %      .reservations (reservations struct): reservations info
        %      .flights (flights struct): flights info
        %      .failures_LBSD (n by 1 vector): 0 if LBSD flight success; else 1
        %   range (float): the maximum range the radar field can have
        %   noise (3x3): noise that each radar will have
        %   angle (float): the radian degree from middle of radar field to the
        %        edge.
        %   c (float) - number of radar
        radar(c).x = xcord;
        radar(c).y = ycord;
        radar(c).z = min(vertexes(:,3));
        radar(c).dx = 0;
        radar(c).dy = 0;
        radar(c).dz = 1;
        radar(c).phi = angle;
        radar(c).max_range = range;
        radar(c).id = c;
        radar(c).pos_noise = noise;
    end
    function radar = horizontalRadar(xcord, ycord,side, angle, range, noise, c)
        % horizontalRadar - creates radar that are horizontal aligned
        % Input -
        %   xcord (float) - x coord
        %   ycord (float) - y coord
        %   side (float) - side length of the radar
        %   res (results struct): results info
        %      .roads (roads struct): roads info
        %      .airways (airways struct): airways info
        %      .requests (requests struct): LBSD requests
        %      .reservations (reservations struct): reservations info
        %      .flights (flights struct): flights info
        %      .failures_LBSD (n by 1 vector): 0 if LBSD flight success; else 1
        %   range (float): the maximum range the radar field can have
        %   noise (3x3): noise that each radar will have
        %   angle (float): the radian degree from middle of radar field to the
        %        edge.
        %   c (float) - number of radar
        % Radar that is 45 degrees off the horizontal line in pos x dir
        radar(c).x = xcord - side/2;
        radar(c).y = ycord - side/2;
        radar(c).z = min(vertexes(:,3));
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
