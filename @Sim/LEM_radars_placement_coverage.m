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

if(max_lane_height == 0)
    max_lane_height = range;
end

radius = max_lane_height*tan(angle);
side = sqrt(2*radius^2)/2;
radar = createsRadarStruct();

c = 1;

ycord = min(lane_verts(:,2));
xcord = min(lane_verts(:,1));

if(ycord == ymax)
    ymax = ycord + side;
end

if(xcord == xmax)
    xmax = xcord + side;
end

% Placing Radars
while xcord <= xmax
    while ycord < ymax
        % Vertical Radar dir = [0,0,1]
        c = placeRadars([xcord, ycord, 0], c);
        ycord = ycord + side;
    end
    xcord = xcord + side;
    ycord = min(lane_verts(:,2));
end

if (isempty(fieldnames(radar)))
    x = min(lane_verts(:,1)) + ...
        max(min(lane_verts(:,1)) - min(lane_verts(:,1)));
    y = min(lane_verts(:,2)) + ...
        max(min(lane_verts(:,2)) - min(lane_verts(:,2)));
    placeRadars([x,y,0], 1);
end

    function radar = createsRadarStruct()
    % createsRadarStruct - Sets up the radar struct.
    % Output:
    %   radar (struct): radar struct
        xmin = min(lane_verts(:,1));
        xmax = max(lane_verts(:, 1));
        ymin = min(lane_verts(:,2));
        ymax = max(lane_verts(:, 2));
        yrange = ymax - ymin;
        xrange =  xmax - xmin;
        num_radar = ceil((xrange*yrange)/(side^2));
        radar(num_radar) = struct();
    end
    function c = verticalRadar(pos, zcord, angle, range, noise, c)
        % verticalRadar - creates radar that are vertically aligned
        % Input -
        %   pos (1x2): x and y coordinates
        %   zcord (float): z coordinates
        %   range (float): the maximum range the radar field can have
        %   noise (3x3): noise that each radar will have
        %   angle (float): the radian degree from middle of radar field to the
        %        edge.
        %   c (float) - number of radar + 1
        % Output:
        %   c (float) - number of radars + 1
        radar(c).x = pos(1);
        radar(c).y = pos(2);
        radar(c).z = zcord;
        radar(c).dx = 0;
        radar(c).dy = 0;
        radar(c).dz = 1;
        radar(c).phi = angle;
        radar(c).max_range = range;
        radar(c).id = num2str(c);
        radar(c).pos_noise = noise;
        c = c + 1;
    end
    function c = horizontalRadar(pos, zcord, uv, angle, range, noise, c)
        % horizontalRadar - creates radar that are horizontal aligned
        % Input -
        %   pos (1x2): x and y cordinates
        %   zcord (float): z cordinates
        %   uv (1 x 3): directional vector for radar field
        %   range (float): the maximum range the radar field can have
        %   noise (3x3): noise that each radar will have
        %   angle (float): the radian degree from middle of radar field to the
        %        edge.
        %   c (float) - number of radar + 1
        % Output:
        %   c (float): number of radars + 1
        % Radar that is 45 degrees off the horizontal line in pos x dir
        radar(c).x = pos(1);
        radar(c).y = pos(2);
        radar(c).z = zcord;
        radar(c).dx = uv(1);
        radar(c).dy = uv(2);
        radar(c).dz = uv(3);
        radar(c).phi = angle;
        radar(c).max_range = range;
        radar(c).id = num2str(c);
        radar(c).pos_noise = noise;
        c = c + 1;
    end
    function c = placeRadars(pos, c)
        % placeRadars - adds radars into the radar struct pointing in all
        %   directions.
        % Input:
        %   pos (1 x 3): The position to place the radar
        %   c (float): current number of radars placed + 1;
        % Output:
        %   c (float): Current number of radars placed + 1;
        if(~alreadyInPlace(pos))
            c = verticalRadar(pos(1:2), 0, angle, range, noise, c);
            
            % +/-X Direction, +/-Y Direction
%             c = horizontalRadar(pos(1:2), 0, [1, 0, 0], angle, range, noise, c);
%             c = horizontalRadar(pos(1:2), 0, [-1, 0, 0], angle, range, noise, c);
%             c = horizontalRadar(pos(1:2), 0, [0, 1, 0], angle, range, noise, c);
%             c = horizontalRadar(pos(1:2), 0, [0, -1, 0], angle, range, noise, c);
        end
    end
    function exists = alreadyInPlace(pos)
        % alreadyInPlace: Checks whether a radar has already been placed in the
        %   specific coorindate.
        % Input:
        %   pos (1x3): x, y, z coordinates to place radar
        % Output:
        %   exists (bool) : 0 if there isn't a radar in the current pos.
        %   Otherwise 1.
        if(~isempty(fieldnames(radar)))
            x = transpose([radar(:).x]);
            y = transpose([radar(:).y]);
            curPos = [x, y];
            exists = ismember(pos(1:2), curPos, 'rows');
        else
            exists = 0;
        end
    end

end
