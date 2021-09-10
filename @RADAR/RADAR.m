classdef RADAR < handle
    %RADAR An instance of this class represents a radar within the
    %   simulation. It's main purpose it to detect objects that are within
    %   its own radar field.
    
    properties
        location % [x; y; z coordinates]
        targets % Most recent targets detected
        range % Range of the Radar
        apexAngle % apexAngle divided by 2
        dirVector % [dx, dy, dz] direction cordinates
        time % Time
        ID % Radar Identification number
        sim % handle to the simulator
        detection_listers % A list of those listening to radar class
    end
    
    methods
        
        function obj = RADAR(pos, range, angle, dir, ID)
            % Radar - constructor to Create Radar Objects
            % Input:
            %   pos (3x1): x, y, z coordinates for radar location
            %   range (float): maximum distance the radar can cover
            %   angle (float): half of the apex angle
            %   dir (3 x 1): directional vector where the radar is pointing
            % Call:
            %   radar = RADAR([0;0;0], 500, pi/4, [0,0,1], "1");
            obj.location = pos;
            obj.range = range;
            obj.apexAngle = angle;
            obj.dirVector = dir;
            obj.ID = ID;
        end
        
        function scan(obj,UAS)
            % scan - scans the environment to be able to detect objects
            % Input:
            %   UAS (UAS bjects): objects that are within the environment
            %       .gps (struct): Gives locaitonal informaiton
            num_targets = 0;
            obj.targets = [];
            for object = 1:length(UAS)
                uas_v = obj.location - [UAS(object).gps.lon; ...
                    UAS(object).gps.lat; UAS(object).gps.alt];
                dist_v = norm(uas_v);
                unit_v = v/dist_v;
                alpha = posori(acos(dot(unit_v, obj.dirVector)));
                noise = mvnrnd(mu, eye(3));
                if(alpha <= obj.apexAngle && dist_v <= obj.range)
                    num_targets = num_targets + 1;
                    num_targets = num_targets + 1;
                    targets(num_targets).x = obj.location(1) + noise(1);
                    targets(num_targets).y = obj.location(2) + noise(2);
                    targets(num_targets).z = obj.location(3) + noise(3);
                    targets(num_targets).s = UAS(object).nominal_speed...
                        + rand;
                    %targets(num_targets).diam = ksize + rand;
                    targets(num_targets).time = obj.time;
                    targets(num_targets).id = obj.ID;
                end
            end
            if (~isempty(objects.targets))
                notify(obj, 'detection');
            end
        end
        
        function subscribeToDetection(obj, subscriber)
        % subscribeToDetection - sets an event listener to trigger when 
        %   the radar detects an object
        % Input:
        %   obj - an instance of the radar class
        %   subscriber - a function hangle to trigger
            lh = obj.addlistener('detection', subscriber);
            obj.detection_listers = [obj.detection_listers, lh];
        end
    end
end
    
