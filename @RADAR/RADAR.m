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
        lbsd % handle to the lane system
        detection_listers % A list of those listening to radar class
        graph % Boolean of whether to display the graph
        lanesCovered = []; % holds the lanes that are covered by the radar
    end
    
    events
        Detection
    end
    
     methods (Static)
        theta = posori(angle);
        radars = LEM_radars_placement_coverage(lbsd, range, noise, angle);
    end
    
    methods
        
        function obj = RADAR(pos, range, angle, dir, ID, lbsd)
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
            obj.lbsd = lbsd;
            obj.time = 0;
        end
        
        function scan(obj, UAS)
            % scan - scans the environment to be able to detect objects
            % Input:
            %   UAS (n x 3) - all the updated UAS positions in the
            %      environment
            num_targets = 0;
            obj.targets = [];
            for object = 1:length(UAS)
                uas = UAS(object);
                uas_v = [uas.gps.lat, uas.gps.lon, uas.gps.alt] - ...
                    obj.location;
                dist_v = norm(uas_v);
                unit_v = uas_v/dist_v;
                alpha = RADAR.posori(acos(dot(unit_v, obj.dirVector)));
                noise = mvnrnd([0,0,0], eye(3));
                if(alpha <= obj.apexAngle && dist_v <= obj.range)
                    num_targets = num_targets + 1;
                    obj.targets(num_targets).x = uas.gps.lat + noise(1);
                    obj.targets(num_targets).y = uas.gps.lon + noise(2);
                    obj.targets(num_targets).z = uas.gps.alt + noise(3);
                    obj.targets(num_targets).s = uas.nominal_speed...
                        + rand;
                    %targets(num_targets).diam = ksize + rand;
                    obj.targets(num_targets).time = obj.time;
                    obj.targets(num_targets).id = obj.ID;
                end
            end
            if(obj.graph)
                obj.showDetection(1)
            end
            if (~isempty(obj.targets))
                notify(obj, 'Detection');
            end
        end
        
        function handle_events(obj, src, event)
        % handle_NewPositions - handles the event of UAS moving through 
        %   the simulation
        % Input:
        %   obj (radar object) - the specific radar object
        %   src (Sim object) - simulation object
        %   event (event object) - The event that caused this method to 
        %       to be called.
            if event.EventName == "Tick"
                obj.scan(src.uas_list);
                obj.showDetection(obj.graph);
            end
                
        end
        
        function showDetection(obj, displayGraph)
        % showDetection - Displays the radar field at the given the current
        %   time period.
            if(displayGraph)
                p = obj.lbsd.plot();
                p.DisplayName = 'lane';
                hold on;
                grid on;
                r = linspace(0,2*pi);
                th = linspace(0,2*pi) +30;
                [R,T] = meshgrid(r,th) ;
                X = R.*cos(T) ;
                Y = R.*sin(T) ;
                Z = R;
                hold on;
                h = surf(X + obj.location(1),...
                    Y+ obj.location(2),Z + obj.location(3), ...
                    'EdgeColor', 'interp', ...
                    'DisplayName', 'Radar Field', 'Visible', 'off');
                set(h,'facealpha',0.1)
                set(h,'edgealpha',0.05)
                alpha = 90 - acosd(dot([0,0,1], obj.dirVector));
                rotate(h, obj.dirVector, (90 - alpha));          
                set(h, 'Visible', 'on')
                view(2);
                axis equal;
                scatter(gca, obj.location(1), obj.location(2), 'go', ...
                    'DisplayName', 'Radar Position');
                title(strcat("Time: ", num2str(obj.time), " Radar ", ...
                    num2str(obj.ID)));              
                if(~isempty(obj.targets))
                    x = obj.targets(:).x;
                    y = obj.targets(:).y;
                    scatter(ax1, x, y, 'ro', 'DisplayName', 'Object');
                    obj.graph = 1;
                end
                legend('Location', 'eastoutside');
                hold off;
                xlim([min(h.XData(:)) - 3, max(h.XData(:)) + 3]);
                ylim([min(h.YData(:)) - 3, max(h.YData(:)) + 3]);
                xlabel("East - West Direction");
                ylabel("North - South Direction");
            else
                obj.graph = 0;
            end               
        end
        
        function subscribe_to_detection(obj, subscriber)
        % subscribeToDetection - sets an event listener to trigger when 
        %   the radar detects an object
        % Input:
        %   obj - an instance of the radar class
        %   subscriber - a function hangle to trigger
            lh = obj.addlistener('Detection', subscriber);
            obj.detection_listers = [obj.detection_listers, lh];
        end
    end
end
    
