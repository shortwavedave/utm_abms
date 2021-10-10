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
        scanData; % Scan Figure handle
        showgraph = 0;
    end
    
    events
        Detection
    end
    
    methods (Static)
        theta = posori(angle);
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
            obj.scanData = struct('fHandle', figure('Visible', 'off'), ...
                'targets', ...
                scatter(obj.location(1) - obj.range, ...
                obj.location(2) - obj.range, ...
                'DisplayName', 'Object'));
            set(obj.scanData.fHandle, 'Name', strcat(num2str(obj.time), " Radar ", num2str(obj.ID)));
            linkdata(obj.scanData.fHandle);
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
            if(obj.showgraph)
                if(~isvalid(obj.scanData.targets))
                    obj.scanData.targets = scatter(obj.location(1) - obj.range, ...
                        obj.location(2) - obj.range, ...
                        'DisplayName', 'Object');
                end
                if(isempty(obj.targets))
                    obj.scanData.targets.XData = obj.location(1) - obj.range;
                    obj.scanData.targets.YData = obj.location(2) - obj.range;
                else
                    obj.scanData.targets.XData = obj.targets(:).x;
                    obj.scanData.targets.YData = obj.targets(:).y;
                end
                refreshdata(obj.scanData.targets)
                if(isvalid(obj.scanData.fHandle))
                    title(obj.scanData.fHandle.CurrentAxes,...
                        strcat(num2str(obj.time), " Radar ", num2str(obj.ID)));
                end
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
                obj.time = obj.time + src.tick_del_t;
            end
            
        end
        
        function showDetection(obj)
            % showDetection - Displays the radar field at the given the current
            %   time period.
            alpha = 90 - acosd(dot([0,0,1], obj.dirVector));
            if(alpha == 90)
                obj.horizontalPlot();
            else
                obj.tiltedPlot();
            end
            obj.showgraph = true;
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
    
    methods(Access = private)
        function horizontalPlot(obj)
            obj.scanData.fHandle = figure();
            r = linspace(0,2*pi);
            th = linspace(0,2*pi) +30;
            [R,T] = meshgrid(r,th) ;
            X = R.*cos(T) ;
            Y = R.*sin(T) ;
            Z = R;
            radarField = surf(X + obj.location(1),...
                Y+ obj.location(2),Z + obj.location(3), ...
                'EdgeColor', 'interp', ...
                'DisplayName', 'Radar Field', 'Visible', 'off');
            set(radarField,'facealpha',0.1)
            set(radarField,'edgealpha',0.05)
            set(radarField, 'Visible', 'on');
            hold on;
            lanePlot = obj.lbsd.plot();
            lanePlot.DisplayName = 'lane';
            view(2);
            grid on;
            scatter(gca, obj.location(1), obj.location(2), 'go', ...
                'DisplayName', 'Radar Position');
            if(~isempty(obj.targets))
                obj.scanData.targets.x = obj.targets(:).x;
                obj.scanData.targets.y = obj.targets(:).y;
                obj.graph = 1;
            else
                obj.scanData.targets = struct('x', obj.location(1) - obj.range...
                    , 'y', obj.location(2) - obj.range);
            end
            obj.scanData.targets = scatter(gca, obj.scanData.targets(:).x,...
                obj.scanData.targets(:).y, 'ro', 'DisplayName', 'Object');
            linkdata(obj.scanData.fHandle);
            title(strcat("Time: ", num2str(obj.time), " Radar ", ...
                num2str(obj.ID)));
            xlim([min(radarField.XData(:)) - 3, ...
                max(radarField.XData(:)) + 3]);
            ylim([min(radarField.YData(:)) - 3, ...
                max(radarField.YData(:)) + 3]);
            legend('location', 'eastoutside');
            xlabel("West - East");
            ylabel("South - North");
        end
        
        function tiltedPlot(obj)
            obj.scanData.fHandle = figure();
            r = linspace(0,2*pi);
            th = linspace(0,2*pi) + 30;
            [R,T] = meshgrid(r,th) ;
            X = R.*cos(T) ;
            Y = R.*sin(T) ;
            Z = R;
            radarField = surf(X + obj.location(1),...
                Y+ obj.location(2),Z + obj.location(3), ...
                'EdgeColor', 'interp', ...
                'DisplayName', 'Radar Field', 'Visible', 'on');
            set(radarField,'facealpha',0.1)
            set(radarField,'edgealpha',0.05)
            alpha = 90 - acosd(dot([0,0,1], obj.dirVector));
            rotate(radarField, obj.dirVector, (90 - alpha));
            minZ = min(radarField.ZData(:));
            [row, col] = find(radarField.ZData == minZ, 1);
            ydiff = obj.location(2) - mean(mean(radarField.YData(row, col)));
            radarField.YData = radarField.YData + ydiff;
            xdiff = obj.location(1) - mean(radarField.XData(:));
            radarField.XData = radarField.XData - xdiff;
            set(radarField, 'Visible', 'on');
            hold on;
            lanePlot = obj.lbsd.plot();
            lanePlot.DisplayName = 'lane';
            view(2);
            grid on;
            scatter(gca, radarField.XData(row), radarField.YData(row), 'go', ...
                'DisplayName', 'Radar Position');
            if(~isempty(obj.targets))
                obj.scanData.targets.x = obj.targets(:).x;
                obj.scanData.targets.y = obj.targets(:).y;
                obj.graph = 1;
            else
                obj.scanData.targets = struct('x', obj.location(1) ...
                    - obj.range, 'y', obj.location(2) - obj.range);
            end
            obj.scanData.targets = scatter(gca, obj.scanData.targets(:).x,...
                obj.scanData.targets(:).y, 'ro', 'DisplayName', 'Object');
            linkdata(obj.scanData.fHandle);
            title(strcat("Time: ", num2str(obj.time), " Radar ", ...
                num2str(obj.ID)));
            xlim([min(radarField.XData(:)) - 3, ...
                max(radarField.XData(:)) + 3]);
            ylim([min(radarField.YData(:)) - 3, ...
                max(radarField.YData(:)) + 3]);
            legend('location', 'eastoutside');
            xlabel("West - East");
            ylabel("South - North");
        end
    end
end

