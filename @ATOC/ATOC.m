classdef ATOC < handle
    %ATOC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        reseverationData  % Reseveration Data (planned flight data)
        telemetryData % Sensory/Drone Data 
        laneData % Density Informaiton
        radars % Radar Information
        time % Keep track of time
    end
    
    function aDensity = averageDensity(obj, plot, time, lanes)
            % Average Density/time Graph Creation
            if plot == 1
                % Show the average density graph
                if ~isEmpty(time)
                    % Show the selected time selection
                else
                    % Show all time
                end
                if ~isEmpty(lanes)
                    % Show the selected lanes/density lane graph
                else
                    % Show across all lanes
                end
            end
            % Return average Density given the time interval
            % Have to calculate the number of lanes/drones.
            return lanesnum/dronesnum;
        end
        
        function denseList = mostDense(obj, number, time)
            % Returns the specified number of most dense lanes
            if isEmpty(number)
                % Return the most dense lane
            end
            
            if isEmpty(time)
                % return the most dense throughout the simulation
            end
                
        end
        
        function laneTrajectory(obj, drone, planned, actual, sensor, time)
            % Plots The Drone Trajectory Information
            if ~isEmpty(planned)
                % Plot the planned trajectory
            end
            if ~isEmpty(actual)
                % Plot the actual trajectory
            end
            if ~isEmpty(sensor)
                % Plot the sensor information
            end
            if ~isEmpty(time)
                % Grab just the selected area
            else
                % plot the entire drone flight
            end
        end
        
        function averDev = averageDev(obj, drones, time)
            if ~isEmpty(time)
                % Only find it for the specfied Time interval
            end
            % Calculates the average deviation between the
            % sensory/drone from planned
            % Can use LEM_dist_line_line(line1, line2) - to calculate the
            % different between actual and planned.
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        
        function handle_events(obj, src, event)
            if event.EventName == "Tick"
                % Update drone information
                % Update sensor information
                % 
            end
            if event.EventName == "reseveration"
                % Update Reservation System
            end
            if event.EventName == "telemetry"
                % Store Telemetry Data
                id = src.id;
                droneData = [src.x, src.v, obj.time];
                obj.telemetryData(id).droneInfo(end + 1) = droneData;
            end
        end
    end
end

