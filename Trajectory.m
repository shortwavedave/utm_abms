classdef Trajectory < handle
    %TRAJECTORY An object of this class holds a trajectory
    %   This trajectory object is stateful, so each call to "step" advances
    %   the state (e.g., position, velocity, etc.). To reinitialize, call
    %   "reset".
    %   Example Usage:
    %       traj = Trajectory(1, [0 0 0; 0 0 100; 100 0 100; 100 0 0], ...
    %           [0; 10; 20; 30],[0 10 10 0], [0 0 0 0])
    %       traj.plot(true);
    
    properties
        trajectory % trajectory object
        position = [0 0 0]
        orientation % quaternion
        angularVelocity = [0 0 0]
        velocity = [0 0 0]
        acceleration = [0 0 0]
        time_s = 0
        f_hz = 0
        complete = false
    end
    
    methods
        function obj = Trajectory(f_hz, waypoints_m, toa_s, ...
                ground_speed_ms, climb_rate_ms)
            %TRAJECTORY Construct an trajectory plan object
            %   On Input:
            %       f_hz: the sample rate of the trajectory in hertz
            %       waypoints_m: nx3 positions in meters [x,y,z], each row
            %           is an individual waypoint.
            %       toa_s: nx1 vector of nonnegative increasing numbers
            %           representing the time-of-arrival constraints for
            %           each waypoint. The first element must be 0.
            %       ground_speed_ms: n element vector representing 
            %           gound speed at each waypoint, in m/s
            %       climb_rate_ms: n element vector representing climb rate 
            %           at each waypoint, in m/s
            %   On Output:
            %       PlanTrajectory object
            %   Example:
            %       plan = PlanTrajectory(1, [0 0 0; 1 1 1], [0; 1], ...
            %           [0 0], [0 0])
            obj.f_hz = f_hz;
            obj.trajectory = waypointTrajectory(waypoints_m, ...
                'TimeOfArrival', toa_s, ...
                'SampleRate', f_hz, ...
                'AutoBank', true, ...
                'AutoPitch', false, ...
                'GroundSpeed', ground_speed_ms, ...
                'ClimbRate', climb_rate_ms);
        end
        
        function [position,orientation,velocity,acceleration,...
                angularVelocity] = step(obj)
            %STEP generate the trajectory frame for the current step
            %   step advances the trajectory, interpolated according to the 
            %   sample rate that is passed to the constructor. Each step 
            %   returns the interpolated state values.
            
            [position,orientation,velocity,acceleration,angularVelocity] ...
                = obj.trajectory.step();
            obj.position = position;
            obj.orientation = orientation;
            obj.velocity = velocity;
            obj.acceleration = acceleration;
            obj.angularVelocity = angularVelocity;
            obj.time_s = obj.time_s + 1/obj.f_hz;
            obj.complete = obj.trajectory.isDone;
        end
        
        function info = getWpInfo(obj)
            %GETWPINFO return the waypoint information as a table
            info = waypointInfo(obj.trajectory);
        end
        
        function reset(obj)
            % RESET re-initialize the trajectory object
            obj.trajectory.reset();
            obj.position = [0 0 0];
            obj.orientation = quaternion();
            obj.velocity = [0 0 0];
            obj.acceleration = [0 0 0];
            obj.angularVelocity = [0 0 0];
            obj.time_s = 0;
            obj.complete = false;
        end
        
        function complete = isDone(obj)
            complete = obj.complete;
        end
        
        function plot(obj, real_time)
            % Plot plot the states of this trajectory.
            %   Note: calling plot will "reset" the trajectory state
            %   because it has to step through each value.
            %   On Input:
            %       obj: an instance of the Trajectory class
            %       real_time: boolean - if true, the plot is updated at
            %       the sample rate passed into the constructor.
            if nargin < 2
                real_time = false;
            end
            tInfo = obj.getWpInfo();
%             figure(1)
            plot3(tInfo.Waypoints(1,1), tInfo.Waypoints(1,2), ...
                tInfo.Waypoints(1,3),'b*')
%             title('Position')
%             axis square
%             xlabel('X')
%             ylabel('Y')
%             grid on
            hold on
            
            orient = zeros(tInfo.TimeOfArrival(end)*...
                obj.trajectory.SampleRate,1,'quaternion');
            vel = zeros(tInfo.TimeOfArrival(end)*...
                obj.trajectory.SampleRate,3);
            acc = vel;
            angVel = vel;
            
            count = 1;
            obj.reset();
            x = [];
            y = [];
            z = [];
%             p = [];
            while ~isDone(obj)
               [pos,orient(count),vel(count,:),acc(count,:),...
                   angVel(count,:)] = obj.step();
               x = [x; pos(1)];
               y = [y; pos(2)];
               z = [z; pos(3)];
               p = plot3(x,y,z,'b-o','MarkerFaceColor','b');
               p.XDataSource = 'x';
               p.YDataSource = 'y';
               p.ZDataSource = 'z';
               if real_time
                   pause(obj.trajectory.SamplesPerFrame/...
                       obj.trajectory.SampleRate)
               end
               count = count + 1;
            end
            
%             figure(2)
%             timeVector = 0:(1/obj.trajectory.SampleRate):...
%                 tInfo.TimeOfArrival(end);
%             eulerAngles = eulerd([orient(1);orient],...
%                 'ZYX','frame');
%             plot(timeVector,eulerAngles(:,1), ...
%                  timeVector,eulerAngles(:,2), ...
%                  timeVector,eulerAngles(:,3));
%             title('Orientation Over Time')
%             legend('Rotation around Z-axis', ...
%                    'Rotation around Y-axis', ...
%                    'Rotation around X-axis', ...
%                    'Location','southwest')
%             xlabel('Time (seconds)')
%             ylabel('Rotation (degrees)')
%             grid on
% 
%             figure(3)
%             plot(timeVector(2:end),vel(:,1), ...
%                  timeVector(2:end),vel(:,2), ...
%                  timeVector(2:end),vel(:,3));
%             title('Velocity Over Time')
%             legend('North','East','Down')
%             xlabel('Time (seconds)')
%             ylabel('Velocity (m/s)')
%             grid on
% 
%             figure(4)
%             plot(timeVector(2:end),acc(:,1), ...
%                  timeVector(2:end),acc(:,2), ...
%                  timeVector(2:end),acc(:,3));
%             title('Acceleration Over Time')
%             legend('North','East','Down','Location','southwest')
%             xlabel('Time (seconds)')
%             ylabel('Acceleration (m/s^2)')
%             grid on
% 
%             figure(5)
%             plot(timeVector(2:end),angVel(:,1), ...
%                  timeVector(2:end),angVel(:,2), ...
%                  timeVector(2:end),angVel(:,3));
%             title('Angular Velocity Over Time')
%             legend('North','East','Down')
%             xlabel('Time (seconds)')
%             ylabel('Angular Velocity (rad/s)')
%             grid on
%             obj.reset();
%             hold off
        end
    end
end

