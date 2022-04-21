%% Track Monitor Unit/Load/Integeration Tests For Classification of Flights
% This class is used to test the Track Monitor class to ensure that the
% classification of flights and the setup for this is working properly.

classdef ClassificationTests < matlab.unittest.TestCase
    properties
        lbsd
        monitor
    end

    methods(TestClassSetup)
        function attachFilesForTestClass(TestClassSetup)
            % attchFilesForTestClass - adds the path for the unit tests for
            % the overall classes.
            addpath('..\..\..\utm_abms');
        end
    end
    %% Before and After Test
    % This section is used to setup each test in the class, as well as
    % breakdown any variables in the class.

    methods(TestMethodSetup)
        function createLBSD(testCase)
            % createLBSD - creates a lane system for each unit test.
            % Input:
            %   testCase (test class handle)
            lane = LBSD();
            lane = lane.genSampleLanes(10, 15);
            testCase.lbsd = lane;
        end
        function createTrackMonitor(testCase)
            % createTrackMonitor - Creates the track monitor for each unit
            % test.
            testCase.monitor = TrackMonitor();
        end
    end

    methods(TestMethodTeardown)
    end

    %% Static Helper Methods
    % This section is where all of the helper methods are located that are
    % used by all of the other tests.

    methods(Static)
        function [sim, radarList, num_steps] = setUpSimulationFlights(testCase)
            sim = Sim();
            sim.lbsd = testCase.lbsd;
            sim.initialize();
            radarList = sim.radar_list;
            res = sim.lbsd.getReservations();
            minTime = min(res.entry_time_s);
            maxTime = max(res.exit_time_s);
            num_steps = floor((maxTime - minTime)/sim.step_rate_hz);
            sim.atoc.time = minTime;
            for numradar = 1:length(sim.radar_list)
                sim.radar_list(numradar).time = minTime;
            end
        end
        function [waypoints, timeOfArrival] = GenerateHobbistTwo(testCase)
            launchVert = testCase.lbsd.getRandLaunchVert();
            pt1 = testCase.lbsd.getVertPositions(launchVert);
            pt1 = transpose(pt1);
            traj = ClassificationTests.LEM_hobby_type2(pt1,10,5,10,...
                1,0.1,10,50,0.5);
            waypoints = traj(:, 1:3);
            timeOfArrival = traj(:, 4);
        end
        function pts = LEM_sphere_pts(center, radius, num_pts)
            pts = zeros(num_pts,3);

            % For the number of points
            for k = 1:num_pts
                % Create some random theta in the x-y plane
                theta = 2*pi*rand;
                % x is the x axis
                x = cos(theta);
                % y is the y axis
                y = sin(theta);
                % some direction
                P = [x;y;0];
                % Reverse direction
                u = [-y;x;0];
                ux = -y;
                uy = x;
                uz = 0;

                phi = 2*pi*rand;
                c = cos(phi);
                s = sin(phi);
                R = zeros(3,3);
                R(1,1) = c + ux^2*(1-c);
                R(1,2) = ux*uy*(1-c) - uz*s;
                R(1,3) = ux*uz*(1-c) + uy*s;
                R(2,1) = uy*ux*(1-c) + uz*s;
                R(2,2) = c + uy^2*(1-c);
                R(2,3) = uy*uz*(1-c) - ux*s;
                R(3,1) = uz*ux*(1-c) - uy*s;
                R(3,2) = uz*uy*(1-c) + ux*s;
                R(3,3) = x + uz^2*(1-c);

                pt = radius*R*P;
                pts(k,:) = pt;
            end
            pts(:,1) = pts(:,1) + center(1);
            pts(:,2) = pts(:,2) + center(2);
            pts(:,3) = pts(:,3) + center(3);
        end
        function traj = LEM_hobby_type2(launch_site,height,radius,num_moves,...
                speed,del_t,min_wait,max_wait,ratio)
            % LEM_hobby_type2 - up and random between sphere surface points
            % On input:
            %     launch_site (3x1 vector): ground aunch point
            %     height (float): altitude for center of sphere
            %     radius (float): radius of sphere
            %     num_moves (int): number of random moves across sphere
            %     speed (float): speed of UAS
            %     del_t (float): time sample interval
            %     min_wait (float): minimum time to hover
            %     max_wait (float): maximum time to hover
            %     ratio (float): determines radius of error circle for traj noise
            %       radius is the inter-point distance * ratio
            % On ouput:
            %     traj (nx4 array): x,y,z,t
            % Call:
            %     traja = LEM_hobby_type2([x;y;z],10,5,10,1,0.1,10,50,0.5);
            % Author:
            %     T. Henderson
            %     UU
            %     Spring 2021
            %

            % up
            pt1 = launch_site;
            pt2 = [pt1(1);pt1(2);height];
            dist12 = norm(pt2-pt1);
            dir = [0;0;1];
            pt = pt1;
            traj = [pt'];
            while norm(pt1-pt)<dist12
                pt = pt + speed*dir*del_t;
                traj = [traj;pt'];
            end
            if norm(pt1-pt)>dist12
                traj(end,:) = pt2';
            end

            % Generate sphere moves
            s_pts = ClassificationTests.LEM_sphere_pts(pt2,radius,num_moves);
            c_pt = pt2;
            for p = 1:num_moves
                n_pt = s_pts(p,:)';
                dir = n_pt - c_pt;
                dir = dir/norm(dir);
                dist_cn= norm(n_pt-c_pt);
                pt = c_pt;
                while norm(pt-c_pt)<dist_cn
                    pt = pt + speed*del_t*dir;
                    traj = [traj;pt'];
                end
                if norm(pt-c_pt)>dist_cn
                    traj(end,:) = n_pt';
                end
                wait_time = min_wait + rand*(max_wait-min_wait);
                t = del_t;
                while t<wait_time
                    t = t + del_t;
                    traj = [traj;pt'];
                end
                c_pt = n_pt;
            end

            % down
            dir = pt2 - c_pt;
            dir = dir/norm(dir);
            dist_c2= norm(c_pt-pt2);
            pt = c_pt;
            while norm(pt-c_pt)<dist_c2
                pt = pt + speed*del_t*dir;
                traj = [traj;pt'];
            end
            if norm(pt-c_pt)>dist_c2
                traj(end,:) = pt2';
            end
            dir = [0;0;-1];
            pt = pt2;
            while norm(pt2-pt)<dist12
                pt = pt + dir*del_t;
                traj = [traj;pt'];
            end
            if norm(pt2-pt)>dist12
                traj(end,:) = pt1';
            end
            num_pts = length(traj(:,1));
            traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
            trajn = ClassificationTests.LEM_noisy_traj(traj,ratio,speed,del_t);
            traj = trajn;
        end
        function trajn = LEM_noisy_traj(traj,ratio,speed,del_t)
            %

            ZERO_THRESH = 0.001;

            [num_pts,~] = size(traj);
            trajn = traj;
            T = zeros(3,3);
            for p = 1:num_pts-1
                pt1 = traj(p,1:3);
                pt2 = traj(p+1,1:3);
                dist12 = norm(pt2-pt1);
                if dist12<ZERO_THRESH
                    trajn(p+1,1:3) = pt1 + 0.01*randn(1,3);
                else
                    radius = dist12*ratio;
                    done = 0;
                    while done==0
                        pu = [-radius;-radius] + 2*radius*rand(2,1);
                        if norm(pu)<=radius
                            done = 1;
                        end
                    end
                    pu = [pu;0;1];
                    dir2 = pt2;
                    dir2 = dir2/norm(dir2);
                    z = [0;0;1];
                    theta = acos(dot(z,dir2));
                    d_perp = cross(z,dir2);
                    T = ClassificationTests.LEM_rot_3D(d_perp,theta);
                    T(1:3,4) = pt2;
                    pc = T*pu;
                    pc = pc(1:3)';
                    dist1c = norm(pc-pt1);
                    if ratio==0
                        v = dist1c;
                    else
                        v = dist1c - min(dist1c/2,randn);
                    end
                    dir1c = pc - pt1;
                    dir1c = dir1c/norm(dir1c);
                    trajn(p+1,1:3) = pt1 + v*dir1c*speed*del_t;
                end
            end
        end
        function T = LEM_rot_3D(x,theta)
            ux = x(1);
            uy = x(2);
            uz = x(3);
            c = cos(theta);
            s = sin(theta);
            T = eye(4,4);
            T(1,1) = c + ux^2*(1-c);
            T(1,2) = ux*uy*(1-c) - uz*s;
            T(1,3) = ux*uz*(1-c) + uy*s;
            T(2,1) = uy*ux*(1-c) + uz*s;
            T(2,2) = c + uy^2*(1-c);
            T(2,3) = uy*uz*(1-c) - ux*s;
            T(3,1) = uz*ux*(1-c) - uy*s;
            T(3,2) = uz*uy*(1-c) + ux*s;
            T(3,3) = c + uz^2*(1-c);
        end
    end

    %% Setup Functions
    % This section is to test that the setup for classifications are
    % working properly. This includes building KD tress as well as ensuring
    % that the integration code is not throwing errors and properly
    % integrated.

    methods(Test)
        function noErrorThrowsKDTree(testCase)
            % noErrorThrowsKDTree - Ensures that when calling the method
            % that the function doesn't throw any errors.
            % Input:
            %   testCase (test class handle)
            try
                testCase.monitor.initializeLaneStructor(testCase.lbsd);
                testCase.verifyTrue(true);
            catch
                testCase.verifyTrue(false);
            end
        end
        function noErrorWithAllofTheIntegrationTests(testCase)
            % noErrorWithAllofTheIntegrationTests - ensures that all of the
            % methods that were used to help with the functionality for
            % classification doesn't throw any errors. 
            try
                telemetry = table("1", [0,0,0], [1,1,1], 0);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], [1,1,1], 0);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry,radar,[], 1);
                testCase.verifyTrue(true);
            catch
                testCase.verifyTrue(false);
            end
        end
    end

    %% Nominally Tests
    % This section is used to test that a flight is nominally rather than
    % a Hobby or Rogue flight. These flights include slightly off course,
    % headway distances are not meet, etc. These flights are heading in the
    % right direction and are close to the KD Points.

    methods(Test)

    end

    %% Hobbyist Type 1:
    % This section is used to test flights that display Hobbyist Type 1:
    % Flies up from one place and makes a few moves above the launch site,
    % then eventually lands at the same site.

    methods(Test)
        function aSingleNormalFlightWithSlightNoise(testCase)
            % aSingleNormalFlightWithSlightNoise - This test is to run a
            % single flight that contains some noisy flight.
            rng(2);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            uas = sim.uas_list(1);
            sim.uas_list = uas;

            for i = 1:num_steps
                tnew = table();
                tnew.ID = "";
                tnew.pos = zeros(1,3);
                tnew.speed = zeros(1,3);
                tnew.time = 0;
                telemetry = tnew;
                radars = tnew;
                index = 1;
                uas_step = uas.stepTrajectory();
                if uas.active
                    pos = uas.exec_traj;
                    if ~isempty(pos)
                        uas.gps.lon = pos(uas_step, 1)+ rand()*.1;
                        uas.gps.lat = pos(uas_step, 2)+ rand()*.1;
                        uas.gps.alt = pos(uas_step, 3)+ rand()*.1;
                        uas.gps.commit();
                        traj = uas.exec_traj;
                        set(uas.h, 'XData', traj(:,1), ...
                            'YData', traj(:,2), ...
                            'ZData', traj(:,3));
                        sim.step(1);
                        telemetry{index, {'ID', 'pos', 'speed', 'time'}} = ...
                                [uas.id, [uas.gps.lon, uas.gps.lat, uas.gps.alt], ...
                                [uas.gps.vx, uas.gps.vy, uas.gps.vz], 0];
                        testCase.monitor.AnalyzeFlights(telemetry, radars, [], 1);
                        flightInfo = testCase.monitor.flights;
                        testCase.verifyEqual(flightInfo.classification(end), "normal");
                    end
                end
            end
        end
        function twoSingleNormalFlights(testCase)
            % twoSingleNormalFlights - Runs through two Normal flights and
            % tests to ensure that it classifies two normal flights in the
            % air system.
            rng(1);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            sim.uas_list = [sim.uas_list(1), sim.uas_list(2)];

            for i = 1:num_steps
                tnew = table();
                tnew.ID = "";
                tnew.pos = zeros(1,3);
                tnew.speed = zeros(1,3);
                tnew.time = 0;
                telemetry = tnew;
                radars = tnew;
                index = 1;
                for j = 1:size(sim.uas_list, 1)
                    uas_step = sim.uas_list(j).stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            sim.step(1);
                            telemetry{index, {'ID', 'pos', 'speed', 'time'}} = ...
                                    [uas.id, [uas.gps.lon, uas.gps.lat, uas.gps.alt], ...
                                    [uas.gps.vx, uas.gps.vy, uas.gps.vz], 0];
                            testCase.monitor.AnalyzeFlights(telemetry, radars, [], 1);
                            flightInfo = testCase.monitor.flights;
                            testCase.verifyEqual(flightInfo.classification(end), "normal");
                        end
                    end
                end
            end
        end
        function twoSingleNormalFlightsWithNoise(testCase)
            % twoSingleNormalFlightsWithNoise - This test ensures that two
            % flights with noise are still classified as Normal.
            rng(1);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            sim.uas_list = [sim.uas_list(1), sim.uas_list(2)];

            for i = 1:num_steps
                tnew = table();
                tnew.ID = "";
                tnew.pos = zeros(1,3);
                tnew.speed = zeros(1,3);
                tnew.time = 0;
                telemetry = tnew;
                radars = tnew;
                index = 1;
                for j = 1:size(sim.uas_list, 1)
                    uas_step = sim.uas_list(j).stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.1;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.1;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.1;
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            sim.step(1);
                            telemetry{index, {'ID', 'pos', 'speed', 'time'}} = ...
                                    [uas.id, [uas.gps.lon, uas.gps.lat, uas.gps.alt], ...
                                    [uas.gps.vx, uas.gps.vy, uas.gps.vz], 0];
                            testCase.monitor.AnalyzeFlights(telemetry, radars, [], 1);
                            flightInfo = testCase.monitor.flights;
                            testCase.verifyEqual(flightInfo.classification(end), "normal");
                        end
                    end
                end
            end
        end
        function multipleSingleNormalFlight(testCase)
            % multipleSingleNormalFlight - This test ensures that if
            % multiple normal flights are classified as normal.
            rng(1);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);

            for i = 1:num_steps
                tnew = table();
                tnew.ID = "";
                tnew.pos = zeros(1,3);
                tnew.speed = zeros(1,3);
                tnew.time = 0;
                telemetry = tnew;
                radars = tnew;
                index = 1;
                for j = 1:size(sim.uas_list, 1)
                    uas_step = sim.uas_list(j).stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1);
                            uas.gps.lat = pos(uas_step, 2);
                            uas.gps.alt = pos(uas_step, 3);
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            sim.step(1);
                            telemetry{index, {'ID', 'pos', 'speed', 'time'}} = ...
                                    [uas.id, [uas.gps.lon, uas.gps.lat, uas.gps.alt], ...
                                    [uas.gps.vx, uas.gps.vy, uas.gps.vz], 0];
                            testCase.monitor.AnalyzeFlights(telemetry, radars, [], 1);
                            flightInfo = testCase.monitor.flights;
                            testCase.verifyEqual(flightInfo.classification(end), "normal");
                        end
                    end
                end
            end
        end
        function multipleSingleNormalFlightWithNoise(testCase)
            % multipleSingleNormalFlightWithNoise - This test checks to run
            % multiple flights with slight noise that ensures that all of
            % these flights are classified as Normal. 
            rng(1);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);

            for i = 1:num_steps
                tnew = table();
                tnew.ID = "";
                tnew.pos = zeros(1,3);
                tnew.speed = zeros(1,3);
                tnew.time = 0;
                telemetry = tnew;
                radars = tnew;
                index = 1;
                for j = 1:size(sim.uas_list, 1)
                    uas_step = sim.uas_list(j).stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.1;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.1;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.1;
                            uas.gps.commit();
                            traj = uas.exec_traj;
                            set(uas.h, 'XData', traj(:,1), ...
                                'YData', traj(:,2), ...
                                'ZData', traj(:,3));
                            sim.step(1);
                            telemetry{index, {'ID', 'pos', 'speed', 'time'}} = ...
                                    [uas.id, [uas.gps.lon, uas.gps.lat, uas.gps.alt], ...
                                    [uas.gps.vx, uas.gps.vy, uas.gps.vz], 0];
                            testCase.monitor.AnalyzeFlights(telemetry, radars, [], 1);
                            flightInfo = testCase.monitor.flights;
                            testCase.verifyEqual(flightInfo.classification(end), "normal");
                        end
                    end
                end
            end
        end
    end

    %% Hobby 2 Tests
    % This section is used to test flights that display Hobbyist Type 2:
    % Flies up from one place and makes a few moves above the launch site,
    % hovers after each move, then eventually lands at the launch site.

    methods(Test)
        function noHobbie2Tests(testCase)
            % noHobbie2Tests - This test ensures that no hobbist detection
            % was given the waypoints.
            % Generate normal point
            rng(0);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            uas = sim.uas_list(1);
            sim.uas_list = uas;

            for i = 1:num_steps
                tnew = table();
                tnew.ID = "";
                tnew.pos = zeros(1,3);
                tnew.speed = zeros(1,3);
                tnew.time = 0;
                telemetry = tnew;
                radars = tnew;
                index = 1;
                uas_step = uas.stepTrajectory();
                if uas.active
                    pos = uas.exec_traj;
                    if ~isempty(pos)
                        uas.gps.lon = pos(uas_step, 1);
                        uas.gps.lat = pos(uas_step, 2);
                        uas.gps.alt = pos(uas_step, 3);
                        uas.gps.commit();
                        traj = uas.exec_traj;
                        set(uas.h, 'XData', traj(:,1), ...
                            'YData', traj(:,2), ...
                            'ZData', traj(:,3));
                        sim.step(1);
                        telemetry{index, {'ID', 'pos', 'speed', 'time'}} = ...
                                [uas.id, [uas.gps.lon, uas.gps.lat, uas.gps.alt], ...
                                [uas.gps.vx, uas.gps.vy, uas.gps.vz], 0];
                        testCase.monitor.AnalyzeFlights(telemetry, radars, [], 1);
                        flightInfo = testCase.monitor.flights;
                        testCase.verifyEqual(flightInfo.classification(end), "normal");
                    end
                end
            end
        end
        function hobbie2TestsSingle(testCase)
            % hobbie2TestsSingle - This test is used to check that the
            % Track Monitor Class Identifies the Hobbist Two Behavior for a
            % single hobbist two flight.
            rng(1);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            [waypoints1, timeOfArrival1] = ...
                ClassificationTests.GenerateHobbistTwo(testCase);
            del_t = 0;
            for index = 1:size(timeOfArrival1, 1)
                currentPosition = waypoints1(index, :);
                vel = [1,1,1];
                time = timeOfArrival1(index);
                del_t = time - del_t;
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], del_t);
            end
            flightInfo = testCase.monitor.flights;
            testCase.verifyEqual(flightInfo.classification(end),...
                "Hobbist Two");
        end
        function hobbie2TestsDoubleFlightsSameTime(testCase)
            % hobbie2TestsDoubleFlightsSameTime - This test ensure that the
            % Track Monitor Class will correctly distinguish between two
            % flight behaivor: Normal and Hobbist two.
            %             [waypoints1, timeOfArrival1] = ...
            %                 ClassificationTests.GenerateHobbistTwo(testCase);
            %             trajectory1 = waypointTrajectory(waypoints1, timeOfArrival1);
            %
            %             [waypoints1, timeOfArrival1] = ...
            %                 ClassificationTests.GenerateWayPointsNormal(testCase);
            %             trajectory2 = waypointTrajectory(waypoints1, timeOfArrival1);
            %
            %             time = 0;
            %             count = 0;
            %
            %             while ~isDone(trajectory1) && ~isDone(trajectory2)
            %                 Pull flight information - create a table with tel_info
            %                 [currentPosition, ~] = trajectory1();
            %                 [currentPosition2, ~] = trajectory2();
            %                 vel = [1,1,1];
            %                 telemetry = table("1", currentPosition, vel, time);
            %                 telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
            %                 name = "2";
            %                 pos = currentPosition2;
            %                 speed = randi(5, [1,3]);
            %                 telemetry{2, {'ID', 'pos', 'speed', 'time'}} ...
            %                     = [name, pos, speed, time];
            %                 radar = table("", [0,0,0], vel, time);
            %                 radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
            %                 time = time + .01;
            %                 testCase.monitor.AnalyzeFlights(telemetry, radar, [], .01);
            %                 if(count > 6)
            %                     flightInfo = testCase.monitor.flights;
            %                     [row1, ~] = find("1", flightInfo.uas_id);
            %                     [row2, ~] = find("2", flightInfo.uas_id);
            %                     testCase.verifyTrue(flightInfo.classification(row1(end)),...
            %                         "Hobbist Two");
            %                     testCase.verifyTrue(flightInfo.classification(row2(end)),...
            %                         "normal");
            %                 end
            %                 count = count + 1;
            %             end
        end

        function hobbie2TwoFlightsSame(testCase)
            % hobbie2TwoFlightsSame - This test ensures that the hobbist 2
            % detection works if there are two hobbist 2 flights in the
            % same structure
            %             [waypoints1, timeOfArrival1] = ...
            %                 ClassificationTests.GenerateHobbistTwo(testCase);
            %             trajectory1 = waypointTrajectory(waypoints1, timeOfArrival1);
            %
            %             [waypoints1, timeOfArrival1] = ...
            %                 ClassificationTests.GenerateHobbistTwo(testCase);
            %             trajectory2 = waypointTrajectory(waypoints1, timeOfArrival1);
            %
            %             time = 0;
            %             count = 0;
            %
            %             while ~isDone(trajectory1) && ~isDone(trajectory2)
            %                 % Pull flight information - create a table with tel_info
            %                 [currentPosition, ~] = trajectory1();
            %                 [currentPosition2, ~] = trajectory2();
            %                 vel = [1,1,1];
            %                 telemetry = table("1", currentPosition, vel, time);
            %                 telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
            %                 name = "2";
            %                 pos = currentPosition2;
            %                 speed = randi(5, [1,3]);
            %                 telemetry{2, {'ID', 'pos', 'speed', 'time'}} ...
            %                     = [name, pos, speed, time];
            %                 radar = table("", [0,0,0], vel, time);
            %                 radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
            %                 time = time + .01;
            %                 testCase.monitor.AnalyzeFlights(telemetry, radar, [], .01);
            %                 if(count > 6)
            %                     flightInfo = testCase.monitor.flights;
            %                     [row1, ~] = find("1", flightInfo.uas_id);
            %                     [row2, ~] = find("2", flightInfo.uas_id);
            %                     testCase.verifyTrue(flightInfo.classification(row1(end)),...
            %                         "Hobbist Two");
            %                     testCase.verifyTrue(flightInfo.classification(row2(end)),...
            %                         "Hobbist Two");
            %                 end
            %                 count = count + 1;
            %             end

        end

        function multipleNormalFlights(testCase)
            % multipleNormalFlights - Runs multiple UAS Normal flights to
            % ensure that no Hobbist 2 detection happens.


        end

        function MultipleNormalWithOneHobbistFlight(testCase)
            % MultipleNormalWithOneHobbistFlight - Runs multiple UAS Normal
            % Flights with one Hobbist flight in the mixture. This is to
            % ensure that the track monitor classifies the Hobbist 2-flight
            % correctly.
        end

        function MultipleNormalWithTwoHobbistFlight(testCase)
            % MultipleNormalWithTwoHobbistFlight - Runs multiple UAS Normal
            % flights with two Hobbist flight in the mixture and ensure
            % that track monitor classifies the Hobbist 2 flight correctly.
        end

        function MultipleHobbistTwoFlights(testCase)
            % MultipleHobbistTwoFlights - Runs multiple UAS Hobbist 2
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end

    %% Hobby 3 Tests
    % This section is used to test flights that display Hobbyist Type 3:
    % Flies up in a circular motion to some highest point then flies down
    % in a circular motion to land.

    methods(Test)
        function noHobbie3Tests(testCase)
            % noHobbie3Tests - This test ensures that the Hobbist three
            % isn't detected with one normal UAS Flight
        end

        function hobbie3TestsSingle(testCase)
            % hobbie3TestsSingle - This test ensures that hobbist 3
            % detection for a single flight.
        end

        function hobbie3TestsDoubleFlightsSameTime(testCase)
            % hobbie3TestsDoubleFlightsSameTime - this test ensures that
            % the hobbist 3 detection correctly given the two UAS Flights
            % one hobbist 3 and the other normal.
        end

        function hobbie3TwoFlightsSame(testCase)
            % hobbie3TwoFlightsSame - This test ensures that the hobbist 3
            % detection works if there are two hobbist 3 flights in the
            % same structure
        end

        function multipleNormalFlightsHobbist3(testCase)
            % multipleNormalFlightsHobbist3 - Runs multiple UAS Normal flights to
            % ensure that no Hobbist 3 detection happens.
        end

        function MultipleNormalWithOneHobbist3Flight(testCase)
            % MultipleNormalWithOneHobbistFlight - Runs multiple UAS Normal
            % Flights with one Hobbist flight in the mixture. This is to
            % ensure that the track monitor classifies the Hobbist 2-flight
            % correctly.
        end

        function MultipleNormalWithTwoHobbist3Flight(testCase)
            % MultipleNormalWithTwoHobbist3Flight - Runs multiple UAS Normal
            % flights with two Hobbist flight in the mixture and ensure
            % that track monitor classifies the Hobbist 3 flight correctly.
        end

        function MultipleHobbistThreeFlights(testCase)
            % MultipleHobbistThreeFlights - Runs multiple UAS Hobbist 3
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end

    %% Rogue 1 Tests
    % This section is used to test flights that display Rogue Type 1: Flies
    % up over and down as for a delivery.

    methods(Test)
        function noRogue1Tests(testCase)
            % noRogue1Tests - This test ensures that the Rogue one
            % isn't detected with one normal UAS Flight
        end

        function rogue1TestsSingle(testCase)
            % rogue1TestsSingle - This test ensures that rogue 1
            % detection for a single flight.
        end

        function rogue1TestsDoubleFlightsSameTime(testCase)
            % rogue1TestsDoubleFlightsSameTime - this test ensures that
            % the rogue 1 detection correctly given the two UAS Flights
            % one rogue 1 and the other normal.
        end

        function rogueOneTwoFlightsSame(testCase)
            % rogueOneTwoFlightsSame - This test ensures that the rogue 1
            % detection works if there are two rogue 1 flights in the
            % same structure
        end

        function multipleNormalFlightsRogue1(testCase)
            % multipleNormalFlightsRogue1 - Runs multiple UAS Normal flights to
            % ensure that no Rogue 1 detection happens.
        end

        function MultipleNormalWithOneRogue1Flight(testCase)
            % MultipleNormalWithOneRogue1Flight - Runs multiple UAS Normal
            % Flights with one Hobbist flight in the mixture. This is to
            % ensure that the track monitor classifies the rogue 1 flight
            % correctly.
        end

        function MultipleNormalWithTwoRogue1Flight(testCase)
            % MultipleNormalWithTwoRogue1Flight - Runs multiple UAS Normal
            % flights with two Rogue 1 flight in the mixture and ensure
            % that track monitor classifies the rogue 1 flight correctly.
        end

        function MultipleRogue1Flights(testCase)
            % MultipleRogue1Flights - Runs multiple UAS Rogue 1
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end

    %% Rogue 2 Tests
    % This section is used to test flights that display Rogue Type 2: Flies
    % up to a lane, flies along the lane to the end, then flies to another
    % lane, and eventually flies down to land.

    methods(Test)
        function noRogue2Tests(testCase)
            % noRogue2Tests - This test ensures that the Rogue two
            % isn't detected with one normal UAS Flight
        end

        function rogue2TestsSingle(testCase)
            % rogue2TestsSingle - This test ensures that rogue 2
            % detection for a single flight.
        end

        function rogue2TestsDoubleFlightsSameTime(testCase)
            % rogue2TestsDoubleFlightsSameTime - this test ensures that
            % the rogue 2 detection correctly given the two UAS Flights
            % one rogue 2 and the other normal.
        end

        function rogueTwoTwoFlightsSame(testCase)
            % rogueTwoTwoFlightsSame - This test ensures that the rogue 2
            % detection works if there are two rogue 2 flights in the
            % same structure
        end

        function multipleNormalFlightsRogue2(testCase)
            % multipleNormalFlightsRogue2 - Runs multiple UAS Normal flights to
            % ensure that no Rogue 2 detection happens.
        end

        function MultipleNormalWithOneRogue2Flight(testCase)
            % MultipleNormalWithOneRogue2Flight - Runs multiple UAS Normal
            % Flights with one Rogue flight in the mixture. This is to
            % ensure that the track monitor classifies the rogue 2 flight
            % correctly.
        end

        function MultipleNormalWithTwoRogue2Flight(testCase)
            % MultipleNormalWithTwoRogue2Flight - Runs multiple UAS Normal
            % flights with two Rogue 2 flight in the mixture and ensure
            % that track monitor classifies the rogue 2 flight correctly.
        end

        function MultipleRogue2Flights(testCase)
            % MultipleRogue2Flights - Runs multiple UAS Rogue 2
            % flights within a simulation this is to ensure that all of the
            % flights are correctly detecting Hobbist flights.
        end
    end
end