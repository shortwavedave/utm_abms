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
        % Normal Flights
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
        function sim = getSuccessfulNormalFlight(sim, num_flights)
            uas_list = sim.getSuccessfulUAS();

            while(size(uas_list, 2) < num_flights)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = uas_list;
        end

        % Hobbist One
        function traj = LEM_hobby_type1(launch_site,height,radius,num_moves,...
            speed,del_t,ratio)
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
            
            % Generate sphere points
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
%             trajn = LEM_noisy_traj(traj,ratio,speed,del_t);
%             traj = trajn;
        end
        function traj = getSuccessfulHobbistOne(testCase)
            launchVert = testCase.lbsd.getRandLaunchVert();
            pt1 = testCase.lbsd.getVertPositions(launchVert);
            
            rogTraj = ClassificationTests.LEM_hobby_type1(pt1', 15, 5,10,...
                    1,1,0.5);

            while(~TrackMonitor.LEM_check_hobbist3(rogTraj))
                launchVert = testCase.lbsd.getRandLaunchVert();
                pt1 = testCase.lbsd.getVertPositions(launchVert);
                rogTraj = ClassificationTests.LEM_rogue_type3(pt1',15, 5, 10,...
                    1,0.1,0.5);
            end

            traj = rogTraj;
        end
        function runHobbistOneSimulation(testCase)
            % Create Rogue 1 Flight
            traj = ClassificationTests.getSuccessfulHobbistOne(testCase);

            % Run the Rogue Flight
            for index = 1:size(traj, 1)
                currentPosition = traj(index, 1:3);
                vel = [1,1,1];
                time = traj(index, 4);
                del_t = .1;
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], del_t);
            end

            flightInfo = testCase.monitor.flights;
            testCase.verifyEqual(flightInfo.classification(end), "Hobbist One");
        end

        % Hobbist Two
        function [waypoints, timeOfArrival] = GenerateHobbistTwo(testCase)
            launchVert = testCase.lbsd.getRandLaunchVert();
            pt1 = testCase.lbsd.getVertPositions(launchVert);
            pt1 = transpose(pt1);
            traj = ClassificationTests.LEM_hobby_type2(pt1,10,5,10,...
                1,0.1,10,50,0.5);
            waypoints = traj(:, 1:3);
            timeOfArrival = traj(:, 4);
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
%             trajn = ClassificationTests.LEM_noisy_traj(traj,ratio,speed,del_t);
%             traj = trajn;
        end
        function traj = getSuccessfulHobbistTwo(testCase)
            launchVert = testCase.lbsd.getRandLaunchVert();
            pt1 = testCase.lbsd.getVertPositions(launchVert);

            rogTraj = ClassificationTests.LEM_hobby_type2(pt1', 15, 5,10,...
                    1,.1,10, 50);

            while(~TrackMonitor.LEM_check_hobbist2(rogTraj))
                launchVert = testCase.lbsd.getRandLaunchVert();
                pt1 = testCase.lbsd.getVertPositions(launchVert);
                rogTraj = ClassificationTests.LEM_hobby_type2(pt1', 15, 5,10,...
                    1,.5,10, 50);
            end

            traj = rogTraj;
        end
        function runHobbistTwoSimulation(testCase)
            traj = ClassificationTests.getSuccessfulHobbistTwo(testCase);

            % Run the Rogue Flight
            for index = 1:size(traj, 1)
                currentPosition = traj(index, 1:3);
                vel = [1,1,1];
                time = traj(index, 4);
                del_t = .1;
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], del_t);
            end

            flightInfo = testCase.monitor.flights;
            testCase.verifyEqual(flightInfo.classification(end), "Hobbist Two");
        end
            
        % Hobbist Three
        function traj = LEM_hobby_type3(center,launch_site,height,speed,del_t,...
            ratio)
            traj = [launch_site'];
            radius = norm(center-launch_site);
            thetad = 0;
            x = launch_site(1);
            y = launch_site(2);
            z = launch_site(3);
            while z<height
                thetad = mod(thetad + 1,360);
                x = center(1) + radius*cosd(thetad);
                y = center(2) + radius*sind(thetad);
                z = z + speed*del_t;
                traj = [traj;x,y,z];
            end
            if z>height
                traj(end,3) = height;
            end
            
            while z>launch_site(3)
                thetad = mod(thetad + 1,360);
                x = center(1) + radius*cosd(thetad);
                y = center(2) + radius*sind(thetad);
                z = z - speed*del_t;
                traj = [traj;x,y,z];
            end
            if traj(end,3)<launch_site(3)
                traj(end,3) = launch_site(3);
            end
            
            num_pts = length(traj(:,1));
            traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
            trajn = LEM_noisy_traj(traj,ratio,speed,del_t);
            traj = trajn;
        end
        function traj = getSuccessfulHobbistThree(testCase)
            launchVert = testCase.lbsd.getRandLaunchVert();
            pt1 = testCase.lbsd.getVertPositions(launchVert);
            
            rogTraj = ClassificationTests.LEM_rogue_type3(pt1', pt1', 15, 1, .5, .5);

            while(~TrackMonitor.LEM_check_hobbist3(rogTraj))
                launchVert = testCase.lbsd.getRandLaunchVert();
                pt1 = testCase.lbsd.getVertPositions(launchVert);
                rogTraj = ClassificationTests.LEM_rogue_type3(pt1', pt1', 15, 1, .5, .5);
            end
            traj = rogTraj;
        end
        function runHobbistThreeSimulation(testCase)
            % Create Rogue 1 Flight
            traj = ClassificationTests.getSuccessfulRogueFlight(testCase);

            % Run the Rogue Flight
            for index = 1:size(traj, 1)
                currentPosition = traj(index, 1:3);
                vel = [1,1,1];
                time = traj(index, 4);
                del_t = .1;
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], del_t);
            end

            flightInfo = testCase.monitor.flights;
            testCase.verifyEqual(flightInfo.classification(end), "Hobbist Three");
        end

        % Rogue One
        function traj = GenerateRogueOne(testCase)
            % GenerateRogueOne - Generates a Waypoint trajectory of Rogue
            % One flight.
            % Input:
            %   testCase (testing handle)
            % Output:
            %   traj (waypoint trajectory) : traj of Rogue 1.
            %
            launchVert = testCase.lbsd.getRandLaunchVert();
            pt1 = testCase.lbsd.getVertPositions(launchVert);
            landVert = testCase.lbsd.getRandLandVert();
            pt2 = testCase.lbsd.getVertPositions(landVert);
            pt3 = [pt1(1:2) 15];
            pt4 = [pt2(1:2) 15];
            dist = norm(pt1 - pt2);
            waypoints = [pt1; pt3; pt4; pt2];
            timeOfArrival = [0; norm(pt3 - pt1)/5; ...
                norm(pt3 - pt1)/5 + dist/5; norm(pt3 - pt1)/5 + dist/5 + norm(pt4 - pt2)/5];
            traj = waypointTrajectory(Waypoints=waypoints, TimeOfArrival=timeOfArrival);
        end
        function traj = LEM_rogue_type1(launch_site,land_site,height,speed,del_t,...
                ratio)
            % LEM_rogue_type1 - up, over & down delivery
            % On input:
            %     launch_site (3x1 vector): ground launch point
            %     land_site (3x1 vector): ground land point
            %     height (float): altitude for center of sphere
            %     speed (float): speed of UAS
            %     del_t (float): time sample interval
            %     ratio (float): determines radius of error circle for traj noise
            %       radius is the inter-point distance * ratio
            % On ouput:
            %     traj (nx4 array): x,y,z,t
            % Call:
            %     traja = LEM_rogue_type1([x1;y1;z1],[x2;y2;z2],5,10,1,0.1,0.5);
            % Author:
            %     T. Henderson
            %     UU
            %     Spring 2021
            %

            % up
            pt1 = launch_site;
            pt2 = [pt1(1);pt1(2);pt1(3)+height];
            pt3 = land_site;
            pt3(3) = pt2(3);
            pt4 = land_site;

            dist12 = norm(pt2-pt1);
            dir = [0;0;1];
            pt = pt1;
            traj = [pt'];
            while norm(pt1-pt)<dist12
                pt = pt + dir*del_t;
                traj = [traj;pt'];
            end
            if norm(pt1-pt)>dist12
                traj(end,:) = pt2';
            end

            % across
            dist23 = norm(pt3-pt2);
            dir = pt3 - pt2;
            dir = dir / norm(dir);
            pt = traj(end,:)';
            while norm(pt2-pt)<dist23
                % r = ro + dir * del_t
                pt = pt + speed*dir*del_t;
                traj = [traj;pt'];
            end
            if norm(pt2-pt)>dist23
                traj(end,:) = pt3';
            end

            % down
            dist34 = norm(pt4-pt3);
            dir = pt4 - pt3;
            dir = dir/norm(dir);
            pt = traj(end,:)';
            while norm(pt-pt3)<dist34
                pt = pt + speed*del_t*dir;
                traj = [traj;pt'];
            end
            if norm(pt3-pt)>dist34
                traj(end,:) = pt4';
            end
            num_pts = length(traj(:,1));
            traj = [traj,[0:del_t:(num_pts-1)*del_t]'];
%             trajn = ClassificationTests.LEM_noisy_traj(traj,ratio,speed,del_t);
%             traj = trajn;

            tch = 0;
        end
        function traj = getSuccessfulRogueFlight(testCase)
            launchVert = testCase.lbsd.getRandLaunchVert();
            pt1 = testCase.lbsd.getVertPositions(launchVert);
            landVert = testCase.lbsd.getRandLandVert();
            pt2 = testCase.lbsd.getVertPositions(landVert);
            rogTraj = ClassificationTests.LEM_rogue_type1(pt1', pt2', 15, 1, .5, .5);

            while(~TrackMonitor.LEM_check_rogue(rogTraj))
                launchVert = testCase.lbsd.getRandLaunchVert();
                pt1 = testCase.lbsd.getVertPositions(launchVert);
                landVert = testCase.lbsd.getRandLandVert();
                pt2 = testCase.lbsd.getVertPositions(landVert);
                rogTraj = ClassificationTests.LEM_rogue_type1(pt1', pt2', 15, 1, .5, .5);
            end
            traj = rogTraj;
        end
        function runRogue1TrajSimulation(testCase)
            % Create Rogue 1 Flight
            traj = ClassificationTests.getSuccessfulRogueFlight(testCase);

            % Run the Rogue Flight
            for index = 1:size(traj, 1)
                currentPosition = traj(index, 1:3);
                vel = [1,1,1];
                time = traj(index, 4);
                del_t = .1;
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], del_t);
            end
            for i = 1:3
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                telemetry = radar;
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], .1);
            end
            flightInfo = testCase.monitor.flights;
            testCase.verifyEqual(flightInfo.classification(end), "Rogue One");
        end

        % Helper Functions
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
        % Single Flight Without Noise - Multiple Random Seeds
        function SingleNormalFlightRngZero(testCase)
            % SingleNormalFlight - This test a single flight that contains
            % no noise to ensure that track monitor will classify it as
            % normal.
            rng(0);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 1)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end
            uas = uas_list(1);
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
                    end
                end
            end

            flightInfo = testCase.monitor.flights;
            if(~isempty(flightInfo.classification))
                testCase.verifyEqual(flightInfo.classification(end), "normal");
            end
        end
        function SingleNormalFlightRngOne(testCase)
            % SingleNormalFlightRngOne - This test is to ensure that a
            % single flight without noise is classified correctly
            % regardless of randomization.
            rng(1);
            testCase.createTrackMonitor();
            [sim, ~, ~] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 1)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end
            uas = uas_list(1);
            sim.uas_list = uas;

            while(~isDone(uas.traj))
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
                    end
                end
                flightInfo = testCase.monitor.flights;
                if(~isempty(flightInfo.classification))
                    testCase.verifyEqual(flightInfo.classification(end), "normal");
                end
            end
        end
        function SingleNormalFlightRngTwo(testCase)
            % SingleNormalFlightRngTwo - This test is to ensure that the
            % Track Monitor can classified the normal flight without noise
            % regardless of the random seed.
            rng(2);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 1)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end
            sim.uas_list = uas_list(1);
            uas = sim.uas_list(1);

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
                    end
                end
            end
            flightInfo = testCase.monitor.flights;
            if(~isempty(flightInfo.classification))
                testCase.verifyEqual(flightInfo.classification(end), "normal");
            end
        end
        function SingleNormalFlightRngThree(testCase)
            % SingleNormalFlightRngThree - This test is to ensure that the
            % Track Monitor will classify a normal flight without noise
            % regardless of the random seed.
            rng(3);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 1)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end
            uas = uas_list(1);
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
                    end
                end
            end
            flightInfo = testCase.monitor.flights;
            if(~isempty(flightInfo.classification))
                testCase.verifyEqual(flightInfo.classification(end), "normal");
            end
        end
        function SingleNormalFlightRngFour(testCase)
            % SingleNormalFlightRngFour - This test ensures that the track
            % monitor will classify the normal flight without noise
            % regardless of random seed number.
            rng(4);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 1)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end
            uas = uas_list(1);
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
       
                    end
                end
            end
            flightInfo = testCase.monitor.flights;
            if(~isempty(flightInfo.classification))
                testCase.verifyEqual(flightInfo.classification(end), "normal");
            end
        end

        % Two Flight Without Noise - Multiple Random Seeds
        function twoSingleNormalFlightsRngZero(testCase)
            % twoSingleNormalFlights - Runs through two Normal flights and
            % tests to ensure that it classifies two normal flights in the
            % air system.
            rng(0);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 2)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = [uas_list(1), uas_list(2)];

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
                    uas = sim.uas_list(j);
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
        end
        function twoSingleNormalFlightsRngTwo(testCase)
            % twoSingleNormalFlights - Runs through two Normal flights and
            % tests to ensure that it classifies two normal flights in the
            % air system.
            rng(2);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 2)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = [uas_list(1), uas_list(2)];

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
                    uas = sim.uas_list(j);
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
        end
        function twoSingleNormalFlightsRngFour(testCase)
            % twoSingleNormalFlightsRngFour - Runs through two Normal flights and
            % tests to ensure that it classifies two normal flights in the
            % air system.
            rng(4);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 2)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = [uas_list(1), uas_list(2)];

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
                    uas = sim.uas_list(j);
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
        end
        function twoSingleNormalFlightsRngSix(testCase)
            % twoSingleNormalFlights - Runs through two Normal flights and
            % tests to ensure that it classifies two normal flights in the
            % air system.
            rng(6);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 2)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = [uas_list(1), uas_list(2)];

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
                    uas = sim.uas_list(j);
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
        end

        % Multiple Flight Without Noise - Multiple Random Seeds
        function multipleSingleNormalFlightRngOne(testCase)
            % multipleSingleNormalFlight - This test ensures that if
            % multiple normal flights are classified as normal.
            rng(1);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 3)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = uas_list;

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
                    uas = sim.uas_list(j);
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
        end
        function multipleSingleNormalFlightRngThree(testCase)
            % multipleSingleNormalFlight - This test ensures that if
            % multiple normal flights are classified as normal.
            rng(3);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 3)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = uas_list;

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
                    uas = sim.uas_list(j);
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
        end
        function multipleSingleNormalFlightRngFive(testCase)
            % multipleSingleNormalFlight - This test ensures that if
            % multiple normal flights are classified as normal.
            rng(5);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 3)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = uas_list;

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
                    uas = sim.uas_list(j);
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
        end
        function multipleSingleNormalFlightRngSeven(testCase)
            % multipleSingleNormalFlight - This test ensures that if
            % multiple normal flights are classified as normal.
            rng(7);
            testCase.createTrackMonitor();
            [sim, ~, num_steps] = ...
                ClassificationTests.setUpSimulationFlights(testCase);
            testCase.lbsd = sim.lbsd;
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            uas_list = sim.getSuccessfulUAS();
            while(size(uas_list, 2) < 3)
                sim.initialize();
                uas_list = sim.getSuccessfulUAS;
            end

            sim.uas_list = uas_list;

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
                    uas = sim.uas_list(j);
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
        end

        % Single Flight with Noise - Multiple Random Seeds
        function aSingleNormalFlightWithSlightNoiseRngOne(testCase)
            % aSingleNormalFlightWithSlightNoise - This test is to run a
            % single flight that contains some noisy flight.
            rng(1);
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
                        uas.gps.lon = pos(uas_step, 1)+ rand()*.01;
                        uas.gps.lat = pos(uas_step, 2)+ rand()*.01;
                        uas.gps.alt = pos(uas_step, 3)+ rand()*.01;
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
        function aSingleNormalFlightWithSlightNoiseRngSix(testCase)
            % aSingleNormalFlightWithSlightNoise - This test is to run a
            % single flight that contains some noisy flight.
            rng(6);
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
                        uas.gps.lon = pos(uas_step, 1)+ rand()*.01;
                        uas.gps.lat = pos(uas_step, 2)+ rand()*.01;
                        uas.gps.alt = pos(uas_step, 3)+ rand()*.01;
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
        function aSingleNormalFlightWithSlightNoiseRngTen(testCase)
            % aSingleNormalFlightWithSlightNoise - This test is to run a
            % single flight that contains some noisy flight.
            rng(10);
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
                        uas.gps.lon = pos(uas_step, 1)+ rand()*.01;
                        uas.gps.lat = pos(uas_step, 2)+ rand()*.01;
                        uas.gps.alt = pos(uas_step, 3)+ rand()*.01;
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
        function aSingleNormalFlightWithSlightNoiseRng14(testCase)
            % aSingleNormalFlightWithSlightNoise - This test is to run a
            % single flight that contains some noisy flight.
            rng(14);
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
                        uas.gps.lon = pos(uas_step, 1)+ rand()*.01;
                        uas.gps.lat = pos(uas_step, 2)+ rand()*.01;
                        uas.gps.alt = pos(uas_step, 3)+ rand()*.01;
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

        % Two Flight with Noise - Multiple Random Seeds
        function twoSingleNormalFlightsWithNoiseRngOne(testCase)
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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
        function twoSingleNormalFlightWithNoiseRngFive(testCase)
            % twoSingleNormalFlightsWithNoise - This test ensures that two
            % flights with noise are still classified as Normal.
            rng(5);
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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
        function twoSingleNormalFlightWithNoiseRngNine(testCase)
            % twoSingleNormalFlightsWithNoise - This test ensures that two
            % flights with noise are still classified as Normal.
            rng(9);
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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
        function twoSingleNormalFlightWithNoiseRng13(testCase)
            % twoSingleNormalFlightsWithNoise - This test ensures that two
            % flights with noise are still classified as Normal.
            rng(13);
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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

        % Multiple Flight With Noise - Multiple Random Seeds
        function multipleSingleNormalFlightWithNoiseRng1(testCase)
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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
        function multipleSingleNormalFlightWithNoiseRngEight(testCase)
            % multipleSingleNormalFlightWithNoise - This test checks to run
            % multiple flights with slight noise that ensures that all of
            % these flights are classified as Normal.
            rng(8);
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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
        function multipleSingleNormalFlightWithNoiseRng16(testCase)
            % multipleSingleNormalFlightWithNoise - This test checks to run
            % multiple flights with slight noise that ensures that all of
            % these flights are classified as Normal.
            rng(16);
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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
        function multipleSingleNormalFlightWithNoiseRng32(testCase)
            % multipleSingleNormalFlightWithNoise - This test checks to run
            % multiple flights with slight noise that ensures that all of
            % these flights are classified as Normal.
            rng(32);
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
                    uas = sim.uas_list(j);
                    uas_step = uas.stepTrajectory();
                    if uas.active
                        pos = uas.exec_traj;
                        if ~isempty(pos)
                            uas.gps.lon = pos(uas_step, 1) + rand()*.01;
                            uas.gps.lat = pos(uas_step, 2) + rand()*.01;
                            uas.gps.alt = pos(uas_step, 3) + rand()*.01;
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

    %% Hobbyist Type 1:
    % This section is used to test flights that display Hobbyist Type 1:
    % Flies up from one place and makes a few moves above the launch site,
    % then eventually lands at the same site.

%     methods(Test)
%         % Single Hobbist One Flight Random Seed
%         function SingleFlightHobbistOneRngZero(testCase)
%             % SingleFlightHobbistOneRngZero - This test is to ensure that
%             % Hobbist One flights are classified correctly regardless of
%             % the random seed. 
%             rng(0);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function SingleFlightHobbistOneRngOne(testCase)
%             % SingleFlightHobbistOneRngOne - This test ensures that Track
%             % Monitor is correctly classifing Hobbist One Regardless of the
%             % random seed. 
%             rng(1);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function SingleFlightHobbistOneRngTwo(testCase)
%             % SingleFlightHobbistOneRngTwo - This test ensures that Track
%             % Monitor Correct Classifies Hobbist One flight behavior
%             % regardless of random seed.
%             rng(2);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
% 
%         end
%         function SingleFlightHobbistOneRngThree(testCase)
%             % SingleFlightHobbistOneRngThree - This test ensures that the
%             % Track Monitor correctly classifies the Hobbist One Flight
%             % Regardless of the random seed. 
%             rng(3);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%     
%         % Two Hobbist One Flight Random Seed
%         function TwoFlightHobbistOneRngFour(testCase)
%             % TwoFlightHobbistOneRngFour - This test ensures that Hobbist
%             % ONe flights are classified correctly regardless of Random
%             % seed. 
%             rng(4);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function TwoFlightHobbistOneRngFive(testCase)
%             % TwoFlightHobbistOneRngFive - This test ensures that hobbist
%             % one flight is correctly identified regardless of random seed.
%             rng(5);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function TwoFlightHobbistOneRngSix(testCase)
%             % TwoFlightHobbistOneRngSix - Ensures that Track Monitor
%             % correctly identifies Hobbist One Regardless of the Random
%             % Seed. 
%             rng(6);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function TwoFlightHobbistOneRngSeven(testCase)
%             % TwoFlightHobbistOneRngSeven - This test to ensure that They
%             % correctly identify Hobbist One Flights regardless of Random
%             % Seed. 
%             rng(7);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%     
%         % Multiple Hobbist One Flight Random Seed
%         function MultipleFlightHobbistOneRngEight(testCase)
%             % MultipleFlightHobbistOneRngEight - This Test ensures that
%             % through multiple flights that are hobbist one that they
%             % correctly identified regardless of the random seed. 
%             rng(8);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function MultipleFlightHobbistOneRngNine(testCase)
%             % MultipleFlightHobbistOneRngNine - This test ensures that
%             % through multiple Hobbist One Flights it will correctly
%             % identifies. 
%             rng(9);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function MultipleFlightHobbistOneRngTen(testCase)
%             % MultipleFlightHobbistOneRngTen - This test ensures that
%             % multiple hobbist one flights are correctly classifed as
%             % Hobbist one. 
%             rng(10);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%         function MultipleFlightHobbistOneRngEleven(testCase)
%             % MultipleFlightHobbistOneRngEleven - This test ensures through
%             % multiple hobbist one flights are classified correctly
%             % regardless of random seed. 
%             rng(11);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistOneSimulation(testCase);
%             end
%         end
%     end

    %% Hobby 2 Tests
    % This section is used to test flights that display Hobbyist Type 2:
    % Flies up from one place and makes a few moves above the launch site,
    % hovers after each move, then eventually lands at the launch site.

    methods(Test)
        % Single Flights
        function SingleHobbistTwoRngZero(testCase)
            % SingleHobbistTwoRngZero - This test is used to ensure that
            % the Hobbist two will correctly identify Hobbist two flights
            % regardless of random seed. 
            rng(0);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:1
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function SingleHobbistTwoRngOne(testCase)
            % SingleHobbistTwoRngOne - This test is to ensure that the
            % hobbist two flights are correctly identified regardless of
            % random seed. 
            rng(1);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:1
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function SingleHobbistTwoRngTwo(testCase)
            % SingleHobbistTwoRngTwo - This test is to ensure that the
            % hobbist two flights are correctly identified regardless of
            % random seed. 
            rng(0);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:1
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function SingleHobbistTwoRngThree(testCase)
            % SingleHobbistTwoRngThree - This test ensure that the hobbist
            % two flights are correctly identified regardless of random
            % seed. 
            rng(0);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:1
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
    
        % Two Flights
        function DoubleHobbistTwoRngFour(testCase)
            % DoubleHobbistTwoRngFour - this test is to ensure that two
            % hobbist two flights are correctly identified regardless of
            % random seed. 
            rng(4);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:2
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function DoubleHobbistTwoRngFive(testCase)
            % DoubleHobbistTwoRngFive - this test is to ensure that two
            % hobbist two flights are correctly identified regardless of
            % random seed. 
            rng(5);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:2
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function DoubleHobbistTwoRngSix(testCase)
            % DoubleHobbistTwoRngSix - This test is to ensure that two
            % hobbist two flights are correctly identified regardless of
            % random seed. 
            rng(6);
            testCase.createTrackMonitor();
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:2
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function DoubleHobbistTwoRngSeven(testCase)
            % DoubleHobbistTwoRngSeven - This test is to ensure that two
            % hobbist two flights are correctly identified regardless of
            % random seed. 
            rng(7);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:2
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end

        % Multiple Flights
        function MultipleHobbistTwoRngEight(testCase)
            % MultipleHobbistTwoRngEight - This test is to ensure that
            % multiple hobbist two flights are correctly identified
            % regardless of random seed. 
            rng(8);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:10
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function MultipleHobbistTwoRngNine(testCase)
            % MultipleHobbistTwoRngNine - This test is to ensure that
            % multiple hobbist two flights are correctly identified
            % regardless of random seed. 
            rng(9);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:10
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function MultipleHobbistTwoRngTen(testCase)
            % MultipleHobbistTwoRngTen - This test is to ensure that
            % multiple hobbist two flights are correctly identified
            % regardless of random seed. 
            rng(10);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:10
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
        function MultipleHobbistTwoRngEleven(testCase)
            % MultipleHobbistTwoRngEleven - This test is to ensure that
            % multiple hobbist two flights are correctly identified
            % regardless of random seed. 
            rng(11);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            for i = 1:10
                ClassificationTests.runHobbistTwoSimulation(testCase);
            end
        end
    end

    %% Hobby 3 Tests
    % This section is used to test flights that display Hobbyist Type 3:
    % Flies up in a circular motion to some highest point then flies down
    % in a circular motion to land.

%     methods(Test)
%         % Single Hobbist Three Flight Random Seeds
%         function SingleHobbistThreeRngZero(testCase)
%             % SingleHobbistThreeRngZero - Ensures that the Hobbist Three is
%             % correctly identified by the Track Monitor class.
%             rng(0);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function SingleHobbistThreeRngOne(testCase)
%             % SingleHobbistThreeRngOne - Test to ensure that hobbist three
%             % are correctly identifies Hobbist Three Behavior for a single
%             % flight regardless of random seed. 
%             rng(1);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function SingleHobbistThreeRngTwo(testCase)
%             % SingleHobbistThreeRngTwo - This test ensures that a single
%             % Hobbist three is correctly identified regarless of the random
%             % seed. 
%             rng(2);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function SingleHobbistThreeRngFour(testCase)
%             % SingleHobbistThreeRngFour - This test ensures that Single
%             % Hobbist Three Flight Regardless of random seed.
%             rng(4);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:1
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%     
%         % Two Hobbist Three Flight Random Seeds
%         function TwoHobbistThreeRngFive(testCase)
%             % TwoHobbistThreeRngFive - Tests two ensure that two flights of
%             % Hobbist Three are correctly identified regardless of the
%             % random seed. 
%             rng(5);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function TwoHobbistThreeRngSix(testCase)
%             % TwoHobbistThreeRngSix - This test ensures that hobbist three
%             % flights are correctly identified regardless of the random
%             % seed. 
%             rng(6);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function TwoHobbistThreeRngSeven(testCase)
%             % TwoHobbistThreeRngSeven - This test ensures that Track
%             % Monitor will correctly identified Hobbist Three flights. 
%             rng(7);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function TwoHobbistThreeRngEight(testCase)
%             % TwoHobbistThreeRngEight - This test ensures that the two
%             % hobbist three flights are correctly identify regardless of
%             % random seed. 
%             rng(8);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:2
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%     
%         % Multiple Hobbist Three Flights Multiple Random Seeds
%         function MultipleHobbistThreeRngNine(testCase)
%             % MultipleHobbistThreeRngNine - This test ensures that multiple
%             % hobbist flights are classified correctly regardless of random
%             % seed. 
%             rng(9);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function MultipleHobbistThreeRngTen(testCase)
%             % MultipleHobbistThreeRngTen - Ensures that multiple hobbist
%             % three flights regardless of random seed. 
%             rng(10);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function MultipleHobbistThreeRngEleven(testCase)
%             % MultipleHobbistThreeRngEleven - This test ensures that
%             % multiple hobbist three flights regardless of random seed.
%             rng(11);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%         function MultipleHobbistThreeRngTwelve(testCase)
%             % MultipleHobbistThreeRngTwelve - This test ensures that
%             % Multiple Hobbist Three flights are correctly identified
%             % regardless of the random seed. 
%             rng(12);
%             testCase.createTrackMonitor();
%             testCase.createLBSD();
%             testCase.monitor.initializeLaneStructor(testCase.lbsd);
% 
%             % Create Rogue 1 Flight
%             for i = 1:10
%                 ClassificationTests.runHobbistThreeSimulation(testCase);
%             end
%         end
%     end

    %% Rogue 1 Tests
    % This section is used to test flights that display Rogue Type 1: Flies
    % up over and down as for a delivery.

    methods(Test)
        % Single Flight's With Random Seed Values
        function rogue1TestsSingleRngOne(testCase)
            % rogue1TestsSingle - This test ensures that rogue 1
            % detection for a single flight.
            rng(0);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            traj = ClassificationTests.getSuccessfulRogueFlight(testCase);
            
            for index = 1:size(traj, 1)
                currentPosition = traj(index, 1:3);
                vel = [1,1,1];
                time = traj(index, 4);
                del_t = .1;
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], del_t);
            end

            flightInfo = testCase.monitor.flights;
            testCase.verifyEqual(flightInfo.classification(end),...
                "Rogue One");
        end
        function rogue1TestsSingleRngThree(testCase)
            % rogue1TestsSingleRngThree - This test ensures that rogue 1
            % detection for a single flight is classified correctly
            % regardless of random seed
            rng(3);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            traj = ClassificationTests.getSuccessfulRogueFlight(testCase);
            
            for index = 1:size(traj, 1)
                currentPosition = traj(index, 1:3);
                vel = [1,1,1];
                time = traj(index, 4);
                del_t = .1;
                telemetry = table("1", currentPosition, vel, time);
                telemetry.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                radar = table("", [0,0,0], vel, time);
                radar.Properties.VariableNames = ["ID", "pos", "speed", "time"];
                testCase.monitor.AnalyzeFlights(telemetry, radar, [], del_t);
            end

            flightInfo = testCase.monitor.flights;
            testCase.verifyEqual(flightInfo.classification(end),...
                "Rogue One");
        end
        function rogue1TestsSingleRngTwo(testCase)
            % rogue1TestsSingleRngNine - This test ensures that rogue 1
            % detection for a single flight is classified correctly
            % regardless of random seed.
            rng(2);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            ClassificationTests.runRogue1TrajSimulation(testCase);
        end
        function rogue1TestsSingleRng11(testCase)
            % rogue1TestsSingleRng12 - This test ensures that rogue 1
            % detection for a single flight is classified correctly
            % regardless of random seed.
            rng(11);
            testCase.createLBSD();
            testCase.createTrackMonitor();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);
            ClassificationTests.runRogue1TrajSimulation(testCase);
        end

        % Two Rogue Flights Multiple Random seed values
        function rogue2FlightRngThree(testCase)
            % rogue2FlightRngThree - This test ensures that if two rogue 1
            % flights are happening that they are marked correctly
            % regardless of the random seed.
            rng(3);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:2
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
        function rogue2FlightRngSeven(testCase)
            % rogue2FlightRngSeven - This test is check if the track
            % monitor is correctly identify Rogue 1 behavior regardless of
            % random seed.
            rng(7);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:2
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
        function rogue2FlightRngEleven(testCase)
            % rogue2FlightRngEleven - This test ensure Track Monitor
            % classifies two rogue 1 flights correctly regardless random
            % seed.
            rng(11);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:2
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
        function rogue2FlightRngEightteen(testCase)
            % rogue2FlightRngEightteen - This test ensures that Track
            % Monitor classifies two rogue 1 flights regardless of the
            % random seed. 
            rng(18);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:2
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
        
        % Multiple Rogue Flights Multiple Random Seed Values
        function multipleRogue1FlightsRngFive(testCase)
            % multipleNormalFlightsRogue1 - Runs multiple UAS Normal flights to
            % ensure that no Rogue 1 detection happens.
            rng(5);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:10
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
        function multipleRogue1FlightsRngTen(testCase)
            rng(10);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:10
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
        function multipleRogue1RngTwenty(testCase)
            % multipleRogue1RngTwenty - This test ensures that through
            % multiple rogue 1 flights the track montior correctly
            % identifies the rogue flight, regardless of random seed.
            rng(20);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:10
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
        function multipleRogue1RngThirty(testCase)
            % multipleRogue1RngThirty - This test ensures that the all of
            % the rogue one flights correctly regardless of the random
            % seed.
            rng(30);
            testCase.createTrackMonitor();
            testCase.createLBSD();
            testCase.monitor.initializeLaneStructor(testCase.lbsd);

            % Create Rogue 1 Flight
            for i = 1:10
                ClassificationTests.runRogue1TrajSimulation(testCase);
            end
        end
    end

    %% Rogue 2 Tests
    % This section is used to test flights that display Rogue Type 2: Flies
    % up to a lane, flies along the lane to the end, then flies to another
    % lane, and eventually flies down to land.

    methods(Test)
        
    end
end