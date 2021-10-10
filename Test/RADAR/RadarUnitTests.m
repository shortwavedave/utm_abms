classdef RadarUnitTests < matlab.unittest.TestCase
    %RADARUNITTESTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    %% Helper Methods
    % These methods are used to set up different components for the other
    % testing
    methods (Static)
        function lbsd = LBSDSetup()
            % Set up the Lane System
            lbsd = LBSD.genSampleLanes(10, 15);
            start_time = 0;
            end_time = 100;
            lane_ids = ["1","2","3"];
            num_res = 50;
            speed = 1;
            headway = 5;
            lbsd.genRandReservations(start_time, end_time, ...
                num_res, lane_ids, speed, headway);
        end
        
        function uas = CreateUAS(pos, id)
            uas = UAS(id);
            uas.gps.lan = pos(1);
            uas.gps.lon = pos(2);
            uas.gps.alt = pos(3);
        end
    end
    
    %% Create Data Structure Tests
    % These tests are to check to see if the data structures are being
    %   intialized correctly. 
    % Data Structures:
    %   location: the location of the radar
    %   targets: shows the targets that were picked up during scanning
    %   range: The maximum range the radar field has
    %   apexAngle: The angle between the middle and side of the radar
    %   dirVector: The direction of the radar field
    %   lbsd: The handle to the lane based system
    methods(Test)
        function creationOfRadar(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 30, pi/4, [0,0,1], "1", lbsd);
            testCase.verifyNotEmpty(radar.location);
            testCase.verfiyNotEmpty(radar.range);
            testCase.verfiyNotEmpty(radar.apexAngle);
            testCase.verfiyEmpty(radar.targets);
            testCase.verfiyNotEmpty(radar.lbsd);
            testCase.verifyEqual([0,0,0], radar.location);
            testCase.verifyNotEmpty(radar.ID);
            testCase.verifyNotEmpty(radar.dirVector);
        end
        
        function creationOfRadar2(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([10,0,0], 30, pi/4, [1,0,1], "2", lbsd);
            testCase.verifyNotEmpty(radar.location);
            testCase.verfiyNotEmpty(radar.range);
            testCase.verfiyNotEmpty(radar.apexAngle);
            testCase.verfiyEmpty(radar.targets);
            testCase.verfiyNotEmpty(radar.lbsd);
            testCase.verifyEqual([10,0,0], radar.location);
            testCase.verifyNotEmpty(radar.ID);
            testCase.verifyNotEmpty(radar.dirVector);
        end
    end
    %% Scanning Method Tests On Vertical Radar Direction
    % Checks to see if scaning method on vertical Radars are working
    %   properly. Many of the tests are checking uas being in and out of
    %   radar field. 
    methods(Test)
        function NoUASVerticalRadarFieldNotCloseAtAll(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 30, pi/4, [0,0,1], "1", lbsd);
            uas = RadarUnitTests.CreateUAS([400, 400, 10], "1");
            radar.scan(uas);
            testCase.verifyEmpty(radar.targets);
        end
        
        function NoUASVerticalRadarFieldClose(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [0,0,1], "1", lbsd);
            radius = 10*tand(pi/4);
            uas = RadarUnitTests.CreateUAS([radius+1, 0, 10], "1");
            radar.scan(uas);
            testCase.verifyEmpty(radar.targets);
        end
        
        function OneUASVerticalRadarFieldClose(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [0,0,1], "1", lbsd);
            radius = 10*tand(pi/4);
            uas = RadarUnitTests.CreateUAS([radius, 0, 10], "1");
            radar.scan(uas);
            testCase.verifyNotEmpty(radar.targets);
        end
        
        function OneUASVerticalRadarFieldMiddle(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [0,0,1], "1", lbsd);
            uas = RadarUnitTests.CreateUAS([0, 0, 10], "1");
            radar.scan(uas);
            testCase.verifyNotEmpty(radar.targets);
        end
        
        function EnteringAndLeavingUASInRadarField(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [0,0,1], "1", lbsd);
            radius = 10*tand(pi/4);
            uas = RadarUnitTests.CreateUAS([0, 0, 10], "1");
            radar.scan(uas);
            testCase.verifyNotEmpty(radar.targets);
            pos = [radius + 1, radius + 1, 10];
            uas.gps.lan = pos(1);
            uas.gps.lon = pos(2);
            uas.gps.alt = pos(3);
            radar.scan(uas);
            testCase.verifyEmpty(radar.targets);
        end
    end
    %% Scanning Method Tests On Horizontal Radar Direction
    % Checks to see if scaning method on horizontal Radars are working
    %   properly. Many of the tests are checking uas being in and out of
    %   radar field. 
    methods(Test)
        function NoUASHorizontalRadarFieldNotCloseAtAll(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 30, pi/4, [1,0,0], "1", lbsd);
            uas = RadarUnitTests.CreateUAS([400, 400, 10], "1");
            radar.scan(uas);
            testCase.verifyEmpty(radar.targets);
        end
        
        function NoUASHorizontalRadarFieldClose(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [1,0,0], "1", lbsd);
            radius = 10*tand(pi/4);
            uas = RadarUnitTests.CreateUAS([10, 0, radius+1], "1");
            radar.scan(uas);
            testCase.verifyEmpty(radar.targets);
        end
        
        function OneUASHorizontalRadarFieldClose(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [1,0,0], "1", lbsd);
            radius = 10*tand(pi/4);
            uas = RadarUnitTests.CreateUAS([10, 0, radius], "1");
            radar.scan(uas);
            testCase.verifyNotEmpty(radar.targets);
        end
        
        function OneUASHorizontalRadarFieldMiddle(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [1,0,0], "1", lbsd);
            uas = RadarUnitTests.CreateUAS([8, 0, radius/4], "1");
            radar.scan(uas);
            testCase.verifyNotEmpty(radar.targets);
        end
        
        function EnteringAndLeavingHorizontalUASInRadarField(testCase)
            lbsd = RadarUnitTests.LBSDSetup();
            radar = RADAR([0,0,0], 10, pi/4, [1,0,0], "1", lbsd);
            radius = 10*tand(pi/4);
            uas = RadarUnitTests.CreateUAS([8, 0, radius/4], "1");
            radar.scan(uas);
            testCase.verifyNotEmpty(radar.targets);
            pos = [range, range, range];
            uas.gps.lan = pos(1);
            uas.gps.lon = pos(2);
            uas.gps.alt = pos(3);
            radar.scan(uas);
            testCase.verifyEmpty(radar.targets);
        end
    end
end

